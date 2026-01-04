#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "driver/adc_types_legacy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "AT24C01.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"

static const char *TAG = "BT_SPP_EXAMPLE";

#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "ESP32_MATHEUS"

static esp_spp_cb_t *spp_callback = NULL;

#define BUTTON_GPIO  GPIO_NUM_0
#define LED_GPIO     GPIO_NUM_2

#define ALARM_GPIO   GPIO_NUM_16

#define ADC_CHANNEL ADC2_CHANNEL_0
#define ADC_ATTEN_12 ADC_ATTEN_DB_12  
#define ADC_RESOLUTION ADC_BITWIDTH_12

#define ON 1
#define OFF 0

//By design, the ADC reference voltage for ESP32 is 1100 mV
#define ADC_REF 1100 

static const char *TAG1 = "LEITURA";
static const char *TAG2 = "ESCRITA EEPROM:";
static const char *TAG3 = "LEITURA EEPROM:";

static uint32_t g_spp_handle = 0;

TaskHandle_t ADCHandle;
TaskHandle_t MemoryHandle;
TaskHandle_t AlarmHandle;

QueueHandle_t MemoryQueue;
QueueHandle_t AlarmQueue;

uint16_t voltage_threshold;

static void spp_event_handler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        // após inicializar, coloque visível e sincronize
        //esp_bt_dev_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT, server started, ready to accept");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
    	g_spp_handle = param->srv_open.handle;
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT: Client connected");
        break;

    case ESP_SPP_CLOSE_EVT:
    	g_spp_handle = 0;
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT: Connection closed");

        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT: len=%d handle=%d",
                 (unsigned int)param->data_ind.len, (unsigned int)param->data_ind.handle);
        // processa dados recebidos
        {
            int len = param->data_ind.len;
            char *data = (char *)malloc(len + 1);
            if (data) {
                memcpy(data, param->data_ind.data, len);
                data[len] = '\0';
                ESP_LOGI(TAG, "Received: %s", data);
                // ecoar de volta
                esp_spp_write(param->data_ind.handle, len, (uint8_t *)data);
                free(data);
            }
        }
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT: cong = %d", param->cong.cong);
        break;

    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
        break;

    default:
        ESP_LOGI(TAG, "SPP event: %d", event);
        break;
    }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;

case ESP_BT_GAP_PIN_REQ_EVT:
    ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT: min_16_digit=%d", param->pin_req.min_16_digit);
    if (param->pin_req.min_16_digit) {
        // requer PIN de 16 dígitos
        esp_bt_pin_code_t pin_code = {0};
        // aqui poderia preencher com um PIN de 16 dígitos se necessário
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
    } else {
        // PIN de 4 dígitos "1234"
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '1';
        pin_code[1] = '2';
        pin_code[2] = '3';
        pin_code[3] = '4';
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
    break;

    default:
        ESP_LOGI(TAG, "GAP event: %d", event);
        break;
    }
}


// task que atualiza o threshold 
void MemoryManager(void *pvParameters)
{
	
	while(1){
		if(xQueueReceive(MemoryQueue, &voltage_threshold, portMAX_DELAY))
		{
			gpio_set_level(LED_GPIO, ON);

			eeprom_write_uint16(0x00, voltage_threshold);
			ESP_LOGI(TAG2,"Valor de threshold: %d", voltage_threshold);
		}
		
		gpio_set_level(LED_GPIO, OFF);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// task de alarme
void AlarmVoltage(void *pvParameters)
{
	uint16_t voltage;
	
	while(1)
	{
		if(xQueueReceive(AlarmQueue, &voltage, portMAX_DELAY))
		{
			if(voltage > voltage_threshold)
				gpio_set_level(ALARM_GPIO, ON);
			else
				gpio_set_level(ALARM_GPIO, OFF);		
		}
	}
}

//task de leitura do ADC
void ReadADCMeasure(void *pvParameters)
{
	adc_oneshot_unit_handle_t ADC_Handle;
	adc_cali_handle_t ADC_Cal_Handle;
	
	adc_oneshot_unit_init_cfg_t adc2_config ={
		.unit_id = ADC_UNIT_2,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	
	// configura o ADC para leitura como oneshot
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &ADC_Handle));
	
	adc_oneshot_chan_cfg_t chan_cfg ={
		.bitwidth = ADC_RESOLUTION,
		.atten = ADC_ATTEN_12,
		
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(ADC_Handle, ADC_CHANNEL, &chan_cfg));
	
	adc_cali_line_fitting_config_t adc_cal_config ={
		.unit_id = ADC_UNIT_2,
		.atten = ADC_ATTEN_12,
		.bitwidth = ADC_RESOLUTION,
		.default_vref = ADC_REF,
	};
	
	//calibra o ADC 
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&adc_cal_config,&ADC_Cal_Handle));
	
	int raw_value;
	int voltage;
		
	while(true) {

		adc_oneshot_read(ADC_Handle, ADC_CHANNEL, &raw_value);
		adc_cali_raw_to_voltage(ADC_Cal_Handle, raw_value, &voltage);
		
		ESP_LOGI(TAG1, "ADC -> %d mV", voltage);
		
		if (g_spp_handle != 0)
		{
   			char buf[32];
   			int n = snprintf(buf, sizeof(buf), "ADC:%d  mV\n", voltage);
    		esp_spp_write(g_spp_handle, n, (uint8_t*)buf);
		}

		if( gpio_get_level(BUTTON_GPIO) == 0)
		{
			vTaskDelay(pdMS_TO_TICKS(50));
			
			if(gpio_get_level(BUTTON_GPIO) == 0) 
				xQueueSend(MemoryQueue, &voltage, 0);
		}

		xQueueSend(AlarmQueue, &voltage, 0);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	
}

void app_main(void)
{
	
	esp_rom_gpio_pad_select_gpio(BUTTON_GPIO);
	gpio_set_direction( BUTTON_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode( BUTTON_GPIO, GPIO_PULLUP_ONLY);	
	
	esp_rom_gpio_pad_select_gpio(LED_GPIO);
	gpio_set_direction(LED_GPIO,GPIO_MODE_OUTPUT);
	
	esp_rom_gpio_pad_select_gpio(ALARM_GPIO);
	gpio_set_direction(ALARM_GPIO,GPIO_MODE_OUTPUT);
	
	MemoryQueue = xQueueCreate(10, sizeof(uint16_t));
	AlarmQueue  = xQueueCreate(10, sizeof(uint16_t));
	
	i2c_master_init();
	
	eeprom_read_uint16(0x00, &voltage_threshold);
		
	ESP_LOGI(TAG3, "Leitura do threshold: %d", voltage_threshold);
	
	xTaskCreate(ReadADCMeasure, "ReadADC", 4096, NULL, 1, &ADCHandle);
	
	xTaskCreate(MemoryManager, "MemoryManager", 4096, NULL, 1, &MemoryHandle);
	
	xTaskCreate(AlarmVoltage, "Alarm Task", 4096,  NULL, 1, &AlarmHandle);
	
	esp_err_t ret;

		ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK(ret);

		// Inicializar o controlador BT
	    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	    ret = esp_bt_controller_init(&bt_cfg);
	    if (ret) {
	        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
	        return;
	    }

	    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
	    if (ret) {
	        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
	        return;
	    }

	    ret = esp_bluedroid_init();
	    if (ret) {
	        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
	        return;
	    }
	    ret = esp_bluedroid_enable();
	    if (ret) {
	        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
	        return;
	    }

	    // Registra callbacks
	    esp_bt_gap_register_callback(bt_app_gap_cb);
	    esp_spp_register_callback(spp_event_handler);
	    //esp_spp_init(ESP_SPP_MODE_CB);

	    esp_spp_cfg_t config = {
			.mode = ESP_SPP_MODE_CB,
		};

	    esp_spp_enhanced_init(&config);


	    // Após a inicialização do SPP é chamado ESP_SPP_INIT_EVT onde fazemos “esp_spp_start_srv”
	    // Mas podemos explicitamente iniciar: (isso depende da versão do IDF)
	    esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);

}
	
