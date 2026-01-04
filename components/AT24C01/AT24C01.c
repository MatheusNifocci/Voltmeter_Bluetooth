/*
 * AT24C01.c
 *
 *  Created on: 8 de set. de 2025
 *      Author: Matheus Nifocci
 */

#include "AT24C01.h"
#include "hal/gpio_types.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ          	100000

#define AT24C01_ADDR                (0x50<<1)
#define AT24C01_SIZE                128

void i2c_master_init(void)
{
	
		i2c_config_t config_I2C = // struct de configuração da I2C 
		{
			.mode = I2C_MODE_MASTER,
			.sda_io_num = I2C_MASTER_SDA_IO,
			.scl_io_num = I2C_MASTER_SCL_IO,
			.scl_pullup_en = GPIO_PULLUP_DISABLE,
			.sda_pullup_en = GPIO_PULLUP_DISABLE,
			.master.clk_speed = I2C_MASTER_FREQ,		
		};
		
		// configuração da I2C
		i2c_param_config(I2C_MASTER_NUM, &config_I2C);
		
		// instalação do driver 
		i2c_driver_install(I2C_MASTER_NUM, config_I2C.mode, 0, 0, 0);
	
}

esp_err_t eeprom_write_byte(uint8_t mem_addr, uint8_t data){
	
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AT24C01_ADDR  | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mem_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(10)); // tempo de gravação interna
    
    ESP_LOGI("EEPROM", "Retorno da escrita: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t eeprom_read_byte(uint8_t mem_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    
    // Escrita
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AT24C01_ADDR  | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mem_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    // Leitura
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AT24C01_ADDR | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}


esp_err_t eeprom_write_bytes(uint8_t start_addr, const uint8_t *data, size_t len) {
    esp_err_t ret = ESP_OK;

    for (size_t i = 0; i < len; i++) {
        ret = eeprom_write_byte(start_addr + i, data[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ret;
}


esp_err_t eeprom_read_bytes(uint8_t start_addr, uint8_t *data, size_t len) {
    esp_err_t ret = ESP_OK;

    for (size_t i = 0; i < len; i++) {
        ret = eeprom_read_byte(start_addr + i, &data[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ret;
}

esp_err_t eeprom_write_uint16(uint8_t addr, uint16_t value) {
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // byte alto
    data[1] = value & 0xFF;         // byte baixo
    return eeprom_write_bytes(addr, data, 2);
}

esp_err_t eeprom_read_uint16(uint8_t addr, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret = eeprom_read_bytes(addr, data, 2);
    if (ret != ESP_OK) return ret;

    *value = ((uint16_t)data[0] << 8) | data[1];  // reconstroi o dado
    return ESP_OK;
}
