/*
 * AT24C01.h
 *
 *  Created on: 8 de set. de 2025
 *      Author: Matheus Nifocci
 */

#ifndef AT24C01_H_
#define AT24C01_H_

#include "driver/i2c.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"




void i2c_master_init(void); // Inicializa a I2C como Master

//Funções para escrever e ler um byte
esp_err_t eeprom_write_byte(uint8_t mem_addr, uint8_t data);
esp_err_t eeprom_read_byte(uint8_t mem_addr, uint8_t *data);

//Funções para escrever e ler mais de um byte
esp_err_t eeprom_write_bytes(uint8_t start_addr, const uint8_t *data, size_t len);
esp_err_t eeprom_read_bytes(uint8_t start_addr, uint8_t *data, size_t len);

//Funções para escrever e ler variáveis do tipo uint16
esp_err_t eeprom_write_uint16(uint8_t addr, uint16_t value);
esp_err_t eeprom_read_uint16(uint8_t addr, uint16_t *value);

#endif /* AT24C01_H_ */
