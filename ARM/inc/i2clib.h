/*
 * i2clib.h
 *
 *  Created on: May 8, 2017
 *      Initial author: Tom
 *
 *
 *
 *
 *
 *
 *  I2CLIB USAGE:
 *
 *  INITIALIZATION
 * Mandatory:
 * 1) I2C_init() -- initialize ic2 for the specified I2C bus (I2C1 or I2C2), address, and timeout
 * 2) I2C_slave_init -- initialize the specified slave device for I2C communication
 * 
 * Optional:
 * 3) I2C_slave_mem_init -- initialize memory address size for the slave device. Required before calling I2C_mem_xxx functions
 *
 *  TRANSMITTING DATA:
 *  
 * There are 2 functions for transmitting data:
 * 1) I2C_send_data -- this sends n_bytes from the specified data buffer to the specified device
 * 2) I2C_mem_write -- this writes the specified data to the memory address within the device. Must initialize mem on the device
 *                      with I2C_slave_mem_init
 *
 *  RECEIVING MESSAGES:
 * 1) I2C_send_data -- this reads n_bytes to the specified data buffer from the specified device
 * 2) I2C_mem_write -- this reads to p_data from the specified memory address within the device. Must initialize mem on the device
 *                      with I2C_slave_mem_init
 *
 *
 *  ADDITIONAL NOTES:
 *
 *
 */

#ifndef I2CLIB_H_
#define I2CLIB_H_

#include "stdint.h"
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"


typedef struct I2C_Device {
    I2C_TypeDef *I2Cx;
    uint16_t address;
    uint32_t timeout;
    uint16_t mem_add_size; //Size of the memory addresses for I2C_mem_xxx.
} I2C_Device_t;

int I2C_init(I2C_TypeDef *I2Cx);
int I2C_slave_init(I2C_Device_t *device, I2C_TypeDef *I2Cx, uint16_t address, uint32_t timeout);

//Required for I2C_mem_xxx functions
//mem_address_size = 8 or 16, depending on slave
int I2C_slave_mem_init(I2C_Device_t *device, uint16_t mem_address_size);

// Write n_bytes of data to device
int I2C_send_data(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes);

//Read n_bytes of data from device
int I2C_receive_data(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes);

// Write to MemAddress in device
int I2C_mem_write(I2C_Device_t *device, uint16_t MemAddress, uint8_t *p_data, uint16_t n_bytes);

// Read from MemAddress in device
int I2C_mem_read(I2C_Device_t *device, uint16_t MemAddress, uint8_t *p_data, uint16_t n_bytes);

int I2C_receive_slave(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes);

int I2C_send_slave(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes);

#endif
