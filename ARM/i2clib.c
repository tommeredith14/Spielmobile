/*
 * i2clib.c
 *
 *  Created on: May 8, 2017
 *      Author: Tom
 */

#include "i2clib.h"
#include "pins.h"
#include <limits.h>
#include <string.h>

GPIO_InitTypeDef GPIO_InitStruct;



/*
    (#)Initialize the I2C low level resources by implementing the HAL_I2C_MspInit() API:
        (##) Enable the I2Cx interface clock
        (##) I2C pins configuration
            (+++) Enable the clock for the I2C GPIOs
            (+++) Configure I2C pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the I2Cx interrupt priority
            (+++) Enable the NVIC I2C IRQ Channel
        (##) DMA Configuration if you need to use DMA process
            (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive channel
            (+++) Enable the DMAx interface clock using
            (+++) Configure the DMA handle parameters
            (+++) Configure the DMA Tx or Rx channel
            (+++) Associate the initialized DMA handle to the hi2c DMA Tx or Rx handle
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on 
                  the DMA Tx or Rx channel
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {

//    (#)Initialize the I2C low level resources by implementing the HAL_I2C_MspInit() API:
//        (##) Enable the I2Cx interface clock
    //__HAL_I2C_ENABLE(hi2c); //necessary?
    __I2C1_CLK_ENABLE();

//        (##) I2C pins configuration
//            (+++) Enable the clock for the I2C GPIOs
    __GPIOB_CLK_ENABLE();

//            (+++) Configure I2C pins as alternate function open-drain

    GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = I2C_SCL_GPIO_PIN | I2C_SDA_GPIO_PIN,
            .Mode = GPIO_MODE_AF_OD,
            .Pull = GPIO_PULLUP, //?
            .Speed = GPIO_SPEED_FREQ_HIGH, //?
            .Alternate = GPIO_AF4_I2C1
    };
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

//        (##) NVIC configuration if you need to use interrupt process
//            (+++) Configure the I2Cx interrupt priority
//            (+++) Enable the NVIC I2C IRQ Channel
//        (##) DMA Configuration if you need to use DMA process
//            (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive channel
//            (+++) Enable the DMAx interface clock using
//            (+++) Configure the DMA handle parameters
//            (+++) Configure the DMA Tx or Rx channel
//            (+++) Associate the initialized DMA handle to the hi2c DMA Tx or Rx handle
//            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on 
//                  the DMA Tx or Rx channel

}
static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef hi2c2;

int I2C_init(I2C_TypeDef *I2Cx) {

    I2C_HandleTypeDef *hi2c;
    if (I2Cx == I2C1) {
        hi2c = &hi2c1;
        hi2c->Instance = I2C1;
    } else {
        return -1; // Not implemented
    }


    hi2c->Init.ClockSpeed = 400000;
		hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0x16;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0xFE;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(hi2c) != HAL_OK)
        return -1;

    return 0;    

}

int I2C_slave_init(I2C_Device_t *device, I2C_TypeDef *I2Cx, uint16_t address, uint32_t timeout) {
    if (!device)
        return -1;

    device->I2Cx = I2Cx;
    device -> address = address;
    device->timeout = timeout;

    return 0;
}

int I2C_slave_mem_init(I2C_Device_t *device, uint16_t mem_address_size) {
    if (!device)
        return -1;
    if (mem_address_size == 8) {
        device->mem_add_size = I2C_MEMADD_SIZE_8BIT;
    } else if (mem_address_size == 16) {
        device->mem_add_size = I2C_MEMADD_SIZE_16BIT;
    } else {
        return -1; //bad size. must be 8 or 16
    }
    return 0;
}

// Write n_bytes of data to device
int I2C_send_data(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }



    ret = HAL_I2C_Master_Transmit(hi2c,device->address,p_data,n_bytes,device->timeout);

    return ret;
}

//Read n_bytes of data from device
int I2C_receive_data(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }


    ret = HAL_I2C_Master_Receive(hi2c, device->address, p_data,n_bytes,device->timeout);
    return ret;
}

// Write to MemAddress in device
int I2C_mem_write(I2C_Device_t *device, uint16_t MemAddress, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }

    ret = HAL_I2C_Mem_Write(hi2c,device->address, MemAddress, device->mem_add_size, p_data, n_bytes, device->timeout);


    return ret;
}

// Read from MemAddress in device
int I2C_mem_read(I2C_Device_t *device, uint16_t MemAddress, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }

    ret = HAL_I2C_Mem_Read(hi2c,device->address, MemAddress, device->mem_add_size, p_data, n_bytes, device->timeout);


    return ret;
}

// Write n_bytes of data to device
int I2C_send_slave(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }

    ret = HAL_I2C_Slave_Transmit(hi2c, p_data,n_bytes,device->timeout);

    return ret;
}

//Read n_bytes of data from device
int I2C_receive_slave(I2C_Device_t *device, uint8_t *p_data, uint16_t n_bytes) {
    int ret;
    I2C_HandleTypeDef *hi2c;
    if (device->I2Cx == I2C1) {
        hi2c = &hi2c1;
    } else {
        return -1; // Not implemented
    }

    ret = HAL_I2C_Slave_Receive(hi2c, p_data,n_bytes,device->timeout);
    return ret;
}
