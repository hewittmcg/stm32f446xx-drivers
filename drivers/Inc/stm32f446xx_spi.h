/*
 * stm32f446xx_spi.h
 *
 *  Created on: Oct. 23, 2020
 *      Author: hmcga
 */

#ifndef INC_STM32F446XX_SPI_H_
#define INC_STM32F446XX_SPI_H_

#include "stm32f446xx.h"

// Config structure for SPI peripherals
typedef struct {
    uint8_t device_mode; // Master/slave config
    uint8_t bus_config; // Full duplex/half duplex/simplex
    uint8_t sclk_speed; // SCLK speed
    uint8_t dff; // Shift register frame format (8/16 bit)
    uint8_t cpol; // Clock polarity bit
    uint8_t cpha; // Clock phase bit
    uint8_t ssm; // SW/HW slave management config
} SpiConfig;

// Handle structure for SPI peripherals 
typedef struct {
    SpiRegDef *p_spi; // Holds base address of SPI peripheral
    SpiConfig spi_config;
} SpiHandle;

#endif /* INC_STM32F446XX_SPI_H_ */
