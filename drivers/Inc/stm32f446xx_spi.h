/*
 * stm32f446xx_spi.h
 *
 *  Created on: Oct. 23, 2020
 *      Author: hmcga
 */

// Basic SPI driver for STM32F446xx MCUs.

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
    SpiRegDef *p_spi_reg; // Holds base address of SPI peripheral
    SpiConfig spi_config;
} SpiHandle;

// Driver API

// Set up the SPI perhiperal clock
// Takes in pointer to the base address of the SPI register
// and whether to enable or disable the clock for that register
void spi_pclk_control(SpiRegDef *p_spi_reg, uint8_t en_or_di);

// Initialize the given SPI port with the given settings.
void spi_init(SpiHandle *p_spi_handle);

// De-initialize -- reset all registers of the given SPI register.
void spi_deinit(SpiRegDef *p_spi_reg);

// TX
void spi_send(SpiRegDef *p_spi_reg, uint8_t *p_tx_buffer, uint32_t len);

// RX 
void spi_receive(SpiRegDef *p_spi_reg, uint8_t *p_rx_buffer, uint32_t len);

// Interrupt configuration (processor side -- see Cortex-M4 Generic User Guide)
void spi_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

// Interrupt priority configuration (processor side -- see Cortex-M4 Generic User Guide)
void spi_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority);

// Process interrupt 
void spi_irq_handle(SpiHandle *p_handle);

#endif /* INC_STM32F446XX_SPI_H_ */
