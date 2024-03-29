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
// TODO: change types to their respective enums
typedef struct {
    uint8_t device_mode; // Master/slave config
    uint8_t bus_config; // Full duplex/half duplex/simplex
    uint8_t sclk_speed; // SCLK speed
    uint8_t dff; // Shift register frame format (8/16 bit)
    uint8_t cpol; // Clock polarity bit
    uint8_t cpha; // Clock phase bit
    uint8_t ssm; // SW/HW slave management config
} SpiConfig;

// Device modes
typedef enum {
    SPI_DEVICE_MODE_SLAVE = 0, 
    SPI_DEVICE_MODE_MASTER, 
    NUM_SPI_DEVICE_MODES,
} SpiDeviceMode;

// Bus config modes 
// Note: use full duplex for simplex TX only configurations
typedef enum {
    SPI_BUS_CONFIG_FD = 0, // Full duplex
    SPI_BUS_CONFIG_HD, // Half duplex
    SPI_BUS_CONFIG_S_RXONLY, // Simplex, RX only
    NUM_SPI_BUS_CONFIGS,
} SpiBusConfig;

// Sclk speeds (see p.g. 887 of reference manual)
// Each speed represents a ratio of the system PCLK:
// SPI_SCLK_SPEED_DIV_2 = f(PCLK) / 2 and so on
typedef enum {
    SPI_SCLK_SPEED_DIV_2 = 0,
    SPI_SCLK_SPEED_DIV_4,
    SPI_SCLK_SPEED_DIV_8,
    SPI_SCLK_SPEED_DIV_16,
    SPI_SCLK_SPEED_DIV_32,
    SPI_SCLK_SPEED_DIV_64,
    SPI_SCLK_SPEED_DIV_128,
    SPI_SCLK_SPEED_DIV_256,
    NUM_SPI_SCLK_SPEEDS,
} SpiSclkSpeed;

// DFF formats
typedef enum {
    SPI_DFF_FORMAT_8_BIT = 0, 
    SPI_DFF_FORMAT_16_BIT, 
    NUM_SPI_DFF_FORMATS,
} SpiDffFormat;

// CPOL 
typedef enum {
    SPI_CPOL_LOW = 0, 
    SPI_CPOL_HIGH, 
    NUM_SPI_COPLS,
} SpiCpol;

// CPHA
typedef enum {
    SPI_CPHA_LOW = 0, 
    SPI_CPHA_HIGH, 
    NUM_SPI_CPHAS,
} SpiCpha;

// SSM SW config
typedef enum {
    SPI_SSM_CONFIG_DI = 0, 
    SPI_SSM_CONFIG_EN, 
    NUM_SPI_SSM_CONFIGS,
} SpiSsmConfig;

// Handle structure for SPI peripherals 
typedef struct {
    SpiRegDef *p_spi_reg; // Holds base address of SPI peripheral
    SpiConfig spi_config;
    // For use with interrupt-configured SPI:
    uint8_t *p_tx_buffer; // Application TX buffer address
    uint8_t *p_rx_buffer; // Application RX buffer address
    uint32_t tx_len;
    uint32_t rx_len;
    uint8_t tx_state;
    uint8_t rx_state;
} SpiHandle;


// Status flags, definitions
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1 << SPI_SR_BSY)

// SPI application states
typedef enum {
    SPI_STATE_READY = 0,
    SPI_STATE_BUSY_RX,
    SPI_STATE_BUSY_TX,
    NUM_SPI_STATES,
} SpiState;

// SPI application events
typedef enum {
    SPI_EVENT_TX_DONE = 0,
    SPI_EVENT_RX_DONE,
    SPI_EVENT_OVR_ERR,
    SPI_EVENT_CRC_ERR,
    NUM_SPI_EVENTS,
} SpiEvent;

// Driver API

// Set up the SPI perhiperal clock
// Takes in pointer to the base address of the SPI register
// and whether to enable or disable the clock for that register
void spi_pclk_control(SpiRegDef *p_spi_reg, uint8_t en_or_di);

// Initialize the given SPI port with the given settings.
void spi_init(SpiHandle *p_spi_handle);

// De-initialize -- reset all registers of the given SPI register.
void spi_deinit(SpiRegDef *p_spi_reg);

// Return the status of the requested flag.
uint8_t spi_get_flag_status(SpiRegDef *p_spi_reg, uint32_t flag_type);

// Clear the OVR flag.
void spi_clear_ovr_flag(SpiRegDef *p_spi_reg);

// End SPI transmission.
void spi_end_tx(SpiHandle *p_spi_handle);

// End SPI reception.
void spi_end_rx(SpiHandle *p_spi_handle);

// Send data over SPI.  p_tx_buffer points to an array of bytes to be sent.
// Note: this is a blocking call.
void spi_send(SpiRegDef *p_spi_reg, uint8_t *p_tx_buffer, uint32_t len);

// RX 
void spi_receive(SpiRegDef *p_spi_reg, uint8_t *p_rx_buffer, uint32_t len);

// Same functions, but interrupt-based and non-blocking:
// Return STATUS_SUCCESS if successful, STATUS_FAILED if SPI busy
uint8_t spi_send_it(SpiHandle *p_spi_handle, uint8_t *p_tx_buffer, uint32_t len);

uint8_t spi_receive_it(SpiHandle *p_spi_handle, uint8_t *p_rx_buffer, uint32_t len);


// Interrupt configuration (processor side -- see Cortex-M4 Generic User Guide)
void spi_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

// Interrupt priority configuration (processor side -- see Cortex-M4 Generic User Guide)
void spi_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority);

// Process interrupt 
void spi_irq_handle(SpiHandle *p_handle);

// Other periperal control APIs

// Enable/disable SPI peripheral
void spi_peripheral_control(SpiRegDef *p_spi_reg, uint8_t en_or_di);

// Enable/disable SSI
void spi_ssi_config(SpiRegDef *p_spi_reg, uint8_t en_or_di);

// Enable/disable SSOE
void spi_ssoe_config(SpiRegDef *p_spi_reg, uint8_t en_or_di);


// Application callback:

// Must be implemented by the application
void spi_app_event_cb(SpiHandle *p_spi_handle, uint8_t event);

#endif /* INC_STM32F446XX_SPI_H_ */
