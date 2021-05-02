#ifndef INC_STM32F446XX_I2C_H_
#define INC_STM32F446XX_I2C_H_

#include "stm32f446xx.h"

// Interrupt request events
// See p. 778 of reference manual
typedef enum {
    I2C_INT_REQ_SB = 0, // Start bit sent (master)
    I2C_INT_REQ_ADDR, // Address sent (master), address matched (slave)
    I2C_INT_REQ_ADD10, // 10-bit header sent (master)
    I2C_INT_REQ_STOPF, // Stop received (slave) 
    I2C_INT_REQ_BTF, // Data byte transfer finished
    I2C_INT_REQ_RXNE, // Receive buffer not empty
    I2C_INT_REQ_TXE, // Transmit buffer empty
    I2C_INT_REQ_BERR, // Bus error
    I2C_INT_REQ_ARLO, // Arbitration loss (master)
    I2C_INT_REQ_AF, // Ack fail
    I2C_INT_REQ_OVR, // Overrun/underrun
    I2C_INT_REQ_PECERR, // PEC error
    I2C_INT_REQ_TIMEOUT, // Timeout/Tlow error
    I2C_INT_REQ_SMBALERT, // SMBus Alert
    NUM_I2C_INT_REQS,
} I2cIntReq;

// Config structure for I2C peripherals
typedef struct {
    uint32_t speed;
    uint8_t addr;
    uint8_t ack_en;
    uint8_t fm_duty_cycle; // fast mode duty cycle
} I2cConfig;

// Handle structure for I2C peripherals
typedef struct {
    I2cRegDef *p_i2c_reg;
    I2cConfig i2c_config;
} I2cHandle;

// Speeds
#define I2C_SCL_SPEED_SM 100000 // 100 kHz
#define I2C_SCL_SPEED_FM 400000 // 400 kHz

// TRISE defs
#define I2C_TRISE_MAX_SM 1000
#define I2C_TRISE_MAX_FM 300

// ACK EN/DI (see CCR register)
#define I2C_ACK_DI 0
#define I2C_ACK_EN 1

// Fast mode duty cycle (see CCR register)
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

// Driver API

// Set up the I2C perhiperal clock
// Takes in pointer to the base address of the I2C register
// and whether to enable or disable the clock for that register
void i2c_pclk_control(I2cRegDef *p_i2c_reg, uint8_t en_or_di);

// Initialize the given I2C port with the given settings.
// NOTE: must be called when I2C peripheral is disabled
// (e.g. prior to calling i2c_perhiperal_control() with ENABLE)
void i2c_init(I2cHandle *p_i2c_handle);

// De-initialize -- reset all registers of the given SPI register.
void i2c_deinit(I2cRegDef *p_i2c_reg);

// Send data from master
void i2c_master_send(I2cHandle *p_i2c_handle, uint8_t *tx_buf, uint32_t len, uint8_t slave_addr);

// Return the status of the requested flag.
uint8_t i2c_get_flag_status(I2cRegDef *p_i2c_reg, uint32_t flag_type);

// Clear the OVR flag.
void i2c_clear_ovr_flag(I2cRegDef *p_i2c_reg);

// Interrupt configuration (processor side -- see Cortex-M4 Generic User Guide)
void i2c_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

// Interrupt priority configuration (processor side -- see Cortex-M4 Generic User Guide)
void i2c_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority);

// Enable/disable I2C peripheral
void i2c_peripheral_control(I2cRegDef *p_i2c_reg, uint8_t en_or_di);

// Application callback:

// Must be implemented by the application
void i2c_app_event_cb(I2cHandle *p_i2c_handle, uint8_t event);


#endif /* INC_STM32F446XX_I2C_H_ */
