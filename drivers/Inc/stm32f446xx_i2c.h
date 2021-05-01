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
    uint8_t speed;
    uint8_t addr;
    uint8_t ack_en;
    uint8_t fm_duty_cycle; // fast mode duty cycle
} I2cConfig;

// Handle structure for I2C peripherals
typedef struct {
    I2cRegDef *p_i2c_reg;
    I2cConfig i2c_config;
} I2cHandle;

#endif /* INC_STM32F446XX_I2C_H_ */
