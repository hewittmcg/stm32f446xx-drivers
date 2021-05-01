/*
 * stm32f446xx_i2c.c
 *
 *  Created on: May 13, 2021
 *      Author: Hewitt
 */

#include "stm32f446xx_i2c.h"


void i2c_pclk_control(I2cRegDef *p_i2c_reg, uint8_t en_or_di) {
    if(en_or_di == ENABLE) {
        if(p_i2c_reg == I2C1) {
            I2C1_PCLK_EN();       
        } else if(p_i2c_reg == I2C2) {
            I2C2_PCLK_EN();
        } else if(p_i2c_reg == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        if(p_i2c_reg == I2C1) {
            I2C1_PCLK_DI();       
        } else if(p_i2c_reg == I2C2) {
            I2C2_PCLK_DI();
        } else if(p_i2c_reg == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}

void i2c_init(I2cHandle *p_i2c_handle) {
    // Configure mode (standard/fast)
    // Configure SCL speed
    // Configure device address (if slave)
    // Enable ACK
    // Configure rise time for I2C pins
}

void i2c_deinit(I2cRegDef *p_i2c_reg) {
    if(p_i2c_reg == I2C1) {
        I2C1_REG_RESET();       
    } else if(p_i2c_reg == I2C2) {
        I2C2_REG_RESET();
    } else if(p_i2c_reg == I2C3) {
        I2C3_REG_RESET();
    }
}

void i2c_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di) {
    if(en_or_di == ENABLE) {
		if(IRQ_number <= NVIC_ISER0_MAX_PRIORITY) {
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQ_number);
		} else if(IRQ_number < NVIC_ISER1_MAX_PRIORITY) {
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQ_number % 32));
		} else if(IRQ_number < NVIC_ISER2_MAX_PRIORITY) {
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQ_number % 64));
		}
	} else {
		if(IRQ_number <= NVIC_ICER0_MAX_PRIORITY) {
			// Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQ_number);
		} else if(IRQ_number < NVIC_ICER1_MAX_PRIORITY) {
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQ_number % 32));
		} else if(IRQ_number < NVIC_ICER2_MAX_PRIORITY) {
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQ_number % 64));
		}
	}
}

void i2c_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority) {
    // Determine IPR register
	uint8_t ipr = IRQ_number / 4;
	uint8_t ipr_section = IRQ_number % 4;

	// Jump to ipr register address and flip bits corresponding to priority 
	// See p.g. 4-7 of generic user guide for more information
	uint8_t shift_amt = (8 * ipr_section) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + ipr) |= IRQ_priority << shift_amt;
}

void i2c_peripheral_control(I2cRegDef *p_i2c_reg, uint8_t en_or_di) {
    uint16_t temp = 1 << I2C_CR1_PE;
    if(en_or_di == ENABLE) {
        // Set PE bit
        p_i2c_reg->CR1 |= temp;
    } else {
        // Clear PE bit
        p_i2c_reg->CR1 &= ~(temp);
    }
}


