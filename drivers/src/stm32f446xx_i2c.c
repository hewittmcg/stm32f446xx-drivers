/*
 * stm32f446xx_i2c.c
 *
 *  Created on: May 13, 2021
 *      Author: Hewitt
 */

#include "stm32f446xx_i2c.h"

#define HZ_TO_MHZ 1000000U


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

// Calculate value from APB1
static uint32_t prv_rcc_get_pclk1(void) {
    uint32_t pclk1 = 0, sys_clk = 0;

    uint8_t clk_src = 0;

    // Read SWS (system clock switch status) -- bits 2-3
    clk_src = (RCC->CFGR >> 2) & 0b11;
    if(clk_src == 0) {
        // HSI
        sys_clk = CLK_SPEED_HSI;
    } else if(clk_src == 1) {
        // HSE
        // NOTE: this should not be reached for this driver implementation!
        sys_clk = CLK_SPEED_HSE;
    } else if(clk_src == 2) {
        // PLL
        // NOTE: also unused currently
        sys_clk = 0;
    }

    // Read AHB prescaler (HPRE field in RCC_CFGR)
    uint8_t hpre = (RCC->CFGR >> 4) & 0b1111;
    uint8_t ahb_prescale = 1;
    if(hpre < 8) {
        ahb_prescale = 1;
    } else {
        // Prescaler is equal to 2^((3 LSB) + 1)
        // (see p.g. 133 of reference manual)
         uint8_t pow = (hpre & 0b111) + 1;
         ahb_prescale = (1 << pow);
    }

    // Read APB1 prescaler (PPRE1 field in RCC_CFGR)
    uint8_t ppre1 = (RCC->CFGR >> 10) &0b111;
    uint8_t apb1_prescale = 1;
    if(ppre1 < 4) {
        apb1_prescale = 1;
    } else {
        // Similar to above, except only on 2 LSB
        uint8_t pow = (ppre1 &0b11) + 1;
        apb1_prescale = (1 << pow);
    }

    // Apply prescalers to system clock
    pclk1 = (sys_clk / ahb_prescale) / apb1_prescale;
    return pclk1;
}

void i2c_init(I2cHandle *p_i2c_handle) {
    // Configure SCL speed (6 LSB of CR2)
    uint32_t temp_cr2 = prv_rcc_get_pclk1() / HZ_TO_MHZ;
    p_i2c_handle->p_i2c_reg->CR2 = (temp_cr2 &0b111111) << I2C_CR2_FREQ;

    // Configure device address (if slave)
    uint32_t temp_oar1 = p_i2c_handle->i2c_config.addr << 1;
    temp_oar1 |= 1 << 14; // bit 14 must be set in sw
    p_i2c_handle->p_i2c_reg->OAR1 = temp_oar1;

    // Enable ACK
    uint32_t temp_cr1 = 0;
    temp_cr1 |= p_i2c_handle->i2c_config.ack_en << I2C_CR1_ACK;
    p_i2c_handle->p_i2c_reg->CR1 = temp_cr1;

    // CCR calculations
    // NOTE: this needs to be refactored
    uint16_t ccr_val = 0;
    uint32_t temp_ccr = 0;
    if(p_i2c_handle->i2c_config.speed < I2C_SCL_SPEED_SM) {
        // Standard mode
        // Assume Tlow = Thigh
        ccr_val = prv_rcc_get_pclk1() / (2 *p_i2c_handle->i2c_config.speed);
    } else {
        // Fast mode
        temp_ccr |= (1 << I2C_CCR_FS);
        uint8_t duty = p_i2c_handle->i2c_config.fm_duty_cycle;
        if(duty == I2C_FM_DUTY_2) {
            // 2 duty cycle
            ccr_val = prv_rcc_get_pclk1() / (3 *p_i2c_handle->i2c_config.speed);
        } else {
            // 16:9 duty cycle
            ccr_val = prv_rcc_get_pclk1() / (25 *p_i2c_handle->i2c_config.speed);
        }
    }
    temp_ccr |= (ccr_val &0xFFF); // 12 LSB only
    p_i2c_handle->p_i2c_reg->CCR = temp_ccr;

    // Configure rise time for I2C pins (TODO)
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

// Generate the START condition
static void prv_i2c_gen_start(I2cRegDef *p_i2c_reg) {
    p_i2c_reg->CR1 |= (1 << I2C_CR1_START);
}

// Send address of slave with r/nw bit set to 0 (WRITE)
static void prv_i2c_send_addr(I2cRegDef *p_i2c_reg, uint8_t addr) {
    // Shift up to make space for r/w bit
    addr = addr << 1;

    // Clear r/w bit
    addr &= ~(1);
    p_i2c_reg->DR = addr;
}

// Clear the ADDR flag by reading SR1 and SR2
static void prv_i2c_clear_addr_flag(I2cRegDef *p_i2c_reg) {
    uint32_t dummy_read = p_i2c_reg->SR1;
    dummy_read = p_i2c_reg->SR2;
    (void)dummy_read; // avoid unused error
}

// Generate STOP condition
static void prv_i2c_gen_stop(I2cRegDef *p_i2c_reg) {
    p_i2c_reg->CR1 |= (1 << I2C_CR1_START);
}

void i2c_master_send(I2cHandle *p_i2c_handle, uint8_t *tx_buf, uint32_t len, uint8_t slave_addr) {
    // Generate START condition
    prv_i2c_gen_start(p_i2c_handle->p_i2c_reg);

    // Confirm start generation complete by checking whether SB flag cleared
    uint8_t flag = (1 << I2C_SR1_SB);
    while(i2c_get_flag_status(p_i2c_handle->p_i2c_reg, flag)); 

    prv_i2c_send_addr(p_i2c_handle->p_i2c_reg, p_i2c_handle->i2c_config.addr);

    prv_i2c_clear_addr_flag(p_i2c_handle->p_i2c_reg);

    // Send data until len == 0
    while(len > 0) {
        // Wait for TXE to be set
        while(!i2c_get_flag_status(p_i2c_handle->p_i2c_reg, (1 << I2C_SR1_TXE)));
        
        p_i2c_handle->p_i2c_reg->DR = *tx_buf;
        tx_buf++;
        len--;
    }

    // Transmission finished -- close communication
    // Wait for TXE=1 and BTF=1 to represent that SR, DR both empty
    while(!i2c_get_flag_status(p_i2c_handle->p_i2c_reg, (1 << I2C_SR1_TXE)));
    while(!i2c_get_flag_status(p_i2c_handle->p_i2c_reg, (1 << I2C_SR1_BTF)));

    // Generate stop condition
    prv_i2c_gen_stop(p_i2c_handle->p_i2c_reg);
}

uint8_t i2c_get_flag_status(I2cRegDef *p_i2c_reg, uint32_t flag_type) {
    if(p_i2c_reg->SR1 & flag_type) {
        return FLAG_SET;
    } else {
        return FLAG_RESET;
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


