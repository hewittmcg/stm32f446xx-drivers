/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Oct. 9, 2020
 *      Author: hmcga
 */

#include "stm32f446xx_gpio.h"

void gpio_pclk_control(GPIO_reg_def_t *p_GPIOx, uint8_t en_or_di) {
	if(en_or_di == ENABLE) {
		if(p_GPIOx ==  GPIOA)
			GPIOA_PCLK_EN();

		else if(p_GPIOx ==  GPIOB)
			GPIOB_PCLK_EN();

		else if(p_GPIOx ==  GPIOC)
			GPIOC_PCLK_EN();

		else if(p_GPIOx ==  GPIOD)
			GPIOD_PCLK_EN();

		else if(p_GPIOx ==  GPIOE)
			GPIOE_PCLK_EN();

		else if(p_GPIOx ==  GPIOF)
			GPIOF_PCLK_EN();

		else if(p_GPIOx ==  GPIOG)
			GPIOG_PCLK_EN();

		else if(p_GPIOx ==  GPIOH)
			GPIOH_PCLK_EN();

	} else {
		if(p_GPIOx ==  GPIOA)
			GPIOA_PCLK_DI();

		else if(p_GPIOx ==  GPIOB)
			GPIOB_PCLK_DI();

		else if(p_GPIOx ==  GPIOC)
			GPIOC_PCLK_DI();

		else if(p_GPIOx ==  GPIOD)
			GPIOD_PCLK_DI();

		else if(p_GPIOx ==  GPIOE)
			GPIOE_PCLK_DI();

		else if(p_GPIOx ==  GPIOF)
			GPIOF_PCLK_DI();

		else if(p_GPIOx ==  GPIOG)
			GPIOG_PCLK_DI();

		else if(p_GPIOx ==  GPIOH)
			GPIOH_PCLK_DI();
	}
}

void gpio_init(GPIO_handle_t *p_GPIO_handle) {
	// Configure mode of GPIO pin
	uint32_t temp = 0; // temp register

	// Enable peripheral clock
	gpio_pclk_control(p_GPIO_handle->p_GPIO_x, ENABLE);
	
	if(p_GPIO_handle->pin_config.pin_mode <= GPIO_MODE_ANALOG) {
		// Non-interrupt mode
		// Multiply by width of bit field (2)
		temp = p_GPIO_handle->pin_config.pin_mode << (2 * p_GPIO_handle->pin_config.pin_number);
		p_GPIO_handle->p_GPIO_x->MODER &= ~(0x3 << p_GPIO_handle->pin_config.pin_number); // Clear bits
		p_GPIO_handle->p_GPIO_x->MODER |= temp; // Set bits
		temp = 0;
	} else {
		// Interrupt mode
		if(p_GPIO_handle->pin_config.pin_mode == GPIO_MODE_IT_FT) {
			// Configure FTSR
			EXTI->FTSR |= (1 << p_GPIO_handle->pin_config.pin_number);

			// Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << p_GPIO_handle->pin_config.pin_number);
		} else if(p_GPIO_handle->pin_config.pin_mode == GPIO_MODE_IT_RT) {
			// Configure RTSR
			EXTI->RTSR |= (1 << p_GPIO_handle->pin_config.pin_number);

			// Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1 << p_GPIO_handle->pin_config.pin_number);

		} else if(p_GPIO_handle->pin_config.pin_mode == GPIO_MODE_IT_RFT) {
			// Configure FTSR and RTSR
			EXTI->FTSR |= (1 << p_GPIO_handle->pin_config.pin_number);
			EXTI->RTSR |= (1 << p_GPIO_handle->pin_config.pin_number);
		}

		// Configure GPIO port selection in SYSCFG_EXTI_CR
		uint8_t exticr_num = p_GPIO_handle->pin_config.pin_number / 4; // EXTICR register to use for pin
		uint8_t exti_block = p_GPIO_handle->pin_config.pin_number % 4; // block to use in EXTI register
		uint8_t port_code = GPIO_BASE_ADDR_TO_CODE(p_GPIO_handle->p_GPIO_x);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[exticr_num] = port_code << (exti_block * 4); // blocks are 4 bits wide

		// Enable EXTI interrupt delivery using IMR 
		EXTI->IMR |= (1 << p_GPIO_handle->pin_config.pin_number); 
	}
	
	// Configure speed of GPIO pin
	temp = p_GPIO_handle->pin_config.pin_speed << (2 * p_GPIO_handle->pin_config.pin_number);
	p_GPIO_handle->p_GPIO_x->OSPEEDR &= ~(0x3 << p_GPIO_handle->pin_config.pin_number); // Clear bits
	p_GPIO_handle->p_GPIO_x->OSPEEDR |= temp; // Set bits
	temp = 0;

	// Configure PU/PD settings
	temp = p_GPIO_handle->pin_config.pin_pu_pd_control << (2 * p_GPIO_handle->pin_config.pin_number);
	p_GPIO_handle->p_GPIO_x->PUPDR &= ~(0x3 << p_GPIO_handle->pin_config.pin_number); // Clear bits
	p_GPIO_handle->p_GPIO_x->PUPDR |= temp; // Set bits
	temp = 0;

	// Configure output type (only if pin is an output pin)
	if(p_GPIO_handle->pin_config.pin_mode == GPIO_MODE_OUT) {
		temp = p_GPIO_handle->pin_config.pin_op_type << (p_GPIO_handle->pin_config.pin_number);
		p_GPIO_handle->p_GPIO_x->OTYPER &= ~(0x1 << p_GPIO_handle->pin_config.pin_number); // Clear bits
		p_GPIO_handle->p_GPIO_x->OTYPER |= temp; // Set bits
		temp = 0;
	}

	// Configure alternate functionality
	if(p_GPIO_handle->pin_config.pin_mode == GPIO_MODE_ALTFN) {
		uint32_t temp1 = 0, temp2 = 0;
		temp1 = p_GPIO_handle->pin_config.pin_number / 8;
		temp2 = p_GPIO_handle->pin_config.pin_number % 8;
		p_GPIO_handle->p_GPIO_x->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clear bits
		p_GPIO_handle->p_GPIO_x->AFR[temp1] |= (p_GPIO_handle->pin_config.pin_altfn_mode << (4 * temp2)); // Set bits
	}
}

void gpio_deinit(GPIO_reg_def_t *p_GPIOx) {
	if(p_GPIOx ==  GPIOA)
		GPIOA_REG_RESET();

	else if(p_GPIOx ==  GPIOB)
		GPIOB_REG_RESET();

	else if(p_GPIOx ==  GPIOC)
		GPIOC_REG_RESET();

	else if(p_GPIOx ==  GPIOD)
		GPIOD_REG_RESET();

	else if(p_GPIOx ==  GPIOE)
		GPIOE_REG_RESET();

	else if(p_GPIOx ==  GPIOF)
		GPIOF_REG_RESET();

	else if(p_GPIOx ==  GPIOG)
		GPIOG_REG_RESET();

	else if(p_GPIOx ==  GPIOH)
		GPIOH_REG_RESET();
}

uint8_t gpio_read_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number) {
	uint8_t value;
	// Get value from input register corresponding to pin 
	value = (uint8_t)((p_GPIOx->IDR >> pin_number) & 0x1); 

	return value;
}

uint16_t gpio_read_port(GPIO_reg_def_t *p_GPIOx) {
	uint16_t value;

	value = (uint16_t)p_GPIOx->IDR;

	return value;
}

void gpio_write_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number, uint8_t value) {
	if(value == GPIO_PIN_SET) {
		// Write 1 to output data register at bitfield corresponding to pin number
		p_GPIOx->ODR |= (1 << pin_number);
	} else {
		// Write 0 to output data register at bitfield corresponding to pin number
		p_GPIOx->ODR &= ~(1 << pin_number);
	}
}

void gpio_write_port(GPIO_reg_def_t *p_GPIOx, uint16_t value) {
	p_GPIOx->ODR = value;
}

void gpio_toggle_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number) {
	p_GPIOx->ODR ^= (1 << pin_number);
}

void gpio_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di) {
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

void gpio_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority) {
	// Determine IPR register
	uint8_t ipr = IRQ_number / 4;
	uint8_t ipr_section = IRQ_number % 4;

	// Jump to ipr register address and flip bits corresponding to priority 
	// See p.g. 4-7 of generic user guide for more information
	uint8_t shift_amt = (8 * ipr_section) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + ipr) |= IRQ_priority << shift_amt;
}

void gpio_irq_handle(uint8_t pin_number) {
	// Clear the EXTI PR register corresponding to the pin number 
	if(EXTI->PR & (1 << pin_number)) {
		// Interrupt pending, so clear 
		EXTI->PR |= (1 << pin_number);	
	}
}
