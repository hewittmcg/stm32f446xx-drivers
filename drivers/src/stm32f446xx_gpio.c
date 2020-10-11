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
	
	if(p_GPIO_handle->pin_config.pin_mode <= GPIO_MODE_ANALOG) {
		// Non-interrupt mode
		// Multiply by width of bit field (2)
		temp = p_GPIO_handle->pin_config.pin_mode << (2 * p_GPIO_handle->pin_config.pin_number);
		p_GPIO_handle->p_GPIO_x->MODER &= ~(0x3 << p_GPIO_handle->pin_config.pin_number); // Clear bits
		p_GPIO_handle->p_GPIO_x->MODER |= temp; // Set bits
		temp = 0;
	} else {
		// Interrupt mode
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

	// Configure output type
	temp = p_GPIO_handle->pin_config.pin_op_type << (p_GPIO_handle->pin_config.pin_number);
	p_GPIO_handle->p_GPIO_x->OTYPER &= ~(0x1 << p_GPIO_handle->pin_config.pin_number); // Clear bits
	p_GPIO_handle->p_GPIO_x->OTYPER |= temp; // Set bits
	temp = 0;

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

}

uint16_t gpio_read_port(GPIO_reg_def_t *p_GPIOx) {

}

void gpio_write_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number, uint8_t value) {

}

void gpio_write_port(GPIO_reg_def_t *p_GPIOx, uint16_t value) {

}

void gpio_toggle_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number) {

}

void gpio_irq_config(uint8_t IRQ_number, uint8_t IRQ_priority, uint8_t en_or_di) {

}

void gpio_irq_handle(uint8_t pin_number) {

}
