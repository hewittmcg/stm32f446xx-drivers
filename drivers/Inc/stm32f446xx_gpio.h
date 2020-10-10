/*
 * stm32f446xx_gpio.h
 *
 *  Created on: Oct. 9, 2020
 *      Author: hmcga
 */

#ifndef INC_STM32F446XX_GPIO_H_
#define INC_STM32F446XX_GPIO_H_

#include "stm32f446xx.h"

// Config structure for GPIO pin
typedef struct {
	uint8_t pin_number;
	uint8_t pin_mode;
	uint8_t pin_speed;
	uint8_t pin_pu_pd_control;
	uint8_t pin_op_type;
	uint8_t pin_altfn_mode;
} GPIO_pin_config_t;

// Handle structure for GPIO pin
typedef struct {
	// Base address of GPIO peripheral
	GPIO_reg_def_t *p_GPIO_x; // Base address of GPIO port
	GPIO_pin_config_t *pin_config; // Pin config
}GPIO_handle_t;

// Driver API

// Set up the GPIO peripheral clock
void gpio_pclk_control(GPIO_reg_def_t *p_GPIOx, uint8_t en_or_di);

// Initialize
void gpio_init(GPIO_handle_t *p_GPIO_handle);

// De-initialize
void gpio_deinit(GPIO_reg_def_t *p_GPIOx);

// Read from an input pin
uint8_t gpio_read_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number);

// Read from an input port
uint16_t gpio_read_port(GPIO_reg_def_t *p_GPIOx);

// Write to an output pin
void gpio_write_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number, uint8_t value);

// Write to an output port
void gpio_write_port(GPIO_reg_def_t *p_GPIOx, uint16_t value);

// Toggle an output pin
void gpio_toggle_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number);

// Interrupt configuration
void gpio_irq_config(uint8_t IRQ_number, uint8_t IRQ_priority, uint8_t en_or_di);

// Process interrupt
void gpio_irq_handle(uint8_t pin_number);


#endif /* INC_STM32F446XX_GPIO_H_ */
