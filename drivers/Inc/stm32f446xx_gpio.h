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
	uint8_t pin_number; // Values from GPIO_pin
	uint8_t pin_mode; // Values from GPIO_pin_mode
	uint8_t pin_speed; // Values from GPIO_speed
	uint8_t pin_pu_pd_control; // Values from GPIO_pin_PUPD
	uint8_t pin_op_type; // Values from GPIO_op_type
	uint8_t pin_altfn_mode;
} GPIO_pin_config_t;

// Handle structure for GPIO pin
typedef struct {
	// Base address of GPIO peripheral
	GPIO_reg_def_t *p_GPIO_x; // Base address of GPIO port
	GPIO_pin_config_t pin_config; // Pin config
} GPIO_handle_t;

// GPIO pin numbers
typedef enum {
	GPIO_PIN_0 = 0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
	NUM_GPIO_PINS
} GPIO_pin;

// Gpio pin modes
typedef enum {
	GPIO_MODE_IN = 0,
	GPIO_MODE_OUT,
	GPIO_MODE_ALTFN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT, // Falling edge interrupt
	GPIO_MODE_IT_RT, // Rising edge interrupt
	GPIO_MODE_IT_RFT, // Rising/falling edge interrupt
	NUM_GPIO_MODES
} GPIO_pin_mode;

// GPIO output types
typedef enum {
	GPIO_OP_TYPE_PP = 0,
	GPIO_OP_TYPE_OD,
	NUM_GPIO_OP_TYPES,
} GPIO_op_type;

// GPIO pin output speeds
typedef enum {
	GPIO_SPEED_LOW = 0,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_FAST,
	GPIO_SPEED_HIGH,
	NUM_GPIO_SPEEDS
} GPIO_speed;

// GPIO pin pull up/pull down states
typedef enum {
	GPIO_NO_PUPD = 0,
	GPIO_PIN_PU,
	GPIO_PIN_PD,
	NUM_GPIO_PIN_PUPDS
} GPIO_pin_PUPD;

// Driver API

// Set up the GPIO peripheral clock
// Takes in pointer to the base address of the GPIO register
//and whether to enable or disable the clock for that register
void gpio_pclk_control(GPIO_reg_def_t *p_GPIOx, uint8_t en_or_di);

// Initialize the given GPIO pin with the given settings.
void gpio_init(GPIO_handle_t *p_GPIO_handle);

// De-initialize -- reset all registers of the given GPIO register.
void gpio_deinit(GPIO_reg_def_t *p_GPIOx);

// Read from the given input pin, returning either 0 or 1.
uint8_t gpio_read_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number);

// Read from the entire input port in the register; return the value read.
uint16_t gpio_read_port(GPIO_reg_def_t *p_GPIOx);

// Write the given value (must be either 0 or 1) to the given output register at the 
// position given by pin_number.
void gpio_write_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number, uint8_t value);

// Set the given register to the given value.
void gpio_write_port(GPIO_reg_def_t *p_GPIOx, uint16_t value);

// Toggle the given output pin in the given output register.
void gpio_toggle_pin(GPIO_reg_def_t *p_GPIOx, uint8_t pin_number);

// Interrupt configuration (processor side -- see Cortex-M4 Generic User Guide)
void gpio_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

// Interrupt priority configuration (processor side -- see Cortex-M4 Generic User Guide)
void gpio_irq_priority_config(uint8_t IRQ_number, uint8_t IRQ_priority);

// Process interrupt 
void gpio_irq_handle(uint8_t pin_number);


#endif /* INC_STM32F446XX_GPIO_H_ */
