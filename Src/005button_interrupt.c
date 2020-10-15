// FW to toggle interrupts on the falling edge of the F446RE onboard button being pressed, ]
// and toggle the LED through these interrupts.

#include<string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"


int main(void) {
    GPIO_handle_t gpio_led, button;

    // Init both to 0 
    memset(&gpio_led, 0, sizeof(gpio_led));
    memset(&button, 0, sizeof(button));

    // STM32F446RE onboard LED config
    gpio_led.p_GPIO_x = GPIOA;
    gpio_led.pin_config.pin_number = GPIO_PIN_5;
    gpio_led.pin_config.pin_mode = GPIO_MODE_OUT;
    gpio_led.pin_config.pin_speed = GPIO_SPEED_FAST;
    gpio_led.pin_config.pin_op_type = GPIO_OP_TYPE_PP;
    gpio_led.pin_config.pin_pu_pd_control = GPIO_NO_PUPD;

    // STM32F446RE onboard button config
    button.p_GPIO_x = GPIOC;
    button.pin_config.pin_number = GPIO_PIN_13;
    button.pin_config.pin_mode = GPIO_MODE_IT_FT; // falling edge interrupt config
    button.pin_config.pin_speed = GPIO_SPEED_FAST;
    button.pin_config.pin_pu_pd_control = GPIO_NO_PUPD;

    gpio_pclk_control(GPIOA, ENABLE);
    gpio_init(&gpio_led);

    gpio_pclk_control(GPIOC, ENABLE);
    gpio_init(&button);

    // IRQ config for button pin
    gpio_irq_priority_config(IRQ_POS_EXTI15_10, NVIC_IRQ_PRIORITY_15); // configure priority to 15
    gpio_irq_interrupt_config(IRQ_POS_EXTI15_10, ENABLE);

    while(1);

    return 0;

}

// On button interrupt, toggle LED
void EXTI15_10_IRQHandler(void) {
    gpio_irq_handle(GPIO_PIN_13);
    gpio_toggle_pin(GPIOA, GPIO_PIN_5);
}







