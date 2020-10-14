#include "stm32f446xx.h" 
#include "stm32f446xx_gpio.h"

// Rough software delay
static void prv_delay(void) {
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void) { 
    GPIO_handle_t gpio_led, button;

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
    button.pin_config.pin_mode = GPIO_MODE_IN;
    button.pin_config.pin_speed = GPIO_SPEED_FAST;
    button.pin_config.pin_pu_pd_control = GPIO_NO_PUPD;

    gpio_pclk_control(GPIOA, ENABLE);
    gpio_init(&gpio_led);

    gpio_pclk_control(GPIOC, ENABLE);
    gpio_init(&button);


    
    while(1) {
    	// When pressed, pin pulls input low
        if(!gpio_read_pin(GPIOC, GPIO_PIN_13)) {
            prv_delay(); // delay to debounce
            gpio_toggle_pin(GPIOA, GPIO_PIN_5);
        }
        
    }

    return 0;

}







