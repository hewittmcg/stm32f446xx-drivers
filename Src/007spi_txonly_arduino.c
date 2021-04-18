#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#include <string.h>

// For semihosting
extern void initialise_monitor_handles();


// Code to send a string over SPI to an Arduino slave
// when the onboard button is pressed

// PB12 : SPI2_NSS
// PB13 : SPI2_CLK
// PB14 : SPI2_MISO
// PB15 : SPI2_MOSI
// AF5

// Rough software delay
static void prv_delay(void) {
    for(uint32_t i = 0; i < 500000; i++);
}

// Initialize GPIO for the above settings
void spi2_gpio_init(void) {
	GPIO_handle_t spi_pins = {
		.p_GPIO_x = GPIOB,
		.pin_config = {
			.pin_mode = GPIO_MODE_ALTFN,
			.pin_altfn_mode = 5,
			.pin_op_type = GPIO_OP_TYPE_PP, 
			.pin_pu_pd_control = GPIO_PIN_PU,
			.pin_speed = GPIO_SPEED_FAST,
		},
	};
	
	// NSS
	spi_pins.pin_config.pin_number = GPIO_PIN_12;
	gpio_init(&spi_pins);

	// SCLK
	spi_pins.pin_config.pin_number = GPIO_PIN_13;
	gpio_init(&spi_pins);

	// MISO
	//spi_pins.pin_config.pin_number = GPIO_PIN_14;
	//gpio_init(&spi_pins);

	// MOSI
	spi_pins.pin_config.pin_number = GPIO_PIN_15;
	gpio_init(&spi_pins);

}

// Initialize SPI2
void spi2_init(void) {
	SpiHandle spi_handle = {
		.p_spi_reg = SPI2,
		.spi_config.bus_config = SPI_BUS_CONFIG_FD,
		.spi_config.device_mode = SPI_DEVICE_MODE_MASTER,
		.spi_config.sclk_speed = SPI_SCLK_SPEED_DIV_8, // 2mkz sclk
		.spi_config.dff = SPI_DFF_FORMAT_8_BIT,
		.spi_config.cpol = SPI_CPOL_LOW,
		.spi_config.cpha = SPI_CPHA_LOW,
		.spi_config.ssm = SPI_SSM_CONFIG_DI, // use HW slave management
	};

	spi_init(&spi_handle);
}

// Initialize the onboard button as a GPIO input
void gpio_button_init(void) {
    GPIO_handle_t button;

    // STM32F446RE onboard button config
    button.p_GPIO_x = GPIOC;
    button.pin_config.pin_number = GPIO_PIN_13;
    button.pin_config.pin_mode = GPIO_MODE_IN;
    button.pin_config.pin_speed = GPIO_SPEED_FAST;
    button.pin_config.pin_pu_pd_control = GPIO_NO_PUPD;

    gpio_init(&button);
}

int main(void) {
	gpio_button_init();

	spi2_gpio_init();
	
	spi2_init();

    // enable NSS output by setting SSOE to 1
    spi_ssoe_config(SPI2, ENABLE);

	
	char test_data[] = "hello world";
	uint8_t data_size = strlen(test_data);

    while(1) {
        // wait in busy loop for pin to be pulled low on button press
        while(gpio_read_pin(GPIOC, GPIO_PIN_13));
        prv_delay();
        
        // enable SPI2 peripheral
        spi_peripheral_control(SPI2, ENABLE);

		// send data length in bytes
		spi_send(SPI2, &data_size, 1);

		// send data
        spi_send(SPI2, (uint8_t*)test_data, strlen(test_data));

		// ensure SPI isn't busy
		while(spi_get_flag_status(SPI2, SPI_BUSY_FLAG));

        // disable SPI2 peripheral
        spi_peripheral_control(SPI2, DISABLE);
    }

	return 0;
}
