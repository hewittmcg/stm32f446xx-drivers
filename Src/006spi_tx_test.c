#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#include <string.h>

// PB12 : SPI2_NSS
// PB13 : SPI2_CLK
// PB14 : SPI2_MISO
// PB15 : SPI2_MOSI
// AF5

// Initialize GPIO for the above settings
void spi2_gpio_init(void) {
	GPIO_handle_t spi_pins = {
		.p_GPIO_x = GPIOB,
		.pin_config = {
			.pin_mode = GPIO_MODE_ALTFN,
			.pin_altfn_mode = 5,
			.pin_op_type = GPIO_OP_TYPE_PP, 
			.pin_pu_pd_control = GPIO_NO_PUPD,
			.pin_speed = GPIO_SPEED_FAST,
		},
	};

	// NOTE: uncomment MISO/NSS code if used
	// NSS
	//spi_pins.pin_config.pin_number = GPIO_PIN_12;
	//gpio_init(&spi_pins);

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
		.spi_config.sclk_speed = SPI_SCLK_SPEED_DIV_2,
		.spi_config.dff = SPI_DFF_FORMAT_8_BIT,
		.spi_config.cpol = SPI_CPOL_LOW,
		.spi_config.cpha = SPI_CPHA_LOW,
		.spi_config.ssm = SPI_SSM_CONFIG_EN,
	};

	spi_init(&spi_handle);
}
int main(void) {
	spi2_gpio_init();
	
	spi2_init();

	// pull NSS high internally to avoid MODF error
	spi_ssi_config(SPI2, ENABLE);

	// enable SPI2 peripheral
	spi_peripheral_control(SPI2, ENABLE);
	
	char test_data[] = "hello world";

	spi_send(SPI2, (uint8_t*)test_data, strlen(test_data));

	while(1);

	return 0;
}
