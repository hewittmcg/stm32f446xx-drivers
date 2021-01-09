#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#include <string.h>

// FW to test SPI communication between STM32 master and Arduino slave

// PB12 : SPI2_NSS
// PB13 : SPI2_CLK
// PB14 : SPI2_MISO
// PB15 : SPI2_MOSI
// AF5


// Command codes
#define COMMAND_LED_CTRL 0x50
#define COMMAND_SENSOR_READ 0x51 
#define COMMAND_LED_READ 0x52
#define COMMAND_PRINT 0x53
#define COMMAND_ID_READ 0x54

#define LED_ON 1
#define LED_OFF 0

// Arduino analog pins
#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOGN_PIN4 4

// Arduino LED
#define LED_PIN 9

// ACK byte
#define ARDUINO_ACK 0xF5

// Verify whether the response from the Arduino 
// constitutes an ACK
#define SPI_VERIFY_RESPONSE(ack_byte) ((ack_byte) == ARDUINO_ACK)


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
	spi_pins.pin_config.pin_number = GPIO_PIN_14;
	gpio_init(&spi_pins);

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
    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read;

	gpio_button_init();

	spi2_gpio_init();
	
	spi2_init();

    // enable NSS output by setting SSOE to 1
    spi_ssoe_config(SPI2, ENABLE);

    while(1) {
        // wait in busy loop for pin to be pulled low on button press
        while(gpio_read_pin(GPIOC, GPIO_PIN_13));
        prv_delay();
        
        // enable SPI2 peripheral
        spi_peripheral_control(SPI2, ENABLE);

		// send CMD_LED_CTRL
        uint8_t command_code = COMMAND_LED_CTRL;
        uint8_t ack_byte;
        uint8_t args[2];

        spi_send(SPI2, &command_code, 1);

        // dummy read to empty RX buffer and clear RXNE
        spi_receive(SPI2, &dummy_read, 1);

        // process ACK/NACK
        // send dummy byte to fetch response from slave
        spi_send(SPI2, &dummy_write, 1);
        spi_receive(SPI2, &ack_byte, 1);
        
        // If ACK OK, send other arguments to turn on/off LED
        if(SPI_VERIFY_RESPONSE(ack_byte)) {
           args[0] = LED_PIN;
           args[1] = LED_ON;

           spi_send(SPI2, args, 2);
        }

		// ensure SPI isn't busy
		while(spi_get_flag_status(SPI2, SPI_BUSY_FLAG));

        // disable SPI2 peripheral
        spi_peripheral_control(SPI2, DISABLE);
    }

	return 0;
}
