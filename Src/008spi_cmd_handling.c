#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#include <string.h>
#include <stdio.h>

// For semihosting
extern void initialise_monitor_handles(void);

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

// Length of board ID
#define BOARD_ID_SIZE 10


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

// Send CMD_LED_CTRL (turn off LED at pin LED_PIN)
// This function also enables SPI2.
static void prv_send_cmd_led_ctrl(void) {
    uint8_t command_code = COMMAND_LED_CTRL;
    uint8_t ack_byte, dummy_read;
    uint8_t dummy_write = 0xFF;
    uint8_t args[2];

    // wait for button press
    while(gpio_read_pin(GPIOC, GPIO_PIN_13));
    printf("Sending CMD_LED_CTRL\n");
    prv_delay();
    
    // enable SPI2 peripheral
    spi_peripheral_control(SPI2, ENABLE);

    spi_send(SPI2, &command_code, 1);

    // dummy read to empty RX buffer and clear RXNE
    spi_receive(SPI2, &dummy_read, 1);

    // process ACK/NACK
    // send dummy byte to fetch response from slave
    spi_send(SPI2, &dummy_write, 1);
    spi_receive(SPI2, &ack_byte, 1);
    
    // If ACK OK, send other arguments to turn on/off LED
    if(SPI_VERIFY_RESPONSE(ack_byte)) {
        printf("ACK OK\n");
        args[0] = LED_PIN;
        args[1] = LED_OFF;

        spi_send(SPI2, args, 2);
    }
}

// Send CMD_SENSOR_READ (read from analog pin 0)
static void prv_send_cmd_sensor_read(void) {
    uint8_t command_code = COMMAND_SENSOR_READ;
    uint8_t ack_byte, dummy_read;
    uint8_t dummy_write = 0xFF;
    uint8_t args[2];

    while(gpio_read_pin(GPIOC, GPIO_PIN_13));
    printf("Sending CMD_SENSOR_READ\n");
    prv_delay();

    spi_send(SPI2, &command_code, 1);

    // dummy read to empty RX buffer and clear RXNE
    spi_receive(SPI2, &dummy_read, 1);

    // process ACK/NACK
    // send dummy byte to fetch response from slave
    spi_send(SPI2, &dummy_write, 1);
    spi_receive(SPI2, &ack_byte, 1);
    
    // If ACK OK, send other arguments to read from analog pin 0
    if(SPI_VERIFY_RESPONSE(ack_byte)) {
        printf("ACK OK\n");
        args[0] = ANALOG_PIN0;

        spi_send(SPI2, args, 1);
        // dummy read to empty RX buffer and clear RXNE
        spi_receive(SPI2, &dummy_read, 1);

        // delay for slave to get data
        prv_delay();

        // send dummy byte to fetch response from slave
        spi_send(SPI2, &dummy_write, 1);

        uint8_t analog_data;
        spi_receive(SPI2, &analog_data, 1);
        printf("Analog data: %d\n", analog_data);
    }
}

// Send CMD_LED_READ to read the status of an LED connected to the Arduino
static void prv_send_cmd_led_read(void) {
    uint8_t command_code = COMMAND_LED_READ;
    uint8_t ack_byte, dummy_read;
    uint8_t dummy_write = 0xFF;
    uint8_t args[2];

    while(gpio_read_pin(GPIOC, GPIO_PIN_13));
    printf("Sending CMD_LED_READ\n");
    prv_delay();

    spi_send(SPI2, &command_code, 1);

    // dummy read to empty RX buffer and clear RXNE
    spi_receive(SPI2, &dummy_read, 1);

    // process ACK/NACK
    // send dummy byte to fetch response from slave
    spi_send(SPI2, &dummy_write, 1);
    spi_receive(SPI2, &ack_byte, 1);
    
    // If ACK OK, send argument with pin to read
    if(SPI_VERIFY_RESPONSE(ack_byte)) {
        args[0] = LED_PIN;

        spi_send(SPI2, args, 1);

        // dummy read to empty RX buffer and clear RXNE
        spi_receive(SPI2, &dummy_read, 1);

        // delay for slave to get data
        prv_delay();

        // send dummy byte to fetch response from slave
        spi_send(SPI2, &dummy_write, 1);

        uint8_t led_data;
        spi_receive(SPI2, &led_data, 1);
        printf("LED data: %d\n", led_data);
    }
}

// Send a message to be printed by the Arduino.
static void prv_send_cmd_print(void) {
    uint8_t command_code = COMMAND_PRINT;
    uint8_t ack_byte, dummy_read;
    uint8_t dummy_write = 0xFF;

    char msg[] = "Message from STM32";
    uint8_t msg_size = strlen(msg);

    while(gpio_read_pin(GPIOC, GPIO_PIN_13));
    printf("Sending CMD_PRINT\n");
    prv_delay();

    spi_send(SPI2, &command_code, 1);

    // dummy read to empty RX buffer and clear RXNE
    spi_receive(SPI2, &dummy_read, 1);

    // process ACK/NACK
    // send dummy byte to fetch response from slave
    spi_send(SPI2, &dummy_write, 1);
    spi_receive(SPI2, &ack_byte, 1);
    
    // If ACK OK, send message size followed by message
    if(SPI_VERIFY_RESPONSE(ack_byte)) {
        printf("ACK OK\n");
        spi_send(SPI2, &msg_size, 1);

        spi_send(SPI2, (uint8_t*)msg, msg_size);
    }
}

// Receive the ID of the Arduino over SPI.
static void prv_send_cmd_id_read(void) {
    uint8_t command_code = COMMAND_ID_READ;
    uint8_t ack_byte, dummy_read;
    uint8_t dummy_write = 0xFF;
    char board_id[BOARD_ID_SIZE + 1]; 
    
    while(gpio_read_pin(GPIOC, GPIO_PIN_13));
    printf("Sending CMD_ID_READ\n");
    prv_delay();

    spi_send(SPI2, &command_code, 1);

    // dummy read to empty RX buffer and clear RXNE
    spi_receive(SPI2, &dummy_read, 1);

    // process ACK/NACK
    // send dummy byte to fetch response from slave
    spi_send(SPI2, &dummy_write, 1);
    spi_receive(SPI2, &ack_byte, 1);
    
    // If ACK OK, send other arguments to read from analog pin 0
    if(SPI_VERIFY_RESPONSE(ack_byte)) {
        printf("ACK OK\n");
        // continue fetching the board ID until no response given
        for(uint8_t i = 0; i < BOARD_ID_SIZE; i++) {
            // send dummy byte to fetch response from slave
            spi_send(SPI2, &dummy_write, 1);
            spi_receive(SPI2, &(board_id[i]), 1);
        }
        // append null-terminating char
        board_id[BOARD_ID_SIZE] = '\0';
        
        printf("ID: %s\n", board_id);
    }
}

int main(void) {
    // for semihosting
    initialise_monitor_handles();

    printf("Starting setup...\n");
	gpio_button_init();

	spi2_gpio_init();
	
	spi2_init();

    // enable NSS output by setting SSOE to 1
    spi_ssoe_config(SPI2, ENABLE);
    
    printf("SPI and GPIO initialized\n");

    while(1) {
        prv_send_cmd_led_ctrl();

        prv_send_cmd_sensor_read();

        prv_send_cmd_led_read();

        prv_send_cmd_print();

        prv_send_cmd_id_read();

		// ensure SPI isn't busy
		while(spi_get_flag_status(SPI2, SPI_BUSY_FLAG));

        // disable SPI2 peripheral
        spi_peripheral_control(SPI2, DISABLE);
    }

	return 0;
}
