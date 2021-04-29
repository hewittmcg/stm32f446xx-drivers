/*
 * 009_spi_message_rcv_it.c
 *
 *  Created on: Apr. 18, 2021
 *      Author: Hewitt McGaughey
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

// For semihosting
extern void initialise_monitor_handles(void);

static SpiHandle s_spi2_handle;

#define MSG_MAX_LEN 500

char rx_buf[MSG_MAX_LEN];

volatile char read_byte;

volatile uint8_t rx_stop = 0;

// Flag set in interrupt handler of Arduino interrupt GPIO
volatile uint8_t data_available = 0;

// Rough delay function
static void prv_delay(void) {
    for(uint32_t i = 0; i < 250000; i++);
}

// PB12 : SPI2_NSS
// PB13 : SPI2_CLK
// PB14 : SPI2_MISO
// PB15 : SPI2_MOSI
// ALT function mode 5
static void prv_spi2_gpio_init(void) {
    GPIO_handle_t spi_pins = {
        .p_GPIO_x = GPIOB,
        .pin_config.pin_mode = GPIO_MODE_ALTFN,
        .pin_config.pin_altfn_mode = 5,
        .pin_config.pin_op_type = GPIO_OP_TYPE_PP,
        .pin_config.pin_pu_pd_control = GPIO_NO_PUPD,
        .pin_config.pin_speed = GPIO_SPEED_FAST,
    };

    // SCLK
    spi_pins.pin_config.pin_number = GPIO_PIN_13;
    gpio_init(&spi_pins);

    // MOSI
    spi_pins.pin_config.pin_number = GPIO_PIN_15;
    gpio_init(&spi_pins);

    // MISO
    spi_pins.pin_config.pin_number = GPIO_PIN_14;
    gpio_init(&spi_pins);

    // NSS
    spi_pins.pin_config.pin_number = GPIO_PIN_12;
    gpio_init(&spi_pins);
}

static void prv_spi2_init(void) {
    s_spi2_handle.p_spi_reg = SPI2;
    s_spi2_handle.spi_config.bus_config = SPI_BUS_CONFIG_FD;
    s_spi2_handle.spi_config.device_mode = SPI_DEVICE_MODE_MASTER;
    s_spi2_handle.spi_config.sclk_speed = SPI_SCLK_SPEED_DIV_32;
    s_spi2_handle.spi_config.dff = SPI_DFF_FORMAT_8_BIT;
    s_spi2_handle.spi_config.cpol = SPI_CPOL_LOW;
    s_spi2_handle.spi_config.cpha = SPI_CPHA_LOW;
    s_spi2_handle.spi_config.ssm = SPI_SSM_CONFIG_DI; // hardware slave management enabled

    spi_init(&s_spi2_handle);
}

// Configure GPIO pin for SPI peripheral to issue data avaiable interrupt
static void prv_slave_gpio_it_pin_init(void) {
    GPIO_handle_t it_pin;
    memset(&it_pin, 0, sizeof(it_pin));

    // same GPIO configuration as LED
    it_pin.p_GPIO_x = GPIOD;
    it_pin.pin_config.pin_number = GPIO_PIN_4;
    it_pin.pin_config.pin_mode = GPIO_MODE_IT_FT;
    it_pin.pin_config.pin_speed = GPIO_SPEED_LOW;
    it_pin.pin_config.pin_pu_pd_control = GPIO_PIN_PU;

    gpio_init(&it_pin);

    gpio_irq_priority_config(IRQ_POS_EXTI9_5, NVIC_IRQ_PRIORITY_15);
    gpio_irq_interrupt_config(IRQ_POS_EXTI9_5, ENABLE);
}

int main(void) {
    printf("in main\n");
    uint8_t dummy = 0xFF;

    prv_slave_gpio_it_pin_init();

    // Initialize SPI2
    prv_spi2_gpio_init();
    prv_spi2_init();

    // Make SSOE 1 to enable NSS output
    spi_ssoe_config(SPI2, ENABLE);

    spi_irq_interrupt_config(IRQ_POS_SPI2, ENABLE);

    while(1) {
        rx_stop = 0;

        // Wait for data to be available from slave
        while(!data_available);

        // Disable interrupts
        gpio_irq_interrupt_config(IRQ_POS_EXTI9_5, DISABLE);

        // Enable SPI2 peripheral
        spi_peripheral_control(SPI2, ENABLE);

        while(!rx_stop) {
            // fetch data byte by byte in interrupt mode
            while(spi_send_it(&s_spi2_handle, &dummy, 1) == STATUS_FAILED);
            while(spi_send_it(&s_spi2_handle, &read_byte, 1) == STATUS_FAILED);
        }

        // confirm SPI not busy
        while(spi_get_flag_status(SPI2, SPI_BUSY_FLAG));

        // Disable SPI2 peripheral
        spi_peripheral_control(SPI2, DISABLE);
        printf("Received data: %s\n", rx_buf);

        data_available = 0;

        // Re-enable interrupts
        gpio_irq_interrupt_config(IRQ_POS_EXTI9_5, ENABLE);
    }

    return 0;
}

// Runs when data byte received from perhiperal over SPI
void SPI2_IRQHandler(void) {
    spi_irq_handle(&s_spi2_handle);
}

// Implemented to handle RX_DONE events
void spi_app_event_cb(SpiHandle *p_spi_handle, uint8_t event) {
    static uint32_t i = 0;
    // If RX complete, copy data to rx buffer
    if(event == SPI_EVENT_RX_DONE) {
        rx_buf[i++] = read_byte;

        // Stop reading when null terminating char sent or max length hit
        if(read_byte == '\0' || (i == MSG_MAX_LEN)) {
            rx_stop = 1;
            rx_buf[i-1] = '\0';
            i = 0;
        }
    }
}

// Slave data available interrupt handler
void EXTI9_5_IRQHandler(void) {
    gpio_irq_handle(GPIO_PIN_6);
    data_available = 1;
}


