#include "stm32f446xx_spi.h"

void spi_pclk_control(SpiRegDef *p_spi_reg, uint8_t en_or_di) {
    if(en_or_di == ENABLE) {
		if(p_spi_reg ==  SPI1)
			SPI1_PCLK_EN();

		else if(p_spi_reg ==  SPI2)
			SPI2_PCLK_EN();

		else if(p_spi_reg ==  SPI3)
			SPI3_PCLK_EN();

		else if(p_spi_reg ==  SPI4)
			SPI4_PCLK_EN();

	} else {
		if(p_spi_reg ==  SPI1)
			SPI1_PCLK_DI();
            
		else if(p_spi_reg ==  SPI2)
			SPI2_PCLK_DI();

		else if(p_spi_reg ==  SPI3)
			SPI3_PCLK_DI();

		else if(p_spi_reg ==  SPI4)
			SPI4_PCLK_DI();
	}
}

void spi_init(SpiHandle *p_spi_handle);

void spi_deinit(SpiRegDef *p_spi_reg) {
    if(p_spi_reg == SPI1) {
        SPI1_REG_RESET();
    } else if(p_spi_reg == SPI2) {
        SPI2_REG_RESET();
    } else if(p_spi_reg == SPI3) {
        SPI3_REG_RESET();
    } else if(p_spi_reg == SPI4) {
        SPI4_REG_RESET();
    }
}

void spi_send(SpiRegDef *p_spi_reg, uint8_t *p_tx_buffer, uint32_t len);

void spi_receive(SpiRegDef *p_spi_reg, uint8_t *p_rx_buffer, uint32_t len);

void spi_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

void spi_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority);

void spi_irq_handle(SpiHandle *p_handle);
