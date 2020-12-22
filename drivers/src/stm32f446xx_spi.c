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

void spi_init(SpiHandle *p_spi_handle) {
	// Configure device mode (see p.g. 886 of reference manual)
	uint32_t temp_reg = 0;

	// Enable peripheral clock
	spi_pclk_control(p_spi_handle->p_spi_reg, ENABLE);

	// Configure device mode
	temp_reg |= (p_spi_handle->spi_config.device_mode << SPI_CR1_MSTR);
	
	// Configure bus config
	if(p_spi_handle->spi_config.bus_config == SPI_BUS_CONFIG_FD) {
		// Clear bi-directional mode
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (p_spi_handle->spi_config.bus_config == SPI_BUS_CONFIG_HD) {
		// Enable bi-directional mode
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
	} else if(p_spi_handle->spi_config.bus_config == SPI_BUS_CONFIG_S_RXONLY) {
		// Clear bi-directional mode, set RXonly bit
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		temp_reg |= (1 << SPI_CR1_RXONLY);
	}

	// Configure SCLK speed (speed configs correspond to 3-bit representation in register)
	temp_reg |= (p_spi_handle->spi_config.sclk_speed << SPI_CR1_BR);

	// Configure DFF
	temp_reg |= (p_spi_handle->spi_config.dff << SPI_CR1_DFF);

	// Configure CPOL
	temp_reg |= (p_spi_handle->spi_config.cpol << SPI_CR1_CPOL);

	// Configure CPHA
	temp_reg |= (p_spi_handle->spi_config.cpha << SPI_CR1_CPHA);

	// Configure SSM
	temp_reg |= (p_spi_handle->spi_config.ssm << SPI_CR1_SSM);

	// Set CR1 register
	p_spi_handle->p_spi_reg->CR1 = temp_reg;
}

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

// Return the status of the flag passed in
static uint8_t prv_spi_get_flag_status(SpiRegDef *p_spi_reg, uint32_t flag_type) {
	if(p_spi_reg->SR & flag_type) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;	
	}
}

void spi_send(SpiRegDef *p_spi_reg, uint8_t *p_tx_buffer, uint32_t len) {
	while(len > 0) {
		// wait for TXE to be set
		while(prv_spi_get_flag_status(p_spi_reg, SPI_TXE_FLAG) == FLAG_RESET);

		// check for DFF bit in CR1
		if(p_spi_reg->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// Load data into DR
			p_spi_reg->DR = *((uint16_t*)p_tx_buffer);
			len -= 2; 
			(uint16_t*)p_tx_buffer++;
		} else {
			// 8-bit DFF
			// Load data into DR
			p_spi_reg->DR = *p_tx_buffer;
			len--;
			p_tx_buffer++;
		}
	}
}

void spi_receive(SpiRegDef *p_spi_reg, uint8_t *p_rx_buffer, uint32_t len);

void spi_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di);

void spi_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority);

void spi_irq_handle(SpiHandle *p_handle);

void spi_peripheral_control(SpiRegDef *p_spi_reg, uint8_t en_or_di) {
	if(en_or_di ==  ENABLE) {
		p_spi_reg->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		p_spi_reg->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
