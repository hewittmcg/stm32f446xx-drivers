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
uint8_t spi_get_flag_status(SpiRegDef *p_spi_reg, uint32_t flag_type) {
	if(p_spi_reg->SR & flag_type) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;	
	}
}

void spi_clear_ovr_flag(SpiRegDef *p_spi_reg) {
	// Read registers to clear flag
	uint8_t temp;
	temp = p_spi_reg->DR;
	temp = p_spi_reg->SR;
	(void)temp; // avoid unused warnings
}

void spi_end_tx(SpiHandle *p_spi_handle) {
	// Prevent setting of TXEIE flag
	p_spi_handle->p_spi_reg->CR2 &= ~(1 << SPI_CR2_TXEIE);
	p_spi_handle->p_tx_buffer = NULL;
	p_spi_handle->tx_len = 0;
	p_spi_handle->tx_state = SPI_STATE_READY;
	spi_app_event_cb(p_spi_handle, SPI_EVENT_TX_DONE);
}

void spi_end_rx(SpiHandle *p_spi_handle) {
	// Prevent setting of RXNEIE flag
	p_spi_handle->p_spi_reg->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	p_spi_handle->p_rx_buffer = NULL;
	p_spi_handle->rx_len = 0;
	p_spi_handle->rx_state = SPI_STATE_READY;
	spi_app_event_cb(p_spi_handle, SPI_EVENT_RX_DONE);
}

void spi_send(SpiRegDef *p_spi_reg, uint8_t *p_tx_buffer, uint32_t len) {
	while(len > 0) {
		// wait for TXE to be set
		while(spi_get_flag_status(p_spi_reg, SPI_TXE_FLAG) == FLAG_RESET);

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

void spi_receive(SpiRegDef *p_spi_reg, uint8_t *p_rx_buffer, uint32_t len) {
	while(len > 0) {
		// wait for RXNE to be set
		while(spi_get_flag_status(p_spi_reg, SPI_RXNE_FLAG) == FLAG_RESET);

		// check for DFF bit in CR1
		if(p_spi_reg->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// Load data from DR into RX buffer
			*((uint16_t*)p_rx_buffer) = p_spi_reg->DR;
			len -= 2; 
			(uint16_t*)p_rx_buffer++;
		} else {
			// 8-bit DFF
			// Load data from DR
			*(p_rx_buffer) = p_spi_reg->DR;
			len--;
			p_rx_buffer++;
		}
	}
}

uint8_t spi_send_it(SpiHandle *p_spi_handle, uint8_t *p_tx_buffer, uint32_t len) {
	uint8_t state = p_spi_handle->tx_state;
	if(state != SPI_STATE_BUSY_TX) {
		// Save tx buffer address, len
		p_spi_handle->p_tx_buffer = p_tx_buffer;
		p_spi_handle->tx_len = len;

		// TX state busy
		p_spi_handle->tx_state = SPI_STATE_BUSY_TX;

		// Enable TXEIE control bit to get interrupt when TXE flag set in SR
		p_spi_handle->p_spi_reg->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data transmission handled by ISR implementation
		return STATUS_SUCCESS;
	}
	return STATUS_FAILED;
}

uint8_t spi_receive_it(SpiHandle *p_spi_handle, uint8_t *p_rx_buffer, uint32_t len) {
	uint8_t state = p_spi_handle->rx_state;
	if(state != SPI_STATE_BUSY_RX) {
		// Save tx buffer address, len
		p_spi_handle->p_rx_buffer = p_rx_buffer;
		p_spi_handle->rx_len = len;

		// TX state busy
		p_spi_handle->rx_state = SPI_STATE_BUSY_RX;

		// Enable RXNEIE control bit to get interrupt when RXNE flag set in SR
		p_spi_handle->p_spi_reg->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data transmission handled by ISR implementation
		return STATUS_SUCCESS;
	}
	return STATUS_FAILED;
}

void spi_irq_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di) {
	if(en_or_di == ENABLE) {
		if(IRQ_number <= NVIC_ISER0_MAX_PRIORITY) {
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQ_number);
		} else if(IRQ_number < NVIC_ISER1_MAX_PRIORITY) {
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQ_number % 32));
		} else if(IRQ_number < NVIC_ISER2_MAX_PRIORITY) {
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQ_number % 64));
		}
	} else {
		if(IRQ_number <= NVIC_ICER0_MAX_PRIORITY) {
			// Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQ_number);
		} else if(IRQ_number < NVIC_ICER1_MAX_PRIORITY) {
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQ_number % 32));
		} else if(IRQ_number < NVIC_ICER2_MAX_PRIORITY) {
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQ_number % 64));
		}
	}
}

void spi_irq_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority) {
	// Determine IPR register
	uint8_t ipr = IRQ_number / 4;
	uint8_t ipr_section = IRQ_number % 4;

	// Jump to ipr register address and flip bits corresponding to priority 
	// See p.g. 4-7 of generic user guide for more information
	uint8_t shift_amt = (8 * ipr_section) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + ipr) |= IRQ_priority << shift_amt;
}

// Handle TXE interrupt
static void prv_spi_handle_txe_it(SpiHandle *p_spi_handle) {
	// check for DFF bit in CR1
	if(p_spi_handle->p_spi_reg->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// Load data into DR
		p_spi_handle->p_spi_reg->DR = *((uint16_t*)p_spi_handle->p_tx_buffer);
		p_spi_handle->tx_len -= 2; 
		(uint16_t*)p_spi_handle->p_tx_buffer++;
	} else {
		// 8-bit DFF
		// Load data into DR
		p_spi_handle->p_spi_reg->DR = *p_spi_handle->p_tx_buffer;
		p_spi_handle->tx_len--;
		p_spi_handle++;
	}

	// End SPI transmission if tx buffer empty
	if(p_spi_handle->tx_len == 0) {
		spi_end_tx(p_spi_handle);
	}
}

// Handle RXNE interrupt
static void prv_spi_handle_rxne_it(SpiHandle *p_spi_handle) {
	// check for DFF bit in CR1
	if(p_spi_handle->p_spi_reg->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// Load data from DR into RX buffer
		*((uint16_t*)p_spi_handle->p_rx_buffer) = p_spi_handle->p_spi_reg->DR;
		p_spi_handle->rx_len -= 2; 
		(uint16_t*)p_spi_handle->p_rx_buffer++;
	} else {
		// 8-bit DFF
		// Load data into DR
		*(p_spi_handle->p_rx_buffer) = p_spi_handle->p_spi_reg->DR;
		p_spi_handle->rx_len--;
		p_spi_handle->p_rx_buffer++;
	}

	// End SPI transmission if RX buffer empty
	if(p_spi_handle->rx_len == 0) {
		spi_end_rx(p_spi_handle);
	}
}

// Handle OVR interrupt
static void prv_spi_handle_ovr_it(SpiHandle *p_spi_handle) {
	// Clear OVR flag
	if(p_spi_handle->tx_state != SPI_STATE_BUSY_TX) {
		spi_clear_ovr_flag(p_spi_handle->p_spi_reg);
	}

	// Broadcast to application
	// If communcation in progress, application will have to clear flag itself 
	// by calling spi_clear_ovr_flag()
	spi_app_event_cb(p_spi_handle, SPI_EVENT_OVR_ERR);
}

void spi_irq_handle(SpiHandle *p_spi_handle) {
	// Check for TXE
	uint8_t txe_status = p_spi_handle->p_spi_reg->SR & (1 <<  SPI_SR_TXE);
	uint8_t txeie_status = p_spi_handle->p_spi_reg->CR2 & (1 <<  SPI_CR2_TXEIE);

	if(txe_status && txeie_status) {
		// Handle TXE
		prv_spi_handle_txe_it(p_spi_handle);
	}

	// Check for RXNE
	uint8_t rxne_status = p_spi_handle->p_spi_reg->SR & (1 << SPI_SR_RXNE);
	uint8_t rxneie_status = p_spi_handle->p_spi_reg->CR2 & (1 << SPI_CR2_RXNEIE);

	if(rxne_status && rxneie_status) {
		// Handle RXNE
		prv_spi_handle_rxne_it(p_spi_handle);
	}

	// Check for overrun error
	uint8_t ovr_status = p_spi_handle->p_spi_reg->SR & (1 << SPI_SR_OVR);
	uint8_t errie_status = p_spi_handle->p_spi_reg->CR2 & (1 << SPI_CR2_ERRIE);

	if(ovr_status && errie_status) {
		// Handle OVR
		prv_spi_handle_ovr_it(p_spi_handle);
	}

	// NOTE: MODF, CRC, FRE error handling unimplemented here
}

void spi_peripheral_control(SpiRegDef *p_spi_reg, uint8_t en_or_di) {
	if(en_or_di ==  ENABLE) {
		p_spi_reg->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		p_spi_reg->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void spi_ssi_config(SpiRegDef *p_spi_reg, uint8_t en_or_di) {
	if(en_or_di ==  ENABLE) {
		p_spi_reg->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		p_spi_reg->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void spi_ssoe_config(SpiRegDef *p_spi_reg, uint8_t en_or_di) {
	if(en_or_di ==  ENABLE) {
		p_spi_reg->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		p_spi_reg->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

__weak void spi_app_event_cb(SpiHandle *p_spi_handle, uint8_t event);

