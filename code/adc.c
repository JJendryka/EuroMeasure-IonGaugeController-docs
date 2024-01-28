#pragma GCC push_options
#pragma GCC optimize ("O3")
// Wait n*5+15
void sleep_some_cycles(uint32_t n) {
    while (n > 0) {
        n--;
        __NOP();
    }
}
#pragma GCC pop_options

static inline void spi_start_transaction(SPI_t * instance) {
    HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_RESET);
    sleep_some_cycles(instance->cs_to_sck_delay);
    SET_BIT(instance->spi->CR1, SPI_CR1_SPE);
}

static inline void spi_stop_transaction(SPI_t * instance) {
    CLEAR_BIT(instance->spi->CR1, SPI_CR1_SPE);
    sleep_some_cycles(instance->sck_to_cs_delay);
    HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_SET);
}

void spi_transaction(SPI_t * instance, uint8_t * tx_buf, uint32_t tx_length, uint8_t * rx_buf, uint32_t rx_length) {
    spi_start_transaction(instance);

    uint32_t tx_remaining_length = tx_length >= rx_length ? tx_length : rx_length;
    uint32_t rx_remaining_length = tx_remaining_length;
    while (rx_remaining_length > 0) {
        // TX fifo not full
        if ((instance->spi->SR & SPI_SR_FTLVL_Msk) != SPI_FTLVL_FULL) {
            // Still have data to send
            if (tx_length) {
                *(uint8_t *) &instance->spi->DR = (*tx_buf); // Needs to be cast to 8 bit to send in 8 bit mode
                tx_buf++;
                tx_length--;
                tx_remaining_length--;
            }
            // No more data to send, sending 0 to clock out all remaining data
            else if (tx_remaining_length) {
                *(uint8_t *) &instance->spi->DR = 0; // Needs to be cast to 8 bit to send in 8 bit mode
                tx_remaining_length--;
            }
        }

        // Data ready in receive fifo
        if(instance->spi->SR & SPI_FLAG_RXNE) {
            // There is still new data expected
            if (rx_length) {
                (*rx_buf) = *(volatile uint8_t *)&instance->spi->DR;
                rx_buf++;
                rx_length--;
                rx_remaining_length--;
            }
            // No new data expected, ignoring
            else {
                volatile uint8_t _ = *(volatile uint8_t *)&instance->spi->DR;
                rx_remaining_length--;
            }
        }
    }

    spi_stop_transaction(instance);
}

uint8_t ads_read_voltage(ads_adc_t * adc, sample_t *sample)
{
    if(adc == NULL){
        return 1;
    }
    static ads_response_frame_t buf;
    spi_transaction(&adc->spi, NULL, 0, (uint8_t *) &buf, sizeof(ads_response_frame_t));
    for(int i = 0; i < 4; i++)
    {
        sample->value[i] = ((int32_t)((buf.channels[3*i + 0] << 16 | buf.channels[3*i + 1] << 8 | buf.channels[3*i+2])<<8))>>8;
    }
    return 0;
}
