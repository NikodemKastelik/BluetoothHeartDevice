#include "BLE_Heart_SPI.h"

#ifdef BLE_Heart_REV200
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
#endif

volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

void BLE_Heart_SPI_Init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_SPIM_PIN_NOT_CONNECTED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
  //  spi_config.mode =NRF_DRV_SPI_MODE_0 ;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));


}