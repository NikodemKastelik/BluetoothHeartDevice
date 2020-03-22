#include "BLE_Heart_LIS3DSH.h"

static uint8_t m_rx_buf[10];
static uint8_t m_tx_buf[10];
static bool LIS_3DSH_Presence;
extern volatile bool spi_xfer_done; //Extern from BLE_Heart_SPI.c
extern const nrf_drv_spi_t spi;     //Extern from BLE_Heart_SPI.c

static struct LIS3DSH_Data mLIS3DSH_Data;
static bool LIS3DSH_Data_ready;

void LIS3DSH_INT1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    LIS3DSH_Data_ready=1;
}

struct LIS3DSH_Data *  LIS3DSH_Data_Getter(void)
{
  return &mLIS3DSH_Data;
}

bool LIS3DSH_Data_Ready_Getter(void)
{
  return LIS3DSH_Data_ready;
}

void LIS3DSH_Check_Presence(void)
{
    spi_xfer_done = false;

        nrf_gpio_pin_clear(SPI_LIS3DSH_CS_PIN);
        m_tx_buf[0] = 0x8f; //Read WHO AM I Register
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 1, m_rx_buf, 2));
        while (!spi_xfer_done)
        {
            __WFE();
        }

       nrf_gpio_pin_set(SPI_LIS3DSH_CS_PIN);

       if(m_rx_buf[1]== 0x3f)
       {
        NRF_LOG_INFO("LIS3DSH Present!");
        LIS_3DSH_Presence=1;
       }else
       {
        NRF_LOG_INFO("LIS3DSH not detected!");
        LIS_3DSH_Presence=0;
       }
        NRF_LOG_FLUSH();
}

void LIS3DSH_Denit(void)
{
    nrf_gpio_cfg_default(SPI_LIS3DSH_CS_PIN);
    m_tx_buf[0] = 0x20; // Write CTRL_REG4
    m_tx_buf[1] = 0x00; // Disable Accelerometer

        spi_xfer_done = false;
        nrf_gpio_pin_clear(SPI_LIS3DSH_CS_PIN);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 0));
        while (!spi_xfer_done)
        {
            __WFE();
        }
       nrf_gpio_pin_set(SPI_LIS3DSH_CS_PIN);

}

void LIS3DSH_Init(void)
{
    nrf_gpio_cfg_output(SPI_LIS3DSH_CS_PIN);

   // ret_code_t err_code = nrf_drv_gpiote_init();
   // APP_ERROR_CHECK(err_code);
    LIS3DSH_Check_Presence();



    if(LIS_3DSH_Presence ==0) return;
    m_tx_buf[0] = 0x20; // Write CTRL_REG4
    m_tx_buf[1] = 0b01000111; //25Hz Sampling rate, All axes enabled

        spi_xfer_done = false;
        nrf_gpio_pin_clear(SPI_LIS3DSH_CS_PIN);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 0));
        while (!spi_xfer_done)
        {
            __WFE();
        }
       nrf_gpio_pin_set(SPI_LIS3DSH_CS_PIN);



    m_tx_buf[0] = 0x23; // Write CTRL_REG3
    m_tx_buf[1] = 0b11101000; //INT 1 EN as data ready.

        spi_xfer_done = false;
        nrf_gpio_pin_clear(SPI_LIS3DSH_CS_PIN);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 0));
        while (!spi_xfer_done)
        {
            __WFE();
        }
       nrf_gpio_pin_set(SPI_LIS3DSH_CS_PIN);

    ret_code_t err_code;

//    err_code = nrf_drv_gpiote_init();
//    APP_ERROR_CHECK(err_code);

//    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
//
//    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
//    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(SPI_LIS3DSH_INT1_PIN, &in_config, LIS3DSH_INT1_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(SPI_LIS3DSH_INT1_PIN, true);
}


void LIS3DSH_Print_data(void)
{
    if(LIS_3DSH_Presence ==0) 
    {
      NRF_LOG_INFO("LIS3DSH not detected/initialized !!");
      NRF_LOG_FLUSH();
      return;
    }

       NRF_LOG_INFO("X: %d Y: %d Z: %d [mg]",mLIS3DSH_Data.X_acc,mLIS3DSH_Data.Y_acc,mLIS3DSH_Data.Z_acc);
       NRF_LOG_FLUSH();
}

void LIS3DSH_get_data(void)
{
    if(LIS_3DSH_Presence ==0) return;
    LIS3DSH_Data_ready=0;
    m_tx_buf[0] = 0b10101001; // Read OUT_X_H
        spi_xfer_done = false;
        nrf_gpio_pin_clear(SPI_LIS3DSH_CS_PIN);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 1, m_rx_buf, 7));
        while (!spi_xfer_done)
        {
            __WFE();
        }
       nrf_gpio_pin_set(SPI_LIS3DSH_CS_PIN);
       mLIS3DSH_Data.X_acc = ((int16_t)m_rx_buf[1]<<8) | m_rx_buf[2] ;
       mLIS3DSH_Data.Y_acc = ((int16_t)m_rx_buf[3]<<8) | m_rx_buf[4] ;
       mLIS3DSH_Data.Z_acc = ((int16_t)m_rx_buf[5]<<8) | m_rx_buf[6] ;

       mLIS3DSH_Data.X_acc=mLIS3DSH_Data.X_acc*2/32.768;
       mLIS3DSH_Data.Y_acc=mLIS3DSH_Data.Y_acc*2/32.768;
       mLIS3DSH_Data.Z_acc=mLIS3DSH_Data.Z_acc*2/32.768;

}