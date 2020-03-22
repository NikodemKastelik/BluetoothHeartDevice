#ifndef BLE_HEART_LIS3DSH_H
#define BLE_HEART_LIS3DSH_H

#include "BLE_Heart_SPI.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_gpiote.h"
#include "math.h"

#ifdef BLE_Heart_REV200
#define SPI_LIS3DSH_CS_PIN 18
#define SPI_LIS3DSH_INT1_PIN 17
#endif

struct LIS3DSH_Data
{
  int16_t X_acc;
  int16_t Y_acc;
  int16_t Z_acc;
};//Data is scaled in [mg]

struct LIS3DSH_Data *  LIS3DSH_Data_Getter(void);
void LIS3DSH_Denit(void);
void LIS3DSH_Init(void);
void LIS3DSH_Check_Presence(void);
void LIS3DSH_Print_data(void);
void LIS3DSH_get_data(void);
bool LIS3DSH_Data_Ready_Getter(void);
#endif /* BLE_HEART_LIS3DSH_H */
