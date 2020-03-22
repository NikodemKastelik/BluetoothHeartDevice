#ifndef BLE_HEART_SPI_H
#define BLE_HEART_SPI_H


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
#define BLE_Heart_REV200

#ifndef SPI_IRQ_PRIORITY
#define SPI_IRQ_PRIORITY 6
#endif


#ifdef BLE_Heart_REV200
#define SPI_INSTANCE  0 /**< SPI instance index. */
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 20
#define SPI_SCK_PIN  21
#endif

void BLE_Heart_SPI_Init(void);
void LIS3DSH_Check_Presence(void);

#endif /* BLE_HEART_SPI_H */