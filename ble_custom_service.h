/**
 * @defgroup ble_custom_service Custom Service
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief Custom Service module.
 *
 * @details This module implements a custom service with an RGB LED and Battery Characteristics.
 *
 *          The application must supply an event handler for receiving Custom Service
 *          events. Using this handler, the service notifies the application when the
 *          RGB LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of battery charge level to connected peers.
 */

#ifndef BLE_CUSTOM_SERVICE_H__
#define BLE_CUSTOM_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Symbol specifying Custom Service BLE observer priority. */
#define BLE_CUSTOM_SERVICE_OBSERVER_PRIO 2

/**
 * @brief Macro for defining a ble_custom_service instance.
 *
 * @param[in] _name Name of the instance.
 *
 * @hideinitializer
 */
#define BLE_CUSTOM_SERVICE_DEF(_name)                  \
static ble_custom_service_t _name;                     \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                    \
                     BLE_CUSTOM_SERVICE_OBSERVER_PRIO, \
                     ble_custom_service_on_ble_evt, &_name)

#define CUSTOM_SERVICE_UUID_BASE     {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                      0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define CUSTOM_SERVICE_UUID_SERVICE   0x1337
#define CUSTOM_SERVICE_UUID_BATT_CHAR 0x1338
#define CUSTOM_SERVICE_UUID_RGB_CHAR  0x1339

// Forward declaration of the ble_custom_service_t type.
typedef struct ble_custom_service_s ble_custom_service_t;

typedef void (*ble_rgb_write_handler_t)(uint16_t               conn_handle,
                                        ble_custom_service_t * p_service,
                                        uint8_t const *        rgb_values);

/** @brief Custom Service config structure.*/
typedef struct
{
    ble_rgb_write_handler_t rgb_write_handler; /**< Event handler to be called when the RGB LED Characteristic is written. */
} ble_custom_service_config_t;

/** @brief LED Button Service structure. */
struct ble_custom_service_s
{
    uint16_t                 service_handle;    /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t rgb_char_handles;  /**< Handles related to the RGB LED Characteristic. */
    ble_gatts_char_handles_t batt_char_handles; /**< Handles related to the Battery Characteristic. */
    uint8_t                  uuid_type;         /**< UUID type for the Custom Service. */
    ble_rgb_write_handler_t  rgb_write_handler; /**< Event handler to be called when the RGB LED Characteristic is written. */
};

/**
 * @brief Function for initializing the Custom Service.
 *
 * @param[out] p_service Pointer to the Custom Service structure
 *                       This structure must be supplied by the application.
 *                       It is initialized by this function and will later
 *                       be used to identify this particular service instance.
 * @param[in]  p_config  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully.
 *                     Otherwise, an error code is returned.
 */
uint32_t ble_custom_service_init(ble_custom_service_t *              p_service,
                                 ble_custom_service_config_t const * p_config);

/**
 * @brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Custom Service.
 *
 * @param[in] p_ble_evt Event received from the BLE stack.
 * @param[in] p_context Pointer to the Custom Service structure.
 */
void ble_custom_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**
 * @brief Function for sending a battery change notification.
 *
 * @param[in] conn_handle Handle of the peripheral connection to which the battery change notification will be sent.
 * @param[in] p_service   Pointer to the Custom Service structure.
 * @param[in] batt_value  New battery charge value.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_custom_service_on_battery_change(uint16_t               conn_handle,
                                              ble_custom_service_t * p_service,
                                              uint8_t                batt_value);

#ifdef __cplusplus
}
#endif

#endif // BLE_CUSTOM_SERVICE_H__

/** @} */
