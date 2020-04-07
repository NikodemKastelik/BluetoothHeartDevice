#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

#define WS2812_PIN_ENABLE   29
#define WS2812_I2S_PIN_DATA 28
#define WS2812_I2S_PIN_SCK  26
#define WS2812_I2S_PIN_LRCK 25

#define DEVICE_NAME                    "Custom_Service"                        /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO          3                                       /**< Application's BLE observer priority.*/
#define APP_BLE_CONN_CFG_TAG           1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL               64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION               BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                  0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification)
                                                                                *   to first time sd_ble_gap_conn_param_update is called (15 seconds).
                                                                                */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT   3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                      0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#endif // APP_CONFIG_H__
