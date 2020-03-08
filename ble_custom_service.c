#include "sdk_common.h"
#include "ble_custom_service.h"
#include "ble_srv_common.h"

/**
 * @brief Function for handling the Write event.
 *
 * @param[in] p_service Pointer to the Custom Service structure.
 * @param[in] p_ble_evt Event received from the BLE stack.
 */
static void on_write(ble_custom_service_t * p_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_service->rgb_char_handles.value_handle) &&
        (p_evt_write->len == 3) &&
        (p_service->rgb_write_handler != NULL))
    {
        p_service->rgb_write_handler(p_ble_evt->evt.gap_evt.conn_handle,
                                     p_service,
                                     p_evt_write->data);
    }
}

void ble_custom_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_custom_service_t * p_service = (ble_custom_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_custom_service_init(ble_custom_service_t *              p_service,
                                 ble_custom_service_config_t const * p_config)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure.
    p_service->rgb_write_handler = p_config->rgb_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_service->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add battery characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = CUSTOM_SERVICE_UUID_BATT_CHAR;
    add_char_params.uuid_type         = p_service->uuid_type;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.max_len           = sizeof(uint8_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_service->service_handle,
                                  &add_char_params,
                                  &p_service->batt_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add RGB LED characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = CUSTOM_SERVICE_UUID_RGB_CHAR;
    add_char_params.uuid_type        = p_service->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 3; // Single byte per LED
    add_char_params.max_len          = sizeof(uint8_t) * 3;
    uint8_t initial_value[3]         = {0};
    add_char_params.p_init_value     = initial_value;
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    return characteristic_add(p_service->service_handle,
                              &add_char_params,
                              &p_service->rgb_char_handles);
}

uint32_t ble_custom_service_on_battery_change(uint16_t               conn_handle,
                                              ble_custom_service_t * p_service,
                                              uint8_t                batt_value)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(batt_value);

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_service->batt_char_handles.value_handle;
    params.p_data = &batt_value;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}
