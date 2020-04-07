#ifndef WS2812_I2S_H__
#define WS2812_I2S_H__

#include <nrfx.h>

/**
 * @brief Function for initializing the WS2812 I2S driver.
 *
 * @warning SCK and LRCK pins are not needed for the WS2812 to operate,
 *          but are critical for the I2S peripheral to work correctly.
 *          Route them to unconnected pins.
 *
 * @param[in] ws2812_pin Data pin for WS2812 LED array.
 * @param[in] sck_pin    I2S SCK pin.
 * @param[in] lrck_pin   I2S LRCK pin.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE The I2S driver was already initialized outside of the WS2812 driver.
 * @retval NRFX_ERROR_INTERNAL      The I2S driver configuration was wrong.
 */
nrfx_err_t ws2812_i2s_init(uint8_t ws2812_pin, uint8_t sck_pin, uint8_t lrck_pin);

/**
 * @brief Function for setting the new RGB values for the WS2812 LED array.
 *
 * @param[in] p_led_rgb_values Pointer to the array containing new RGB LED values.
 *                             The array size must be equal to @c WS2812_LED_COUNT multiplied by 3.
 */
void ws2812_i2s_leds_set(uint8_t const * p_led_rgb_values);

#endif // WS2812_I2S_H__
