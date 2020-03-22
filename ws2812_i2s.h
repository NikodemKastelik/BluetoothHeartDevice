#ifndef WS2812_I2S_H__
#define WS2812_I2S_H__

#include <nrfx.h>

nrfx_err_t ws2812_i2s_init(uint8_t ws2812_pin, uint8_t sck_pin, uint8_t lrck_pin);

void ws2812_i2s_leds_set(uint8_t const * p_led_rgb_values);

#endif // WS2812_I2S_H__
