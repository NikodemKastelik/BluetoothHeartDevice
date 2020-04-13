#ifndef WS2812_I2S_H__
#define WS2812_I2S_H__

#include <nrfx.h>
#include <stdio.h> 
#include <stdlib.h> 
#include "app_timer.h"
#include "nrf_log.h"

nrfx_err_t ws2812_i2s_init(uint8_t ws2812_pin, uint8_t sck_pin, uint8_t lrck_pin);


void ws2812_i2s_leds_set(uint8_t const * p_led_rgb_values);


/* Functionality for random color refresh */
#define LED_BRIGHTNESS_COEFFICENT 0.000784 // 0.000784 is 250mA peak current consumption

void ws2812_random_refresh(void); //This function should be triggered 50 times per second

enum drive_type{All_Leds, Independent_leds, Snake_effect };

struct ws2812_rand_parameters 
{
  uint16_t delay;               //Delay between color change in seconds
  uint8_t brightness;           //Brightness for leds, range [0-255]
  enum drive_type m_drive_type; //Drive type
};



#endif // WS2812_I2S_H__
