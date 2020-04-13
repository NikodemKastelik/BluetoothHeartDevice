#include "ws2812_i2s.h"
#include <nrfx_i2s.h>

#ifndef WS2812_LED_COUNT
#error "Symbol WS2812_LED_COUNT is not defined, so driver cannot determine buffer size."
#endif


APP_TIMER_DEF(led_rand_timer);     /**< Handler for repeated timer used to blink LED 1. */

#if defined(WS2812_LOGIC_INVERT)
    #define WS2812_BIT_0 0x7uL // 0b0111
    #define WS2812_BIT_1 0x1uL // 0b0001
    #define WS2812_DATA_WORD_RESET_PATTERN ~0uL
#else
    #define WS2812_BIT_0 0x8uL // 0b1000
    #define WS2812_BIT_1 0xEuL // 0b1110
    #define WS2812_DATA_WORD_RESET_PATTERN 0uL
#endif

/**
 * @brief Symbol specifying number of I2S bits needed to encode
 *        single bit of RGB value for a single LED.
 *
 * I2S master clock frequency for WS2812 is 3.2 MHz.
 * Therefore single bit of I2S tranmission takes:
 * 1 / 3.2 MHz = 0.3125 us
 * Single bit of WS2812 frame takes 1.2 us.
 * Therefore four bits of I2S transmission form a single bit of WS2812 frame.
 */
#define I2S_BITS_PER_RGB_BIT 4

#define BITS_PER_SINGLE_COLOR 8

#define COLORS_PER_SINGLE_RGB_LED 3

#define I2S_BITS_PER_RGB_LED (I2S_BITS_PER_RGB_BIT  * \
                              BITS_PER_SINGLE_COLOR * \
                              COLORS_PER_SINGLE_RGB_LED)

#define I2S_DATA_WORDS_PER_RGB_LED (I2S_BITS_PER_RGB_LED / 32)

/**
 * @brief Symbol specifying number of I2S bits needed to reset
 *        single WS2812 LED.
 *
 * WS2812 datasheet specifies that a low pulse of minimum 280 us width
 * is needed to reset LED configuration:
 * 280 us / 0.3125 us = 896
 */
#define I2S_BITS_FOR_RESET 896

#define I2S_DATA_WORDS_FOR_RESET ((I2S_BITS_FOR_RESET / 32) + 1)

#define I2S_DATA_WORDS_FOR_LEDS (WS2812_LED_COUNT * I2S_DATA_WORDS_PER_RGB_LED)

#define I2S_BUFFER_SIZE (I2S_DATA_WORDS_FOR_RESET + I2S_DATA_WORDS_FOR_LEDS)

static uint32_t m_i2s_tx_buffer[I2S_BUFFER_SIZE];
static struct ws2812_rand_parameters m_ws2812_rand_parameters;


static void led_rand_timer_handler(void * p_context)
{
    ws2812_random_refresh();
}

static void i2s_dummy_data_handler(nrfx_i2s_buffers_t const * p_released, uint32_t status)
{
}

nrfx_err_t ws2812_i2s_init(uint8_t ws2812_pin, uint8_t sck_pin, uint8_t lrck_pin)
{
    // Populate buffer with WS2812 reset pattern.
    for (size_t word_idx = 0; word_idx < I2S_DATA_WORDS_FOR_RESET; word_idx++)
    {
        m_i2s_tx_buffer[word_idx] = WS2812_DATA_WORD_RESET_PATTERN;
    }

    nrfx_i2s_config_t const i2s_config =
    {
        .sck_pin      = sck_pin,
        .lrck_pin     = lrck_pin,
        .mck_pin      = NRFX_I2S_PIN_NOT_USED,
        .sdout_pin    = ws2812_pin,
        .sdin_pin     = NRFX_I2S_PIN_NOT_USED,
        .irq_priority = NRFX_I2S_CONFIG_IRQ_PRIORITY,
        .mode         = NRF_I2S_MODE_MASTER,
        .format       = NRF_I2S_FORMAT_I2S,
        .alignment    = NRF_I2S_ALIGN_LEFT,
        .sample_width = NRF_I2S_SWIDTH_16BIT,
        .channels     = NRF_I2S_CHANNELS_STEREO,
        .mck_setup    = NRF_I2S_MCK_32MDIV10,
        .ratio        = NRF_I2S_RATIO_32X
    };

    nrfx_err_t status;
    status = nrfx_i2s_init(&i2s_config, i2s_dummy_data_handler);
    if (status != NRFX_SUCCESS)
    {
        return status;
    }

    nrfx_i2s_buffers_t const i2s_buffers =
    {
        .p_rx_buffer = NULL,
        .p_tx_buffer = m_i2s_tx_buffer,
    };

    status = nrfx_i2s_start(&i2s_buffers, I2S_BUFFER_SIZE, 0);
    if (status != NRFX_SUCCESS)
    {
        return status;
    }

    //Timer for leds refresh/random
    ret_code_t err_code;
    err_code = app_timer_create(&led_rand_timer,
                                APP_TIMER_MODE_REPEATED,
                                led_rand_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(led_rand_timer, APP_TIMER_TICKS(20), NULL); //50Hz handler
    APP_ERROR_CHECK(err_code);

    //TODO default parameters
    m_ws2812_rand_parameters.delay =4;
    m_ws2812_rand_parameters.brightness =255;
    m_ws2812_rand_parameters.m_drive_type=All_Leds;
    return NRFX_SUCCESS;
}

void ws2812_i2s_leds_set(uint8_t const * p_led_rgb_values)
{
    for (size_t idx = 0; idx < I2S_DATA_WORDS_FOR_LEDS; idx++)
    {
        uint8_t const led_color = p_led_rgb_values[idx];
        uint32_t data_word = 0;
        for (uint8_t bit_idx = 0; bit_idx < BITS_PER_SINGLE_COLOR; bit_idx++)
        {
            data_word |= ((led_color & (1 << bit_idx)) ? WS2812_BIT_1 : WS2812_BIT_0)
                         << (bit_idx * I2S_BITS_PER_RGB_BIT);
        }

        // Swap bytes in 4-byte word
        data_word = (data_word >> 16) | (data_word << 16);

        m_i2s_tx_buffer[I2S_DATA_WORDS_FOR_RESET + idx] = data_word;
    }
}

void ws2812_random_refresh(void)
{
  static uint32_t time_counter;
  static uint8_t leds_value[WS2812_LED_COUNT * 3];
  time_counter++;
  static uint8_t old_red_value;
  static uint8_t old_green_value;
  static uint8_t old_blue_value;

  static uint8_t new_red_value;
  static uint8_t new_green_value;
  static uint8_t new_blue_value;

  float curr_red_value=0;
  float curr_green_value=0;
  float curr_blue_value=0;

  if(time_counter >= m_ws2812_rand_parameters.delay*50) //Refresh leds data !!
  {
    time_counter=0;
    unsigned rand_value = rand()*2;
    unsigned r = (rand_value & 0xF800) >> 11;
    unsigned g = (rand_value & 0x07E0) >> 5;
    unsigned b = rand_value & 0x001F;


    old_red_value = new_red_value;
    old_green_value = new_green_value;
    old_blue_value = new_blue_value;

    new_red_value = (r << 3) | (r >> 2);
    new_green_value = (g << 2) | (g >> 4);
    new_blue_value = (b << 3) | (b >> 2);

  }

  switch(m_ws2812_rand_parameters.m_drive_type)
  {
    case All_Leds:
    {
          if(new_red_value != old_red_value)
          {
              if(old_red_value < new_red_value)
              {
                curr_red_value = (float)old_red_value + (( new_red_value - old_red_value )/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
              if(old_red_value > new_red_value)
              {
                curr_red_value = (float)old_red_value - ((old_red_value - new_red_value)/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
          }else
          {
                curr_red_value = new_red_value;
          }


          if(new_green_value != old_green_value)
          {
              if(old_green_value < new_green_value)
              {
                curr_green_value = (float)old_green_value + (( new_green_value - old_green_value )/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
              if(old_green_value > new_green_value)
              {
                curr_green_value = (float)old_green_value - ((old_green_value - new_green_value)/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
          }else
          {
                curr_green_value = new_green_value;
          }


          if(new_blue_value != old_blue_value)
          {
              if(old_blue_value < new_blue_value)
              {
                curr_blue_value = (float)old_blue_value + (( new_blue_value - old_blue_value )/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
              if(old_blue_value > new_blue_value)
              {
                curr_blue_value = (float)old_blue_value - ((old_blue_value - new_blue_value)/(m_ws2812_rand_parameters.delay*50.0))*time_counter;
              }
          }else
          {
                curr_blue_value = new_blue_value;
          }
  
        for (uint8_t led_idx = 0; led_idx < WS2812_LED_COUNT * 3; )
        {
            leds_value[led_idx] = curr_red_value*LED_BRIGHTNESS_COEFFICENT*m_ws2812_rand_parameters.brightness;\
            led_idx+=3;
        }

        for (uint8_t led_idx = 1; led_idx < WS2812_LED_COUNT * 3; )
        {
            leds_value[led_idx] = curr_green_value*LED_BRIGHTNESS_COEFFICENT*m_ws2812_rand_parameters.brightness;\
            led_idx+=3;
        }

        for (uint8_t led_idx = 2; led_idx < WS2812_LED_COUNT * 3; )
        {
            leds_value[led_idx] = curr_blue_value*LED_BRIGHTNESS_COEFFICENT*m_ws2812_rand_parameters.brightness;\
            led_idx+=3;
        }
    }

    case Independent_leds:
    {
    }

    case Snake_effect:
    {
    }
  }

      ws2812_i2s_leds_set(leds_value);
}
