#include "ws2812_i2s.h"
#include <nrfx_i2s.h>

#ifndef WS2812_LED_COUNT
#error "Symbol WS2812_LED_COUNT is not defined, so driver cannot determine buffer size."
#endif

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
    if (status == NRFX_ERROR_INVALID_STATE)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    else if (status != NRFX_SUCCESS)
    {
        return NRFX_ERROR_INTERNAL;
    }

    nrfx_i2s_buffers_t const i2s_buffers =
    {
        .p_rx_buffer = NULL,
        .p_tx_buffer = m_i2s_tx_buffer,
    };

    status = nrfx_i2s_start(&i2s_buffers, I2S_BUFFER_SIZE, 0);
    if (status != NRFX_SUCCESS)
    {
        return NRFX_ERROR_INTERNAL;
    }

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

        // I2S sends the upper 2-bytes first.
        // To keep the correct data order, halfwords inside 4-byte word needs to be swapped.
        data_word = (data_word >> 16) | (data_word << 16);

        m_i2s_tx_buffer[I2S_DATA_WORDS_FOR_RESET + idx] = data_word;
    }
}
