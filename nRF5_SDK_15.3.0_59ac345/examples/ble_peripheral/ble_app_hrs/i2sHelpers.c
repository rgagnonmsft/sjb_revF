
/** @file
 * @defgroup i2s_example_main main.c
 * @{
 * @ingroup i2s_example
 *
 * @brief I2S Example Application main file.
 *
 * This file contains the source code for a sample application using I2S.
 */

#include <stdio.h>
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "i2sHelpers.h"


extern uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
extern uint32_t m_buffer_tx[2][I2S_DATA_BLOCK_WORDS];
extern uint8_t volatile m_blocks_transferred;
extern uint8_t          m_zero_samples_to_ignore;
extern uint16_t         m_sample_value_to_send;
extern uint16_t         m_sample_value_expected;
extern bool             m_error_encountered;
extern uint32_t       * volatile mp_block_to_fill;
extern uint32_t const * volatile mp_block_to_check;
extern LEDDRIVERPIXEL *m_pPixels;
extern uint32_t *m_pTXBuffer;
extern uint8_t	m_brightness;
extern uint16_t m_i2sBufferSize;
extern uint16_t m_numPixels;

static void prepare_tx_data(uint32_t * p_block)
{
    // These variables will be both zero only at the very beginning of each
    // transfer, so we use them as the indication that the re-initialization
    // should be performed.
    if (m_blocks_transferred == 0 && m_zero_samples_to_ignore == 0)
    {
        // Number of initial samples (actually pairs of L/R samples) with zero
        // values that should be ignored - see the comment in 'check_samples'.
        m_zero_samples_to_ignore = 2;
        m_sample_value_to_send   = 0xCAFE;
        m_sample_value_expected  = 0xCAFE;
        m_error_encountered      = false;
    }

    // [each data word contains two 16-bit samples]
    uint16_t i;
    for (i = 0; i < I2S_DATA_BLOCK_WORDS; ++i)
    {
        uint16_t sample_l = m_sample_value_to_send - 1;
        uint16_t sample_r = m_sample_value_to_send + 1;
        ++m_sample_value_to_send;

        uint32_t * p_word = &p_block[i];
        ((uint16_t *)p_word)[0] = sample_l;
        ((uint16_t *)p_word)[1] = sample_r;
    }
}


static bool check_samples(uint32_t const * p_block)
{
    // [each data word contains two 16-bit samples]
    uint16_t i;
    for (i = 0; i < I2S_DATA_BLOCK_WORDS; ++i)
    {
        uint32_t const * p_word = &p_block[i];
        uint16_t actual_sample_l = ((uint16_t const *)p_word)[0];
        uint16_t actual_sample_r = ((uint16_t const *)p_word)[1];

        // Normally a couple of initial samples sent by the I2S peripheral
        // will have zero values, because it starts to output the clock
        // before the actual data is fetched by EasyDMA. As we are dealing
        // with streaming the initial zero samples can be simply ignored.
        if (m_zero_samples_to_ignore > 0 &&
            actual_sample_l == 0 &&
            actual_sample_r == 0)
        {
            --m_zero_samples_to_ignore;
        }
        else
        {
            m_zero_samples_to_ignore = 0;

            uint16_t expected_sample_l = m_sample_value_expected - 1;
            uint16_t expected_sample_r = m_sample_value_expected + 1;
            ++m_sample_value_expected;

            if (actual_sample_l != expected_sample_l ||
                actual_sample_r != expected_sample_r)
            {
                NRF_LOG_INFO("%3u: %04x/%04x, expected: %04x/%04x (i: %u)",
                    m_blocks_transferred, actual_sample_l, actual_sample_r,
                    expected_sample_l, expected_sample_r, i);
                return false;
            }
        }
    }
    NRF_LOG_INFO("%3u: OK", m_blocks_transferred);
    return true;
}


static void check_rx_data(uint32_t const * p_block)
{
    ++m_blocks_transferred;

    if (!m_error_encountered)
    {
        m_error_encountered = !check_samples(p_block);
    }

    if (m_error_encountered)
    {
        bsp_board_led_off(LED_OK);
        bsp_board_led_invert(LED_ERROR);
    }
    else
    {
        bsp_board_led_off(LED_ERROR);
        bsp_board_led_invert(LED_OK);
    }
}


static void i2s_data_handler(nrf_drv_i2s_buffers_t const * p_released,
                         uint32_t                      status)
{
    // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
    // each time next buffers are requested, so data corruption is not
    // expected.
    ASSERT(p_released);

    // When the handler is called after the transfer has been stopped
    // (no next buffers are needed, only the used buffers are to be
    // released), there is nothing to do.
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    // First call of this handler occurs right after the transfer is started.
    // No data has been transferred yet at this point, so there is nothing to
    // check. Only the buffers for the next part of the transfer should be
    // provided.
    if (!p_released->p_rx_buffer)
    {
        nrf_drv_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = m_buffer_rx[1],
            .p_tx_buffer = m_buffer_tx[1],
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));

        mp_block_to_fill = m_buffer_tx[1];
    }
    else
    {
        mp_block_to_check = p_released->p_rx_buffer;
        // The driver has just finished accessing the buffers pointed by
        // 'p_released'. They can be used for the next part of the transfer
        // that will be scheduled now.
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));

        // The pointer needs to be typecasted here, so that it is possible to
        // modify the content it is pointing to (it is marked in the structure
        // as pointing to constant data because the driver is not supposed to
        // modify the provided data).
        mp_block_to_fill = (uint32_t *)p_released->p_tx_buffer;
    }
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    bsp_board_leds_on();
    app_error_save_and_stop(id, pc, info);
}

void i2s_init(void) 
{
    uint32_t err_code = NRF_SUCCESS;
    NRF_LOG_INFO("I2S loopback example started.");

    m_pTXBuffer = (uint32_t *) malloc(sizeof(uint32_t) * m_i2sBufferSize);	

    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;

    m_pPixels->r = 0xFF; //test pink led
    m_pPixels->g = 0xC0;
    m_pPixels->b = 0xCB;

    // In Master mode the MCK frequency and the MCK/LRCK ratio should be
    // set properly in order to achieve desired audio sample rate (which
    // is equivalent to the LRCK frequency).
    // For the following settings we'll get the LRCK frequency equal to
    // 15873 Hz (the closest one to 16 kHz that is possible to achieve).
    config.sdin_pin  = I2S_SDIN_PIN;
    config.sdout_pin = I2S_SDOUT_PIN;
    config.mck_setup = NRF_I2S_MCK_32MDIV21;
    config.ratio     = NRF_I2S_RATIO_96X;
    config.channels  = NRF_I2S_CHANNELS_STEREO;
    err_code = nrf_drv_i2s_init(&config, i2s_data_handler);
    APP_ERROR_CHECK(err_code);
}

void i2s_transfer(void) 
{
      uint32_t err_code = NRF_SUCCESS;
       m_blocks_transferred = 0;
        mp_block_to_fill  = NULL;
        mp_block_to_check = NULL;

        prepare_tx_data(m_buffer_tx[0]);

        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_buffer_tx[0],
            .p_rx_buffer = m_buffer_rx[0],
        };
        err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
        APP_ERROR_CHECK(err_code);

        do {
            // Wait for an event.
            __WFE();
            // Clear the event register.
            __SEV();
            __WFE();

            if (mp_block_to_fill)
            {
                prepare_tx_data(mp_block_to_fill);
                mp_block_to_fill = NULL;
            }
            if (mp_block_to_check)
            {
                check_rx_data(mp_block_to_check);
                mp_block_to_check = NULL;
            }
        } while (m_blocks_transferred < BLOCKS_TO_TRANSFER);

        nrf_drv_i2s_stop();

        NRF_LOG_FLUSH();

        bsp_board_leds_off();
        nrf_delay_ms(PAUSE_TIME);
}
/*
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    bsp_board_init(BSP_INIT_LEDS);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("I2S loopback example started.");

    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
    // In Master mode the MCK frequency and the MCK/LRCK ratio should be
    // set properly in order to achieve desired audio sample rate (which
    // is equivalent to the LRCK frequency).
    // For the following settings we'll get the LRCK frequency equal to
    // 15873 Hz (the closest one to 16 kHz that is possible to achieve).
    config.sdin_pin  = I2S_SDIN_PIN;
    config.sdout_pin = I2S_SDOUT_PIN;
    config.mck_setup = NRF_I2S_MCK_32MDIV21;
    config.ratio     = NRF_I2S_RATIO_96X;
    config.channels  = NRF_I2S_CHANNELS_STEREO;
    err_code = nrf_drv_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
        m_blocks_transferred = 0;
        mp_block_to_fill  = NULL;
        mp_block_to_check = NULL;

        prepare_tx_data(m_buffer_tx[0]);

        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_buffer_tx[0],
            .p_rx_buffer = m_buffer_rx[0],
        };
        err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
        APP_ERROR_CHECK(err_code);

        do {
            // Wait for an event.
            __WFE();
            // Clear the event register.
            __SEV();
            __WFE();

            if (mp_block_to_fill)
            {
                prepare_tx_data(mp_block_to_fill);
                mp_block_to_fill = NULL;
            }
            if (mp_block_to_check)
            {
                check_rx_data(mp_block_to_check);
                mp_block_to_check = NULL;
            }
        } while (m_blocks_transferred < BLOCKS_TO_TRANSFER);

        nrf_drv_i2s_stop();

        NRF_LOG_FLUSH();

        bsp_board_leds_off();
        nrf_delay_ms(PAUSE_TIME);
    }
}
*/
/** @} */


void FillI2SDriverBuffer(void)
{
	LEDDRIVERPIXEL*pPixel = m_pPixels;
	int8_t offset = 1;
	uint8_t *pTX = (uint8_t *) m_pTXBuffer;
		
	for (uint16_t iPixel = 0; iPixel < m_numPixels; iPixel++)
	{
		uint32_t rgb = (((pPixel->g * m_brightness) >> 8) << 16) | (((pPixel->r * m_brightness) >> 8) << 8) | ((pPixel->b * m_brightness) >> 8);
		for (uint8_t i_rgb = 0; i_rgb < I2S_WS2812B_DRIVE_BUF_SIZE_PER_LED; i_rgb++)
		{
			switch (rgb & 0x00c00000)
			{
			case (0x00400000):
				*(pTX + offset)  = (uint8_t)((I2S_WS2812B_DRIVE_PATTERN_0 << 4) | I2S_WS2812B_DRIVE_PATTERN_1);
				break;
			case (0x00800000):
				*(pTX + offset)  = (uint8_t)((I2S_WS2812B_DRIVE_PATTERN_1 << 4) | I2S_WS2812B_DRIVE_PATTERN_0);
				break;
			case (0x00c00000):
				*(pTX + offset)  = (uint8_t)((I2S_WS2812B_DRIVE_PATTERN_1 << 4) | I2S_WS2812B_DRIVE_PATTERN_1);
				break;
			default:
				*(pTX + offset)  = (uint8_t)((I2S_WS2812B_DRIVE_PATTERN_0 << 4) | I2S_WS2812B_DRIVE_PATTERN_0);
				break;
			}
			pTX++;
			offset = -offset;
			rgb <<= (24 / I2S_WS2812B_DRIVE_BUF_SIZE_PER_LED);
		}
		pPixel++;
	}
}

void neopixelWrite() 
{
  m_i2sBufferSize = 3 * m_numPixels + RESET_BITS;

  FillI2SDriverBuffer();
  
  nrfx_i2s_buffers_t buffers;
  buffers.p_rx_buffer = NULL;
  buffers.p_tx_buffer = (uint32_t *) m_pTXBuffer;	

  nrf_drv_i2s_start(&buffers, m_i2sBufferSize, 0);

}