//#include <stdio.h>
//#include "nrf_drv_i2s.h"
//#include "nrf_delay.h"
//#include "app_util_platform.h"
//#include "app_error.h"
//#include "boards.h"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"




static void prepare_tx_data(uint32_t * p_block);

static bool check_samples(uint32_t const * p_block);

static void check_rx_data(uint32_t const * p_block);

static void i2s_data_handler(nrf_drv_i2s_buffers_t const * p_released,
                         uint32_t                      status);

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);

void i2s_init(void);

void i2s_transfer(void);

void FillI2SDriverBuffer(void);

void neopixelWrite(void);


#define LED_OK      BSP_BOARD_LED_0
#define LED_ERROR   BSP_BOARD_LED_1

#define I2S_DATA_BLOCK_WORDS    512
static uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
static uint32_t m_buffer_tx[2][I2S_DATA_BLOCK_WORDS];

// Delay time between consecutive I2S transfers performed in the main loop
// (in milliseconds).
#define PAUSE_TIME          500
// Number of blocks of data to be contained in each transfer.
#define BLOCKS_TO_TRANSFER  20

static uint8_t volatile m_blocks_transferred     = 0;
static uint8_t          m_zero_samples_to_ignore = 0;
static uint16_t         m_sample_value_to_send;
static uint16_t         m_sample_value_expected;
static bool             m_error_encountered;

static uint32_t       * volatile mp_block_to_fill  = NULL;
static uint32_t const * volatile mp_block_to_check = NULL;

#define I2S_SDIN_PIN 11 //not necessary for smart jersey- using i2s out only
#define I2S_SDOUT_PIN BADGE_DATA_3P3 //for smart jersey

//Neopixel specific:
#define RESET_BITS 6
#define I2S_WS2812B_DRIVE_PATTERN_0			((uint8_t)0x08)			// Bit pattern for data "0" is "HLLL".
#define I2S_WS2812B_DRIVE_PATTERN_1			((uint8_t)0x0e)      // Bit pattern for data "1" is "HHHL".
#define	I2S_WS2812B_DRIVE_BUF_SIZE_PER_LED	(12)	// buffer size for each LED (8bit * 4 * 3 )

typedef struct 
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} LEDDRIVERPIXEL;

LEDDRIVERPIXEL	*m_pPixels;
uint32_t *m_pTXBuffer;
uint16_t m_numPixels = 3;
uint8_t	m_brightness;
uint16_t m_i2sBufferSize;