/*
 * debug_output.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "usart.h"
#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "data_logger.h"

#include "configs.h"
#include "debug_output.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define DATA_LOGGER_SERIAL_INSTANCE 						USART3
#define DATA_LOGGER_DMA_INSTANCE 							DMA1
#define DATA_LOGGER_DMA_STREAM 								LL_DMA_STREAM_3
#define DATA_LOGGER_BUFF_SIZE 								(512UL)

#define MIN_TRANSMIT_BURST_SIZE								(256UL)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	DATA_LOGGER_BANK_0 = 0,
	DATA_LOGGER_BANK_1,
	DATA_LOGGER_BANK_SIZE
} eSjcDataLoggerOutputBank_t;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static char xDataLoggerBuff0[DATA_LOGGER_BUFF_SIZE];
static char xDataLoggerBuff1[DATA_LOGGER_BUFF_SIZE];

static uint32_t uDataBuffSize = 0;

static eSjcDataLoggerOutputBank_t xDataLoggerBank = DATA_LOGGER_BANK_0;

static bool bSerialBusy = false;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
//static int serial_write(char *ptr, int len);
static void TransmitBuffer(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_DataLogger_Init(void)
{
	  LL_DMA_ConfigAddresses(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM, (uint32_t)&xDataLoggerBuff0, LL_USART_DMA_GetRegAddr(DATA_LOGGER_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	  LL_DMA_EnableIT_TC(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM);

	  LL_USART_EnableDMAReq_TX(DATA_LOGGER_SERIAL_INSTANCE);

	  bSerialBusy = false;
}

int SJC_DataLogger_Transmit(const char *format, ...)
{
	va_list apx;
	int n = 0;
	va_start(apx, format);

	uint32_t initial_index = (uDataBuffSize > 0) ? uDataBuffSize+1 : 0;

	switch (xDataLoggerBank) {
		case DATA_LOGGER_BANK_1:
			n = vsnprintf(&xDataLoggerBuff1[initial_index], DATA_LOGGER_BUFF_SIZE, format, apx);
			break;
		default:
			n = vsnprintf(&xDataLoggerBuff0[initial_index], DATA_LOGGER_BUFF_SIZE, format, apx);
			break;
	}

	va_end(apx);

	if (n > 0)
	{
		uDataBuffSize += n;

		if ((uDataBuffSize > MIN_TRANSMIT_BURST_SIZE) && (false == bSerialBusy))
		{
			TransmitBuffer();
		}
	}

	return n;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

static void TransmitBuffer(void)
{
	LL_DMA_DisableStream(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM);

	switch (xDataLoggerBank) {
	case DATA_LOGGER_BANK_1:
		LL_DMA_ConfigAddresses(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM, (uint32_t)&xDataLoggerBuff1, LL_USART_DMA_GetRegAddr(DATA_LOGGER_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		xDataLoggerBank = DATA_LOGGER_BANK_0;

		break;
	default:
		LL_DMA_ConfigAddresses(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM, (uint32_t)&xDataLoggerBuff0, LL_USART_DMA_GetRegAddr(DATA_LOGGER_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		xDataLoggerBank = DATA_LOGGER_BANK_1;

		break;
	}

	bSerialBusy = true;

	LL_DMA_SetDataLength(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM, uDataBuffSize);
	uDataBuffSize = 0;

	LL_DMA_ClearFlag_TE3(DATA_LOGGER_DMA_INSTANCE);
	LL_DMA_ClearFlag_HT3(DATA_LOGGER_DMA_INSTANCE);
	LL_DMA_ClearFlag_TC3(DATA_LOGGER_DMA_INSTANCE);
	LL_DMA_ClearFlag_FE3(DATA_LOGGER_DMA_INSTANCE);

	LL_DMA_EnableStream(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM);
}

void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	if( LL_DMA_IsActiveFlag_TC3(DATA_LOGGER_DMA_INSTANCE) == 1){
		LL_DMA_ClearFlag_TC3(DATA_LOGGER_DMA_INSTANCE);
		bSerialBusy = false;
	}
  /* USER CODE END DMA1_Stream3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}
// ------------------------------------------------------------------- //
