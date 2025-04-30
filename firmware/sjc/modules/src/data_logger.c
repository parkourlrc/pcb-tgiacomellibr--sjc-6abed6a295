#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "usart.h"

#include "base/include/configs.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"

#define DATA_LOGGER_SERIAL_INSTANCE USART3
#define DATA_LOGGER_DMA_INSTANCE DMA1
#define DATA_LOGGER_DMA_STREAM LL_DMA_STREAM_3
#define DATA_LOGGER_BUFF_SIZE (2048UL)
#define DATA_LOGGER_MID_BANK_INDEX (DATA_LOGGER_BUFF_SIZE / 2)
#define MIN_TRANSMIT_BURST_SIZE (DATA_LOGGER_MID_BANK_INDEX / 2)

typedef enum
{
	DATA_LOGGER_BANK_0 = 0,
	DATA_LOGGER_BANK_1,
	DATA_LOGGER_BANK_SIZE
} eSjcDataLoggerOutputBank_t;

static char xDataLoggerBuff[DATA_LOGGER_BUFF_SIZE];
static char xDataLoggerBuffDbg[512];

static uint32_t uDataBuffSize = 0;
static eSjcDataLoggerOutputBank_t xDataLoggerBank = DATA_LOGGER_BANK_0;

volatile bool bSerialBusy = false;

// Mutex to protect access to UART and DMA
static SemaphoreHandle_t xLoggerMutex = NULL;

// Forward declaration
static void TransmitBuffer(void);

void DataLogger_init(void)
{
	xLoggerMutex = xSemaphoreCreateMutex();

	LL_DMA_ConfigAddresses(
		DATA_LOGGER_DMA_INSTANCE,
		DATA_LOGGER_DMA_STREAM,
		(uint32_t)&xDataLoggerBuff,
		LL_USART_DMA_GetRegAddr(DATA_LOGGER_SERIAL_INSTANCE),
		LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_EnableIT_TC(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM);
	LL_USART_EnableDMAReq_TX(DATA_LOGGER_SERIAL_INSTANCE);

	bSerialBusy = false;
}

int DataLogger_transmit(const char *format, ...)
{
	if (xLoggerMutex != NULL)
	{
		xSemaphoreTake(xLoggerMutex, portMAX_DELAY);
	}

	va_list ap;
	va_start(ap, format);

	// Get bank offset and remaining space
	uint32_t bank_offset = (xDataLoggerBank == DATA_LOGGER_BANK_1) ? DATA_LOGGER_MID_BANK_INDEX : 0;
	uint32_t initial_index = bank_offset + uDataBuffSize;
	size_t bank_remaining = DATA_LOGGER_MID_BANK_INDEX - uDataBuffSize;

	va_list ap_copy;
	va_copy(ap_copy, ap);
	int total_len = vsnprintf(NULL, 0, format, ap_copy);
	va_end(ap_copy);

	if (total_len <= 0)
	{
		va_end(ap);
		if (xLoggerMutex)
		{
			xSemaphoreGive(xLoggerMutex);
		}

		return -1;
	}

	// Case 1: message fits in current bank
	if ((size_t)total_len <= bank_remaining)
	{
		int written = vsnprintf(&xDataLoggerBuff[initial_index], bank_remaining, format, ap);
		uDataBuffSize += written;

		if ((uDataBuffSize > MIN_TRANSMIT_BURST_SIZE) && (!bSerialBusy))
		{
			TransmitBuffer();
		}

		va_end(ap);
		if (xLoggerMutex)
		{
			xSemaphoreGive(xLoggerMutex);
		}

		return written;
	}
	else
	{
		// Case 2: split message across current and next bank
		int written = vsnprintf(&xDataLoggerBuff[initial_index], bank_remaining, format, ap);
		uDataBuffSize += written;

		if (!bSerialBusy)
		{
			TransmitBuffer();
		}

		// Switch to next bank
		xDataLoggerBank = (xDataLoggerBank == DATA_LOGGER_BANK_1) ? DATA_LOGGER_BANK_0 : DATA_LOGGER_BANK_1;
		uDataBuffSize = 0;

		bank_offset = (xDataLoggerBank == DATA_LOGGER_BANK_1) ? DATA_LOGGER_MID_BANK_INDEX : 0;
		initial_index = bank_offset;

		va_start(ap_copy, format);
		written = vsnprintf(&xDataLoggerBuff[initial_index], DATA_LOGGER_MID_BANK_INDEX, format, ap_copy);
		va_end(ap_copy);

		uDataBuffSize += written;
		va_end(ap);

		if (xLoggerMutex)
		{
			xSemaphoreGive(xLoggerMutex);
		}
		return written;
	}
}

int DataLogger_printf(const char *format, ...)
{
	if (xLoggerMutex != NULL)
	{
		xSemaphoreTake(xLoggerMutex, portMAX_DELAY);
	}

	va_list ap;
	int n = 0;
	va_start(ap, format);
	n = vsnprintf(xDataLoggerBuffDbg, sizeof(xDataLoggerBuffDbg) - 1, format, ap);
	va_end(ap);

	if (n > 0)
	{
		for (int i = 0; i < n; i++)
		{
			LL_USART_TransmitData8(DATA_LOGGER_SERIAL_INSTANCE, xDataLoggerBuffDbg[i]);
			while (!LL_USART_IsActiveFlag_TXE(DATA_LOGGER_SERIAL_INSTANCE))
			{
			};
		}
	}

	if (xLoggerMutex)
		xSemaphoreGive(xLoggerMutex);
	return n;
}

static void TransmitBuffer(void)
{
	LL_DMA_DisableStream(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM);

	uint32_t base_addr = (xDataLoggerBank == DATA_LOGGER_BANK_1) ? (uint32_t)&xDataLoggerBuff[DATA_LOGGER_MID_BANK_INDEX] : (uint32_t)&xDataLoggerBuff[0];

	LL_DMA_ConfigAddresses(DATA_LOGGER_DMA_INSTANCE, DATA_LOGGER_DMA_STREAM, base_addr, LL_USART_DMA_GetRegAddr(DATA_LOGGER_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	xDataLoggerBank = (xDataLoggerBank == DATA_LOGGER_BANK_1) ? DATA_LOGGER_BANK_0 : DATA_LOGGER_BANK_1;
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
	if (LL_DMA_IsActiveFlag_TC3(DATA_LOGGER_DMA_INSTANCE))
	{
		LL_DMA_ClearFlag_TC3(DATA_LOGGER_DMA_INSTANCE);
		bSerialBusy = false;
	}

	if (LL_DMA_IsActiveFlag_TE3(DATA_LOGGER_DMA_INSTANCE))
	{
		LL_DMA_ClearFlag_TE3(DATA_LOGGER_DMA_INSTANCE);
		DebugOutput_systemPrintf("[FAULT] DataLogger: DMA transfer error\n");
		bSerialBusy = false;
	}
}
