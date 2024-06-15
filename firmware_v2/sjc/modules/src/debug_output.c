/*
 * debug_output.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "usart.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "debug_output.h"

#include "network.h"
#include "network_configs.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define DEBUG_OUTPUT_SERIAL_INSTANCE 	USART3
#define DEBUG_OUTPUT_DMA_INSTANCE 		DMA1
#define DEBUG_OUTPUT_DMA_STREAM 		LL_DMA_STREAM_3
#define DEBUG_OUTPUT_BUFF_SIZE 			(1572UL)
#define DEBUG_OUTPUT_MID_BANK_INDEX 	(DEBUG_OUTPUT_BUFF_SIZE/2)
#define MIN_TRANSMIT_BURST_SIZE			(DEBUG_OUTPUT_MID_BANK_INDEX/2)

#define SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS ((uint32_t)10U)
#define DEBUG_OUTPUT_SERIAL_INSTANCE	USART3
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	DEBUG_OUTPUT_BANK_0 = 0,
	DEBUG_OUTPUT_BANK_1,
	DEBUG_OUTPUT_BANK_SIZE
} eSjcDebugOutputOutputBank_t;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
osSemaphoreId_t xSystemDebugSemprHandle = NULL;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
		.name = "SystemDebugSemaphore",
		.cb_mem = &SerialDebugSemaphoreControlBlock,
		.cb_size = sizeof(SerialDebugSemaphoreControlBlock),
};

static char xDebugOutputBuff[DEBUG_OUTPUT_BUFF_SIZE];

static uint32_t uDataBuffSize = 0;

static eSjcDebugOutputOutputBank_t xDebugOutputBank = DEBUG_OUTPUT_BANK_0;

volatile bool bDebugOutputSerialBusy = false;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static void TransmitDebugBuffer(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void DebugOutput_init(void)
{
	LL_DMA_ConfigAddresses(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uint32_t)&xDebugOutputBuff, LL_USART_DMA_GetRegAddr(DEBUG_OUTPUT_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_EnableIT_TC(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);

	LL_USART_EnableDMAReq_TX(DEBUG_OUTPUT_SERIAL_INSTANCE);

	bDebugOutputSerialBusy = false;

	xSystemDebugSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SerialDebugSemaphore_attributes);

	if (xSystemDebugSemprHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Serial debug semaphore not created...");
		SJC_SYSTEM_RESET();
	}
	else
	{
		osSemaphoreRelease(xSystemDebugSemprHandle);
	}
}

void DebugOutput_presentSystemInfo(void)
{
	uint32_t xBaudRate;
	SJC_Network_GetBaudRate(&xBaudRate);

	DebugOutput_systemPrintf("\f\r\n"
			"************************************************************"
			"\r\n                      Crane Control"
			"\r\n               Single Joint Control Board"
			"\r\n                         (SJC)"
			"\r\n************************************************************"
			"\r\n\n Information: "
			"\r\n - HW Version: %u.%u"
			"\r\n - FW Version: %u.%u.%u (%u/%lu)"
			"\r\n - O.S. Tick Rate: %lu kHz"
			"\r\n - Network address: %lu"
			"\r\n - Network baudrate: %lu bps"
			"\r\n************************************************************\n\n",
			SJC_HW_VERSION_MAJOR,
			SJC_HW_VERSION_MINOR,
			SJC_FW_VERSION_MAJOR,
			SJC_FW_VERSION_MINOR,
			SJC_FW_VERSION_PATCH,
			SJC_FW_RELEASE_MONTH,
			SJC_FW_RELEASE_YEAR,
			(configTICK_RATE_HZ / 1000),
			NETWORK_BASE_ADRESS,
			xBaudRate
			);
}

int DebugOutput_systemPrintf(const char *format, ...)
{
	configASSERT(xSystemDebugSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xSystemDebugSemprHandle, SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS))
	{
		va_list ap;
		int n = 0;
		va_start(ap, format);

		uint32_t initial_index = (uDataBuffSize > 0) ? uDataBuffSize : 0;
		initial_index += (DEBUG_OUTPUT_BANK_1 == xDebugOutputBank) ? DEBUG_OUTPUT_MID_BANK_INDEX : 0;

		n = vsnprintf(&xDebugOutputBuff[initial_index], DEBUG_OUTPUT_BUFF_SIZE, format, ap);

		va_end(ap);

		if (n > 0)
		{
			uDataBuffSize += n;

			if ((uDataBuffSize > MIN_TRANSMIT_BURST_SIZE) && (false == bDebugOutputSerialBusy))
			{
				TransmitDebugBuffer();
			}
		}

		osSemaphoreRelease(xSystemDebugSemprHandle);

		return n;
	}
	else
	{
		return -1;
	}
}

int DebugOutput_loggerPrintf(const char *fmt, ...)
{
	configASSERT(xSystemDebugSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xSystemDebugSemprHandle, osWaitForever))
	{
		va_list apx;
		int n = 0;
		va_start(apx, fmt);

		static char lbuff[100];

		n = vsnprintf(lbuff, sizeof(lbuff), fmt, apx);

		va_end(apx);

		if (n > 0)
		{
			for (int i = 0; i < n; i++)
			{
				while(!LL_USART_IsActiveFlag_TXE(DEBUG_OUTPUT_SERIAL_INSTANCE));
				LL_USART_TransmitData8(DEBUG_OUTPUT_SERIAL_INSTANCE, lbuff[i]);
			}
		}

		osSemaphoreRelease(xSystemDebugSemprHandle);

		return n;
	}
	else
	{
		return -1;
	}
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
static void TransmitDebugBuffer(void)
{
	for (int i = 0; i < uDataBuffSize; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(DEBUG_OUTPUT_SERIAL_INSTANCE));
		LL_USART_TransmitData8(DEBUG_OUTPUT_SERIAL_INSTANCE, xDebugOutputBuff[i]);
	}
	uDataBuffSize = 0;

	//	LL_DMA_DisableStream(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);
	//
	//	switch (xDebugOutputBank) {
	//	case DEBUG_OUTPUT_BANK_1:
	//		LL_DMA_ConfigAddresses(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uint32_t)&xDebugOutputBuff[DEBUG_OUTPUT_MID_BANK_INDEX], LL_USART_DMA_GetRegAddr(DEBUG_OUTPUT_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	//		xDebugOutputBank = DEBUG_OUTPUT_BANK_0;
	//
	//		break;
	//	default:
	//		LL_DMA_ConfigAddresses(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uint32_t)&xDebugOutputBuff[0], LL_USART_DMA_GetRegAddr(DEBUG_OUTPUT_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	//		xDebugOutputBank = DEBUG_OUTPUT_BANK_1;
	//
	//		break;
	//	}
	//
	//	bDebugOutputSerialBusy = true;
	//
	//	LL_DMA_SetDataLength(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uDataBuffSize));
	//	uDataBuffSize = 0;
	//
	//	LL_DMA_ClearFlag_TE3(DEBUG_OUTPUT_DMA_INSTANCE);
	//	LL_DMA_ClearFlag_HT3(DEBUG_OUTPUT_DMA_INSTANCE);
	//	LL_DMA_ClearFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE);
	//	LL_DMA_ClearFlag_FE3(DEBUG_OUTPUT_DMA_INSTANCE);
	//
	//	LL_DMA_EnableStream(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);
}

void DMA1_Stream3_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	if( LL_DMA_IsActiveFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE) == 1){
		LL_DMA_ClearFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE);
		bDebugOutputSerialBusy = false;
	}
	/* USER CODE END DMA1_Stream3_IRQn 0 */

	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

int __io_putchar(int ch)
{
	DebugOutput_systemPrintf("%c", ch);
	return (ch);
}

//int _write(int fd, char *ptr, int len) {
//	if (xSystemDebugSemprHandle != NULL)
//	{
//#if SJC_HW_VERSION_MAJOR > 1
//		for (int i = 0; i < len; i++)
//		{
//			while (!LL_USART_IsActiveFlag_TXE(USART3));
//			LL_USART_TransmitData8(USART3, ptr[i]);
//		}
//#else
////		CDC_Transmit_FS((uint8_t *)(&ptr[0]), len);
//#endif
//		return len;
//	}
//	else
//	{
//		return 0;
//	}
//}
// ------------------------------------------------------------------- //
