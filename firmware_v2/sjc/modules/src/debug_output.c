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
#define DEBUG_OUTPUT_BUFF_SIZE 			(2048UL)
#define DEBUG_OUTPUT_MID_BANK_INDEX 	(DEBUG_OUTPUT_BUFF_SIZE/2)
#define MIN_TRANSMIT_BURST_SIZE			(DEBUG_OUTPUT_MID_BANK_INDEX/20)
#define DEBUG_OUTPUT_TEMP_BUFF_SIZE 	(DEBUG_OUTPUT_BUFF_SIZE)

#define SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS (osWaitForever)
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
//osSemaphoreId_t xSystemDebugSemprHandle = NULL;
//StaticSemaphore_t SerialDebugSemaphoreControlBlock;
//const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
//		.name = "SystemDebugSemaphore",
//		.cb_mem = &SerialDebugSemaphoreControlBlock,
//		.cb_size = sizeof(SerialDebugSemaphoreControlBlock),
//};

osMutexId_t xSystemDebugMutexHandle;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
const osMutexAttr_t xSystemDebugMutex_attr = {
		"SystemDebugMutex",                     // human readable mutex name
		osMutexPrioInherit,    					// attr_bits
		&SerialDebugSemaphoreControlBlock, 		// memory for control block
		sizeof(SerialDebugSemaphoreControlBlock)// size for control block
};

static char xDebugOutputBuff[DEBUG_OUTPUT_BUFF_SIZE];
static char xDebugOutputTempBuff[DEBUG_OUTPUT_TEMP_BUFF_SIZE];
volatile uint32_t uDataBuffSize = 0;
volatile bool bSerialBusyFlag = false;

static eSjcDebugOutputOutputBank_t xDebugOutputBank = DEBUG_OUTPUT_BANK_0;

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static void transmitDebugBuffer(void);
static bool isSerialBusy(void);
static void setSerialBusyFlag(void);
static void resetSerialBusyFlag(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void DebugOutput_init(void)
{
	//	xSystemDebugSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SerialDebugSemaphore_attributes);
	xSystemDebugMutexHandle = osMutexNew(&xSystemDebugMutex_attr);

	if (xSystemDebugMutexHandle == NULL)
	{
		SJC_SYSTEM_RESET();
	}
	else
	{
		//		osSemaphoreRelease(xSystemDebugSemprHandle);
		//		osMutexRelease(xSystemDebugMutexHandle);
		DebugOutput_systemPrintf("\r\n[OS] Serial debug mutex created...");
	}

	LL_DMA_EnableIT_TC(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);

	LL_USART_EnableDMAReq_TX(DEBUG_OUTPUT_SERIAL_INSTANCE);

	uDataBuffSize = 0;
}

void DebugOutput_presentSystemInfo(void)
{
	uint32_t xBaudRate;
	Network_getBaudRate(&xBaudRate);
	uint32_t xNetworkAddress;
	Network_getAddress(&xNetworkAddress);

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
			xNetworkAddress,
			xBaudRate
	);
}

int DebugOutput_systemPrintf(const char *format, ...)
{
	configASSERT(xSystemDebugMutexHandle != NULL);

	if (osOK == osMutexAcquire(xSystemDebugMutexHandle, SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS))
	{
		// Mount the data payload
		va_list ap;
		size_t n = 0;
		va_start(ap, format);

		n = vsnprintf(xDebugOutputTempBuff, DEBUG_OUTPUT_TEMP_BUFF_SIZE, format, ap);

		va_end(ap);

		// Flush buffer if needed
		if ((n+uDataBuffSize) >= DEBUG_OUTPUT_BUFF_SIZE)
		{
			transmitDebugBuffer();
		}

		// Add data to buffer
		uint32_t initial_index = uDataBuffSize + ( (DEBUG_OUTPUT_BANK_1 == xDebugOutputBank) ? DEBUG_OUTPUT_MID_BANK_INDEX : 0 );
		memcpy(&xDebugOutputBuff[initial_index], xDebugOutputTempBuff, n);
		uDataBuffSize += n;

		// Transmit data if possible
		if (uDataBuffSize > MIN_TRANSMIT_BURST_SIZE)
		{
			transmitDebugBuffer();
		}


		osMutexRelease(xSystemDebugMutexHandle);
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
static bool isSerialBusy(void)
{
	return (bSerialBusyFlag);
}

static void setSerialBusyFlag(void)
{
	bSerialBusyFlag = true;
}

static void resetSerialBusyFlag(void)
{
	bSerialBusyFlag = false;
}

static void transmitDebugBuffer(void)
{
	while(true == isSerialBusy());

	setSerialBusyFlag();

	switch (xDebugOutputBank) {
	case DEBUG_OUTPUT_BANK_1:
		LL_DMA_ConfigAddresses(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uint32_t)&xDebugOutputBuff[DEBUG_OUTPUT_MID_BANK_INDEX], LL_USART_DMA_GetRegAddr(DEBUG_OUTPUT_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		xDebugOutputBank = DEBUG_OUTPUT_BANK_0;

		break;
	default:
		LL_DMA_ConfigAddresses(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uint32_t)&xDebugOutputBuff[0], LL_USART_DMA_GetRegAddr(DEBUG_OUTPUT_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		xDebugOutputBank = DEBUG_OUTPUT_BANK_1;

		break;
	}

	LL_DMA_SetDataLength(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM, (uDataBuffSize));

	uDataBuffSize = 0;

	LL_DMA_ClearFlag_TE3(DEBUG_OUTPUT_DMA_INSTANCE);
	LL_DMA_ClearFlag_HT3(DEBUG_OUTPUT_DMA_INSTANCE);
	LL_DMA_ClearFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE);
	LL_DMA_ClearFlag_FE3(DEBUG_OUTPUT_DMA_INSTANCE);

	LL_DMA_EnableStream(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);

	//	for (int i = 0; i < uDataBuffSize; i++)
	//	{
	//		while(!LL_USART_IsActiveFlag_TXE(DEBUG_OUTPUT_SERIAL_INSTANCE));
	//		LL_USART_TransmitData8(DEBUG_OUTPUT_SERIAL_INSTANCE, xDebugOutputBuff[i]);
	//	}
	//	uDataBuffSize = 0;
	//	resetSerialBusyFlag();
}

void DMA1_Stream3_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	if ((LL_DMA_IsActiveFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE)) && (LL_DMA_IsEnabledIT_TC(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM)))
	{
		LL_DMA_DisableStream(DEBUG_OUTPUT_DMA_INSTANCE, DEBUG_OUTPUT_DMA_STREAM);
		LL_DMA_ClearFlag_TC3(DEBUG_OUTPUT_DMA_INSTANCE);
		resetSerialBusyFlag();
	}
	/* USER CODE END DMA1_Stream3_IRQn 0 */

	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

//void USART3_IRQHandler(void)
//{
//	if (LL_USART_IsActiveFlag_TXE(DEBUG_OUTPUT_SERIAL_INSTANCE) & LL_USART_IsEnabledIT_TXE(DEBUG_OUTPUT_SERIAL_INSTANCE))
//	{
//
//	}
//}

// ------------------------------------------------------------------- //
