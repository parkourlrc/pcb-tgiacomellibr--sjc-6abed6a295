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

#include "configs.h"
#include "network_configs.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define DEBUG_IFC_INSTANCE USART3
#define DEBUG_BUFF_SIZE (512UL)
#define DEBUG_USB_BUFF_SIZE (512UL)

#define CONTROLLER_USB_DBG_SEMPHR_WAIT_TIME_MS			((uint32_t) 10U)
#define CONTROLLER_SERIAL_DBG_SEMPHR_WAIT_TIME_MS		((uint32_t) 10U)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static char xSystemDebugBuff[DEBUG_BUFF_SIZE];
static char xLoggerDebugBuff[DEBUG_USB_BUFF_SIZE];

osSemaphoreId_t xSystemDebugSemprHandle = NULL;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
		.name = "SerialDebugSemaphore",
		.cb_mem = &SerialDebugSemaphoreControlBlock,
		.cb_size = sizeof(SerialDebugSemaphoreControlBlock),
};

osSemaphoreId_t xDataLoggerSemprHandle = NULL;
StaticSemaphore_t UsbDebugSemaphoreControlBlock;
const osSemaphoreAttr_t UsbDebugSemaphore_attributes = {
		.name = "UsbDebugSemaphore",
		.cb_mem = &UsbDebugSemaphoreControlBlock,
		.cb_size = sizeof(UsbDebugSemaphoreControlBlock),
};
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
//int _write(int fd, char *ptr, int len);
int __io_putchar(int ch);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_DebugOutput_Init(void)
{
	xSystemDebugSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SerialDebugSemaphore_attributes);
	xDataLoggerSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &UsbDebugSemaphore_attributes);

	if (xSystemDebugSemprHandle == NULL)
	{
		SJC_DebugOutput_SystemPrintf("\r\n[OS FAULT] Serial debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(xSystemDebugSemprHandle);
	}


	if (xDataLoggerSemprHandle == NULL)
	{
		SJC_DebugOutput_SystemPrintf("\r\n[OS FAULT] USB debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(xDataLoggerSemprHandle);
	}
}

void SJC_DebugOutput_PresentSystemInfo(void)
{
	SJC_DebugOutput_SystemPrintf("\f\r\n************************************************************");
	SJC_DebugOutput_SystemPrintf("\r\n                      Crane Control");
	SJC_DebugOutput_SystemPrintf("\r\n               Single Joint Control Board");
	SJC_DebugOutput_SystemPrintf("\r\n                         (SJC)");
	SJC_DebugOutput_SystemPrintf("\r\n************************************************************");
	SJC_DebugOutput_SystemPrintf("\r\n\n Information: ");
	SJC_DebugOutput_SystemPrintf("\r\n - HW Version: %u.%u", SJC_HW_VERSION_MAJOR, SJC_HW_VERSION_MINOR);
	SJC_DebugOutput_SystemPrintf("\r\n - FW Version: %u.%u.%u (%u/%lu)", SJC_FW_VERSION_MAJOR, SJC_FW_VERSION_MINOR, SJC_FW_VERSION_PATCH, SJC_FW_RELEASE_MONTH, SJC_FW_RELEASE_YEAR);
	SJC_DebugOutput_SystemPrintf("\r\n - O.S. Tick Rate: %lu kHz", configTICK_RATE_HZ/1000);
	SJC_DebugOutput_SystemPrintf("\r\n - Network address: %lu: ", NETWORK_BASE_ADRESS);
	SJC_DebugOutput_SystemPrintf("\r\n - Network baudrate: %lu: bps", NETWORK_BAUD_RATE);
	SJC_DebugOutput_SystemPrintf("\r\n************************************************************\n\n");
}

void SJC_DebugOutput_PresentRuntimeStatistics(void)
{
	// Statistics
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;
	float runtime_percent;
	uxArraySize = uxTaskGetNumberOfTasks();
	pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); // a little bit scary!

	if (pxTaskStatusArray != NULL) {
		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
		SJC_DebugOutput_SystemPrintf("\r\n************************************************************");
		SJC_DebugOutput_SystemPrintf("\r\nTask count = %lu", uxArraySize);
		SJC_DebugOutput_SystemPrintf("\r\n\nNo\tName\tS Usage\tHW");

		for (x = 0; x < uxArraySize; x++) {

			runtime_percent = (float) (100 * (float) pxTaskStatusArray[x].ulRunTimeCounter / (float) ulTotalRunTime);

			SJC_DebugOutput_SystemPrintf("\r\nTask %lu: %-16s\t\t\t\t%2d\t%7.3f\t\t%4i", x, pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].eCurrentState, runtime_percent, pxTaskStatusArray[x].usStackHighWaterMark);
		}
		SJC_DebugOutput_SystemPrintf("\r\n************************************************************");

		vPortFree(pxTaskStatusArray);

	} else {
		SJC_DebugOutput_SystemPrintf("\r\nUnable to allocate stack space");
	}
}

int SJC_DebugOutput_SystemPrintf(const char *format, ...)
{
	configASSERT( xSystemDebugSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xSystemDebugSemprHandle, CONTROLLER_SERIAL_DBG_SEMPHR_WAIT_TIME_MS))
	{
		va_list apx;
		int n = 0;
		va_start(apx, format);
		n = vsnprintf(xLoggerDebugBuff, DEBUG_USB_BUFF_SIZE, format, apx);
		va_end(apx);

		if (n > 0)
		{
			CDC_Transmit_FS(xLoggerDebugBuff, n);
		}

		osSemaphoreRelease(xSystemDebugSemprHandle);

		return n;
	}
	else
	{
		return -1;
	}
}

int SJC_DebugOutput_LoggerPrintf(const char *format, ...)
{
	configASSERT( xDataLoggerSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xDataLoggerSemprHandle, CONTROLLER_SERIAL_DBG_SEMPHR_WAIT_TIME_MS))
	{
		va_list ap;
		int n = 0;
		va_start(ap, format);
		n = vsnprintf(xSystemDebugBuff, DEBUG_BUFF_SIZE, format, ap);
		va_end(ap);

		if (n > 0)
		{
			for (int j = 0; j < n; j++)
			{
				__io_putchar(xSystemDebugBuff[j]);
			}
		}


		osSemaphoreRelease(xDataLoggerSemprHandle);

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
int __io_putchar(int ch)
{
	while (!LL_USART_IsActiveFlag_TXE(DEBUG_IFC_INSTANCE));
	LL_USART_TransmitData8(DEBUG_IFC_INSTANCE, *((uint8_t *)(&ch)));
	return (ch);
}

//int _write(int fd, char *ptr, int len) {
//	int BytesToSend = len;
//	int Idx = 0;
//	do {
//		//waiting until the Transmit Empty flag is set
//		while (!LL_USART_IsActiveFlag_TXE(DEBUG_IFC_INSTANCE)) {
//		};
//		//send data byte by byte
//		LL_USART_TransmitData8(DEBUG_IFC_INSTANCE, ptr[Idx++]);
//
//	} while (BytesToSend--);
//
//	//Wait until the transmit complete Flag to be raised
//	while (!LL_USART_IsActiveFlag_TC(USART2)) {
//	};
//
//	return len;
//}



// ------------------------------------------------------------------- //
