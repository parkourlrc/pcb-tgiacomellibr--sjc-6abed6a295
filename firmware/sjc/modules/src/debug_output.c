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
static char g_debug_buff[DEBUG_BUFF_SIZE];
static char g_usb_debug_buff[DEBUG_USB_BUFF_SIZE];

osSemaphoreId_t xSerialDebugSemprHandle = NULL;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
		.name = "SerialDebugSemaphore",
		.cb_mem = &SerialDebugSemaphoreControlBlock,
		.cb_size = sizeof(SerialDebugSemaphoreControlBlock),
};

osSemaphoreId_t xUsbDebugSemprHandle = NULL;
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
	xSerialDebugSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SerialDebugSemaphore_attributes);
	xUsbDebugSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &UsbDebugSemaphore_attributes);

	if (xSerialDebugSemprHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Serial debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(xSerialDebugSemprHandle);
	}


	if (xUsbDebugSemprHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] USB debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(xUsbDebugSemprHandle);
	}
}

void SJC_DebugOutput_PresentSystemInfo(void)
{
	SJC_DebugOutput_printf(
			"\f\r\n------------------------------------------------------------");
	SJC_DebugOutput_printf(
			"\r\n                        Crane Control                        ");
	SJC_DebugOutput_printf(
			"\r\n              Single Joint Control Board (SJC)               ");
	SJC_DebugOutput_printf(
			"\r\n               FW Version: %u.%u.%u (%u/%lu)                 ",
			SJC_FW_VERSION_MAJOR, SJC_FW_VERSION_MINOR, SJC_FW_VERSION_PATCH,
			SJC_FW_RELEASE_MONTH, SJC_FW_RELEASE_YEAR);
	SJC_DebugOutput_printf(
			"\r\n                       HW Version: %u.%u                     ",
			SJC_HW_VERSION_MAJOR, SJC_HW_VERSION_MINOR);
	SJC_DebugOutput_printf(
			"\r\n                 O.S. Tick Frequency: %lu kHz                ",
			configTICK_RATE_HZ/1000);
	SJC_DebugOutput_printf(
			"\r\n------------------------------------------------------------\n");
}

int SJC_DebugOutput_printf(const char *format, ...)
{
	configASSERT( xSerialDebugSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xSerialDebugSemprHandle, CONTROLLER_SERIAL_DBG_SEMPHR_WAIT_TIME_MS))
	{
		va_list ap;
		int n = 0;
		va_start(ap, format);
		n = vsnprintf(g_debug_buff, DEBUG_BUFF_SIZE, format, ap);
		va_end(ap);

		if (n > 0)
		{
			for (int j = 0; j < n; j++)
			{
				__io_putchar(g_debug_buff[j]);
			}
		}

		osSemaphoreRelease(xSerialDebugSemprHandle);

		return n;
	}
	else
	{
		return -1;
	}
}

int SJC_DebugOutput_UsbPrintf(const char *format, ...)
{
	configASSERT( xUsbDebugSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xUsbDebugSemprHandle, CONTROLLER_SERIAL_DBG_SEMPHR_WAIT_TIME_MS))
	{
		va_list apx;
		int n = 0;
		va_start(apx, format);
		n = vsnprintf(g_usb_debug_buff, DEBUG_USB_BUFF_SIZE, format, apx);
		va_end(apx);

		if (n > 0)
		{
			CDC_Transmit_FS(g_usb_debug_buff, n);
		}

		osSemaphoreRelease(xUsbDebugSemprHandle);

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
