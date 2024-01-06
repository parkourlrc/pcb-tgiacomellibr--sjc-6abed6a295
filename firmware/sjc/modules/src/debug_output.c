/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
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
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static char g_debug_buff[DEBUG_BUFF_SIZE];
static char g_usb_debug_buff[DEBUG_USB_BUFF_SIZE];
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
	MX_USB_DEVICE_Init();
	MX_USART3_UART_Init();
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

		return n;
	}

int SJC_DebugOutput_UsbPrintf(const char *format, ...)
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

	return n;

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
