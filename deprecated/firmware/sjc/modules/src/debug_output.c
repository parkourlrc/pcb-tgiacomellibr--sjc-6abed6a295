/*
 * debug_output.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "modules/include/modbus_network.h"
#include "usart.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/network_configs.h"

#include "modules/include/debug_output.h"
#include "modules/include/modbus_network.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define SYSTEM_DEBUG_BUFF_SIZE (1024UL)
#define SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS ((uint32_t)10U)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static char xSystemDebugBuff[SYSTEM_DEBUG_BUFF_SIZE];

osSemaphoreId_t xSystemDebugSemprHandle = NULL;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
	.name = "SystemDebugSemaphore",
	.cb_mem = &SerialDebugSemaphoreControlBlock,
	.cb_size = sizeof(SerialDebugSemaphoreControlBlock),
};
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
int _write(int fd, char *ptr, int len);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void DebugOutput_init(void)
{
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
	ModbusNetwork_getBaudRate(&xBaudRate);
	uint32_t xNetworkAddress;
	ModbusNetwork_getAddress(&xNetworkAddress);

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

void DebugOutput_presentRuntimeStatistics(void)
{

}

int DebugOutput_systemPrintf(const char *format, ...)
{
	configASSERT(xSystemDebugSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xSystemDebugSemprHandle, SYSTEM_DEBUG_SEMPHR_WAIT_TIME_MS))
	{
		va_list apx;
		int n = 0;
		va_start(apx, format);
		n = vsnprintf(xSystemDebugBuff, SYSTEM_DEBUG_BUFF_SIZE, format, apx);
		va_end(apx);

		if (n > 0)
		{
			_write(0, xSystemDebugBuff, n);
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
int _write(int fd, char *ptr, int len) {
	if (xSystemDebugSemprHandle != NULL)
	{
		CDC_Transmit_FS((uint8_t *)(&ptr[0]), len);
		return len;
	}
	else
	{
		return 0;
	}
}

int __io_putchar(int ch)
{
	DebugOutput_systemPrintf("c", ch);
	return (ch);
}
// ------------------------------------------------------------------- //
