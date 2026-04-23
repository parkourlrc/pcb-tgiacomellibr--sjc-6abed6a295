/*
 * debug_output.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Mutex-protected USB CDC debug output with formatted string support.
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
#include "base/include/configs.h"
#include "common/include/network_configs.h"
#include "modules/include/debug_output.h"
#include "modules/include/modbus_network.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define SYSTEM_DEBUG_BUFF_SIZE (1024UL)
#define SYSTEM_DEBUG_MUTEX_WAIT_TIME_MS ((uint32_t)10U)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static char xSystemDebugBuff[SYSTEM_DEBUG_BUFF_SIZE];

static osMutexId_t xSystemDebugMutexHandle = NULL;
static StaticSemaphore_t SerialDebugMutexControlBlock;
static const osMutexAttr_t SerialDebugMutex_attributes = {
	.name = "SystemDebugMutex",
	.cb_mem = &SerialDebugMutexControlBlock,
	.cb_size = sizeof(SerialDebugMutexControlBlock),
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
	xSystemDebugMutexHandle = osMutexNew(&SerialDebugMutex_attributes);

	if (xSystemDebugMutexHandle == NULL)
	{
		SJC_SYSTEM_RESET();
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
							 xBaudRate);
}

void DebugOutput_presentRuntimeStatistics(void)
{
}

int DebugOutput_systemPrintf(const char *format, ...)
{
	configASSERT(xSystemDebugMutexHandle != NULL);

	if (osOK == osMutexAcquire(xSystemDebugMutexHandle, SYSTEM_DEBUG_MUTEX_WAIT_TIME_MS))
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
int _write(int fd, char *ptr, int len)
{
	if (xSystemDebugMutexHandle != NULL)
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
	DebugOutput_systemPrintf("%c", ch);
	return (ch);
}
// ------------------------------------------------------------------- //
