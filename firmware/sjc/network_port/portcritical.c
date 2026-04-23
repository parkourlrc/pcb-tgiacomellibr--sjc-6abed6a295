/*
 * portcritical.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  FreeModbus port: critical section implementation using osKernelLock.
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "cmsis_os2.h"

/* ----------------------- Start implementation -----------------------------*/
void EnterCriticalSection(void)
{
    portENTER_CRITICAL();
}

void ExitCriticalSection(void)
{
    portEXIT_CRITICAL();
}
