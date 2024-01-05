/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define 				MODBUS_TIMER_INIT			MX_TIM11_Init
#define 				MODBUS_TIMER_INSTANCE 		TIM11

/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	MODBUS_TIMER_INIT();

	volatile uint32_t Period = usTim1Timerout50us;

	if (NETWORK_BAUD_RATE > 19200UL)
	{
		Period = ceil(3.5f * 220000UL/NETWORK_BAUD_RATE);
	}

	if (Period <2)
	{
		Period = 2;
	}

	LL_TIM_SetAutoReload(MODBUS_TIMER_INSTANCE, Period-1);

	return TRUE;
}

void
vMBPortTimerClose( void )
{

}

void
vMBPortTimersEnable(  )
{
	LL_TIM_SetCounter(MODBUS_TIMER_INSTANCE, 0);
	LL_TIM_EnableIT_UPDATE(MODBUS_TIMER_INSTANCE);
	LL_TIM_EnableCounter(MODBUS_TIMER_INSTANCE);
}

void
vMBPortTimersDisable(  )
{
	LL_TIM_DisableCounter(MODBUS_TIMER_INSTANCE);
	LL_TIM_DisableIT_UPDATE(MODBUS_TIMER_INSTANCE);
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
	vTaskDelay( pdMS_TO_TICKS(usTimeOutMS) );
}


void prvvMBTimerIRQHandler(void) {
	BOOL bTaskWoken = FALSE;

	ENTER_CRITICAL_SECTION();

	bTaskWoken |= pxMBPortCBTimerExpired();

	EXIT_CRITICAL_SECTION();

	portEND_SWITCHING_ISR(bTaskWoken ? pdTRUE : pdFALSE);
}


///* ----------------------- Platform includes --------------------------------*/
//#include "port.h"
//#include "cmsis_os2.h"
//
///* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
//#include "mbport.h"
//
///* ----------------------- static functions ---------------------------------*/
//static osTimerId_t xTimerId;
//static uint32_t ulTimeout;
//static void prvvTIMERExpiredISR(void* argument);
//
///* ----------------------- Start implementation -----------------------------*/
//BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
//{
//    ulTimeout = 50 * usTim1Timerout50us * osKernelGetTickFreq() / 1000000 + 1;
//    xTimerId = osTimerNew(prvvTIMERExpiredISR, osTimerOnce, NULL, NULL);
//    return (xTimerId != NULL);
//}
//
//void vMBPortTimersEnable()
//{
//    osTimerStart(xTimerId, ulTimeout);
//}
//
//void vMBPortTimersDisable()
//{
//    osTimerStop(xTimerId);
//}
//
//void
//vMBPortTimersDelay( USHORT usTimeOutMS )
//{
//	vTaskDelay( pdMS_TO_TICKS(usTimeOutMS) );
//}
//
//static void prvvTIMERExpiredISR(void* argument)
//{
//    ( void )pxMBPortCBTimerExpired(  );
//}
//
//void prvvMBTimerIRQHandler(void)
//{
//
//}
