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

#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "cmsis_os.h"

#include "assert.h"
#include "inttypes.h"

#include "common/include/network_configs.h"
#include "system_types/include/types.h"
/* ----------------------- Platform structures -------------------------------*/
typedef enum
{
	MB_PORT_SERIAL_RX_ISR = 0,
	MB_PORT_SERIAL_TX_EMPTY_ISR,
	MB_PORT_SERIAL_TRANSMISSION_COMPLETE_ISR,
} eModbusUsartIrqEvent_t;

/* ----------------------- Defines ------------------------------------------*/
#define USE_QUEUE_BASED_EVENT_HANDLER

#define INLINE inline
#define PR_BEGIN_EXTERN_C \
	extern "C"            \
	{
#define PR_END_EXTERN_C }

#define ENTER_CRITICAL_SECTION() EnterCriticalSection()
#define EXIT_CRITICAL_SECTION() ExitCriticalSection()

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* ----------------------- Prototypes ---------------------------------------*/
// void vMBPortSetWithinException(BOOL bInException);
// BOOL bMBPortIsWithinException(void);
//
void EnterCriticalSection(void);
void ExitCriticalSection(void);

void prvvMBTimerIRQHandler(void);

void prvvUARTTxReadyISR(void);
void prvvUARTRxISR(void);
void prvvUARTRxReceiveCharISR(CHAR data);

#ifdef USE_QUEUE_BASED_EVENT_HANDLER
BOOL xMBPortEventPending(void);
#endif

#endif
