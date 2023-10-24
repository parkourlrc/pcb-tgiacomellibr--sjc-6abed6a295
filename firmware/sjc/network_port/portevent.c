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

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static BOOL     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;
    return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL            xEventHappened = FALSE;

    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    return xEventHappened;
}


///* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
//#include "mbport.h"
//#include "cmsis_os2.h"
//
///* ----------------------- Variables ----------------------------------------*/
//static osEventFlagsId_t     xEventId;
//
///* ----------------------- Start implementation -----------------------------*/
//BOOL
//xMBPortEventInit( void )
//{
//    xEventId = osEventFlagsNew(NULL);
//    return xEventId != NULL;
//}
//
//BOOL
//xMBPortEventPost( eMBEventType eEvent )
//{
//    if (xEventId != NULL)
//    {
//        return (osEventFlagsSet(xEventId, eEvent) & 0x80000000) == 0;
//    }
//    else
//    {
//        return FALSE;
//    }
//}
//
//BOOL
//xMBPortEventGet( eMBEventType * eEvent )
//{
//    uint32_t receivedEvent;
//    const uint32_t EVENTS = EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT;
//
//    /* waiting forever OS event */
//    receivedEvent = osEventFlagsWait(xEventId, EVENTS, osFlagsWaitAny, osWaitForever);
//    switch (receivedEvent)
//    {
//        case EV_READY:
//            *eEvent = EV_READY;
//            break;
//
//        case EV_FRAME_RECEIVED:
//            *eEvent = EV_FRAME_RECEIVED;
//            break;
//
//        case EV_EXECUTE:
//            *eEvent = EV_EXECUTE;
//            break;
//
//        case EV_FRAME_SENT:
//            *eEvent = EV_FRAME_SENT;
//            break;
//
//        default:
//            return FALSE;
//    }
//    return TRUE;
//}
