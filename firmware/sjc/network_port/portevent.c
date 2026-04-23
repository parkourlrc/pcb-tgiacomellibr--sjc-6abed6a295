/*
 * portevent.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  FreeModbus port: event handling for protocol state machine.
 *  Supports queue-based (blocking) or flag-based (polling) mode
 *  via USE_QUEUE_BASED_EVENT_HANDLER in port.h.
 *
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"

#ifdef USE_QUEUE_BASED_EVENT_HANDLER

/* ----------------------- Queue-based implementation -----------------------*/
#include <FreeRTOS.h>
#include <queue.h>

/* ----------------------- Defines ------------------------------------------*/
#define MB_EVENT_QUEUE_LENGTH 1
#define MB_EVENT_QUEUE_WAIT_MS 2

/* ----------------------- Variables ----------------------------------------*/
static QueueHandle_t xQueueHdl;
static StaticQueue_t xQueueControlBlock;
static uint8_t xQueueStorageBuffer[MB_EVENT_QUEUE_LENGTH * sizeof(eMBEventType)];

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit(void)
{
	BOOL bStatus = FALSE;

	xQueueHdl = xQueueCreateStatic(MB_EVENT_QUEUE_LENGTH, sizeof(eMBEventType),
								   xQueueStorageBuffer, &xQueueControlBlock);
	if (xQueueHdl != NULL)
	{
		bStatus = TRUE;
	}
	return bStatus;
}

void vMBPortEventClose(void)
{
	/* Static queue, nothing to free */
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xPortIsInsideInterrupt())
	{
		(void)xQueueSendFromISR(xQueueHdl, (const void *)&eEvent, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		(void)xQueueSend(xQueueHdl, (const void *)&eEvent, pdFALSE);
	}

	return TRUE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
	if (pdTRUE == xQueueReceive(xQueueHdl, eEvent, pdMS_TO_TICKS(MB_EVENT_QUEUE_WAIT_MS)))
	{
		return TRUE;
	}
	return FALSE;
}

BOOL xMBPortEventPending(void)
{
	return (uxQueueMessagesWaiting(xQueueHdl) > 0) ? TRUE : FALSE;
}

#else

/* ----------------------- Flag-based implementation ------------------------*/

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static BOOL xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit(void)
{
	xEventInQueue = FALSE;
	return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
	xEventInQueue = TRUE;
	eQueuedEvent = eEvent;
	return TRUE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
	BOOL xEventHappened = FALSE;

	if (xEventInQueue)
	{
		*eEvent = eQueuedEvent;
		xEventInQueue = FALSE;
		xEventHappened = TRUE;
	}
	return xEventHappened;
}

#endif /* USE_QUEUE_BASED_EVENT_HANDLER */
