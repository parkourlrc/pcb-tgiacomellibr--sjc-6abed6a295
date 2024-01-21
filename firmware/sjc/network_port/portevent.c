/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "cmsis_os2.h"

/* ----------------------- Variables ----------------------------------------*/
static osEventFlagsId_t     xEventId;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    xEventId = osEventFlagsNew(NULL);
    return xEventId != NULL;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    if (xEventId != NULL)
    {
        return (osEventFlagsSet(xEventId, eEvent) & 0x80000000) == 0;
    }
    else
    {
        return FALSE;
    }
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    uint32_t receivedEvent;
    const uint32_t EVENTS = EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT;

    /* waiting forever OS event */
    receivedEvent = osEventFlagsWait(xEventId, EVENTS, osFlagsWaitAny, osWaitForever);
    switch (receivedEvent)
    {
        case EV_READY:
            *eEvent = EV_READY;
            break;

        case EV_FRAME_RECEIVED:
            *eEvent = EV_FRAME_RECEIVED;
            break;

        case EV_EXECUTE:
            *eEvent = EV_EXECUTE;
            break;

        case EV_FRAME_SENT:
            *eEvent = EV_FRAME_SENT;
            break;

        default:
            return FALSE;
    }
    return TRUE;
}

///* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
//#include "mbport.h"
//
///* ----------------------- Variables ----------------------------------------*/
//static eMBEventType eQueuedEvent;
//static BOOL     xEventInQueue;
//
///* ----------------------- Start implementation -----------------------------*/
//BOOL
//xMBPortEventInit( void )
//{
//    xEventInQueue = FALSE;
//    return TRUE;
//}
//
//BOOL
//xMBPortEventPost( eMBEventType eEvent )
//{
//    xEventInQueue = TRUE;
//    eQueuedEvent = eEvent;
//    return TRUE;
//}
//
//BOOL
//xMBPortEventGet( eMBEventType * eEvent )
//{
//    BOOL            xEventHappened = FALSE;
//
//    if( xEventInQueue )
//    {
//        *eEvent = eQueuedEvent;
//        xEventInQueue = FALSE;
//        xEventHappened = TRUE;
//    }
//    return xEventHappened;
//}


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
