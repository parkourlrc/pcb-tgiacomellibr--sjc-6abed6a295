/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "main.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "mb.h"
#include "port.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "network.h"

#include "configs.h"
#include "debug_output.h"
#include "network_configs.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
volatile USHORT usRegInputBuf[REG_INPUT_NREGS];
volatile USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegHoldingStart = REG_HOLDING_START;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_Network_Init(const uint32_t xSlaveAddress)
{
	eMBErrorCode    eStatus;

	if( MB_ENOERR != ( eStatus = eMBInit( MB_RTU, xSlaveAddress, 1, NETWORK_BAUD_RATE, MB_PAR_NONE ) ) )
	{
		SJC_DebugOutput_printf("\r\n[OS] Modbus network initialization failed...");
		/* Can not initialize. Add error handling code here. */
		return SJC_RET_NOK;
	}

	uint32_t xMBInitCount = 0;

	do
	{
		pxMBFrameCBByteReceived();
	} while (xMBInitCount++ < MODBUS_USART_INIT_COUNT);

	//	if( MB_ENOERR != ( eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 ) ) )
	//	{
	//		SJC_DebugOutput_printf("\r\n[OS] Modbus slave address not set...");
	//		/* Can not set slave id. Check arguments */
	//		return SJC_RET_NOK;
	//	}

	if( MB_ENOERR != ( eStatus = eMBEnable(  ) ) )
	{
		SJC_DebugOutput_printf("\r\n[OS] Modbus network activation failed...");
		/* Enable failed. */
		return SJC_RET_NOK;
	}

	usRegHoldingBuf[0] = 1;
	SJC_DebugOutput_printf("\r\n[OS] Modbus network initiated!");
	return SJC_RET_OK;
}

void SJC_Network_Update(void)
{
	( void )eMBPoll(  );
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    USHORT usRegAddress = usAddress - 1;

    if( ( usRegAddress >= REG_INPUT_START ) && ( usRegAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        LL_GPIO_TogglePin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
    	iRegIndex = ( int )( usRegAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    USHORT usRegAddress = usAddress - 1;

    if( ( usRegAddress >= REG_HOLDING_START ) && ( usRegAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
    	LL_GPIO_TogglePin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
    	iRegIndex = ( int )( usRegAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	return MB_ENOREG;
}
// ------------------------------------------------------------------- //
