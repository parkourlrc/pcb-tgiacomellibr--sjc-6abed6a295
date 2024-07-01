/*
 * network.c
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

#include "debug_output.h"
#include "network_configs.h"
#include "digital_interfaces.h"
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

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegHoldingStart = REG_HOLDING_START;
static xSjcNetworkConfig_t xNetworkParameters;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t Network_init(const uint32_t xSlaveAddress)
{
	eMBErrorCode eStatus;

	xNetworkParameters.Address = xSlaveAddress;
	Network_getBaudRate(&xNetworkParameters.Baudrate);
	xNetworkParameters.Parity = MB_PAR_NONE;

	if (MB_ENOERR != (eStatus = eMBInit(MB_RTU, xNetworkParameters.Address, 1, xNetworkParameters.Baudrate, xNetworkParameters.Parity)))
	{
		DebugOutput_systemPrintf("\r\n[OS] Modbus network initialization failed...");
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
	//		DebugOutput_systemPrintf("\r\n[OS] Modbus slave address not set...");
	//		/* Can not set slave id. Check arguments */
	//		return SJC_RET_NOK;
	//	}

	if (MB_ENOERR != (eStatus = eMBEnable()))
	{
		DebugOutput_systemPrintf("\r\n[OS] Modbus network activation failed...");
		/* Enable failed. */
		return SJC_RET_NOK;
	}

	// Update Network Registers
	usRegInputBuf[REG_INPUT_NETWORK_ADDRESS] = xSlaveAddress;
	switch (xNetworkParameters.Parity)
	{
	case MB_PAR_ODD:
		usRegInputBuf[REG_INPUT_NETWORK_PARITY] = 'O';
		break;
	case MB_PAR_EVEN:
		usRegInputBuf[REG_INPUT_NETWORK_PARITY] = 'E';
		break;
	default:
		usRegInputBuf[REG_INPUT_NETWORK_PARITY] = 'N';
		break;
	}
	uint16_t *ui16ptr = (uint16_t *)(&xNetworkParameters.Baudrate);
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE + 1] = ui16ptr[0];

	return SJC_RET_OK;
}

eSjcReturnType_t Network_getAddress(uint32_t *xAddress)
{
#if SJC_HW_VERSION_MAJOR > 1
	bool addr_a, addr_b, addr_c;
	DigitalInterfaces_readInputState(SJC_NET_ADDR_A_INPUT, &addr_a);
	DigitalInterfaces_readInputState(SJC_NET_ADDR_B_INPUT, &addr_b);
	DigitalInterfaces_readInputState(SJC_NET_ADDR_C_INPUT, &addr_c);

	*xAddress = (addr_c << 2) | (addr_b << 1) | addr_a;
	if (*xAddress == 0)
	{
		*xAddress = 247;
	}
#else
	*xAddress = NETWORK_BASE_ADRESS;
#endif

	return SJC_RET_OK;
}

eSjcReturnType_t Network_getBaudRate(uint32_t *xBaudRate)
{
	uint32_t baudrate = LL_USART_GetBaudRate(MODBUS_SERIAL_INSTANCE, HAL_RCC_GetPCLK1Freq(), LL_USART_OVERSAMPLING_16);

	if (baudrate > 577000)
	{
		*xBaudRate = 921600;
	}
	else if (baudrate > 501000)
	{
		*xBaudRate = 576000;
	}
	else if (baudrate > 461000)
	{
		*xBaudRate = 500000;
	}
	else if (baudrate > 231000)
	{
		*xBaudRate = 460800;
	}
	else if (baudrate > 116000)
	{
		*xBaudRate = 230400;
	}
	else if (baudrate > 58000)
	{
		*xBaudRate = 115200;
	}
	else if (baudrate > 39000)
	{
		*xBaudRate = 57600;
	}
	else if (baudrate > 20000)
	{
		*xBaudRate = 38400;
	}
	else if (baudrate > 10000)
	{
		*xBaudRate = 19200;
	}
	else
	{
		*xBaudRate = 9600;
	}

	return SJC_RET_OK;
}

void Network_update(void)
{
	(void)eMBPoll();
}

void Network_processReceivedData(void)
{

}

eSjcReturnType_t Network_getParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity)
{
	*xSlaveAddress = xNetworkParameters.Address;
	*xBaudRate = xNetworkParameters.Baudrate;
	*xParity = xNetworkParameters.Parity;

	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eMBErrorCode
eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;
	USHORT usRegAddress = usAddress - 1;

	if ((usRegAddress >= REG_INPUT_START) && (usRegAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		iRegIndex = (int)(usRegAddress - usRegInputStart);
		while (usNRegs > 0)
		{
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
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
eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;
	USHORT usRegAddress = usAddress - 1;

	if ((usRegAddress >= REG_HOLDING_START) && (usRegAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		iRegIndex = (int)(usRegAddress - usRegHoldingStart);
		switch (eMode)
		{
		case MB_REG_READ:
			while (usNRegs > 0)
			{
				*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
				*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
				iRegIndex++;
				usNRegs--;
			}
			break;

		case MB_REG_WRITE:
			while (usNRegs > 0)
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
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	return MB_ENOREG;
}
// ------------------------------------------------------------------- //
