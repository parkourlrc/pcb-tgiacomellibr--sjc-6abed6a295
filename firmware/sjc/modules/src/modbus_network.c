/*
 * network.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "modules/include/modbus_network.h"
#include "main.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "mb.h"
#include "port.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "modules/include/debug_output.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef struct
{
	uint32_t min_baud;
	uint32_t value;
} BaudRateMap_t;

static const BaudRateMap_t baud_map[] = {
	{577000, 921600},
	{501000, 576000},
	{461000, 500000},
	{231000, 460800},
	{116000, 230400},
	{58000, 115200},
	{39000, 57600},
	{20000, 38400},
	{10000, 19200},
	{0, 9600} // fallback
};
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
eSjcReturnType_t ModbusNetwork_init(const uint32_t xSlaveAddress)
{
	eMBErrorCode eStatus;

	xNetworkParameters.Address = xSlaveAddress;
	ModbusNetwork_getBaudRate(&xNetworkParameters.Baudrate);
	xNetworkParameters.Parity = MB_PAR_NONE;

	eStatus = eMBInit(MB_RTU, xNetworkParameters.Address, 1, xNetworkParameters.Baudrate, xNetworkParameters.Parity);
	if (eStatus != MB_ENOERR)
	{
		DebugOutput_systemPrintf("\r\n[FAULT] Modbus initialization failed.");
		return SJC_RET_NOK;
	}

	for (uint32_t i = 0; i < MODBUS_USART_INIT_COUNT; ++i)
	{
		pxMBFrameCBByteReceived();
	}

	eStatus = eMBEnable();
	if (MB_ENOERR != eStatus)
	{
		DebugOutput_systemPrintf("\r\n[OS] Modbus network activation failed...");
		return SJC_RET_NOK;
	}

	// Update Network Registers
	usRegInputBuf[REG_INPUT_NETWORK_ADDRESS] = xSlaveAddress;
	usRegInputBuf[REG_INPUT_NETWORK_PARITY] = 'N';

	uint16_t *ui16ptr = (uint16_t *)(&xNetworkParameters.Baudrate);

	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE + 1] = ui16ptr[0];

	return SJC_RET_OK;
}

eSjcReturnType_t ModbusNetwork_getAddress(uint32_t *xAddress)
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

eSjcReturnType_t ModbusNetwork_getBaudRate(uint32_t *xBaudRate)
{
	if (xBaudRate == NULL)
		return SJC_RET_NOK;

	uint32_t raw = LL_USART_GetBaudRate(MODBUS_SERIAL_INSTANCE, HAL_RCC_GetPCLK1Freq(), LL_USART_OVERSAMPLING_16);

	for (uint8_t i = 0; i < sizeof(baud_map) / sizeof(baud_map[0]); ++i)
	{
		if (raw > baud_map[i].min_baud)
		{
			*xBaudRate = baud_map[i].value;
			return SJC_RET_OK;
		}
	}

	*xBaudRate = 9600;
	return SJC_RET_OK;
}

eSjcReturnType_t ModbusNetwork_getParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity)
{
	*xSlaveAddress = xNetworkParameters.Address;
	*xBaudRate = xNetworkParameters.Baudrate;
	*xParity = xNetworkParameters.Parity;

	return SJC_RET_OK;
}

void ModbusNetwork_update(void)
{
	(void)eMBPoll();
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex = usAddress - 1 - usRegInputStart;

	if ((usAddress - 1 >= REG_INPUT_START) && (usAddress - 1 + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		while (usNRegs-- > 0)
		{
			*pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] & 0xFF);
			++iRegIndex;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex = usAddress - 1 - usRegHoldingStart;

	if ((usAddress - 1 >= REG_HOLDING_START) && (usAddress - 1 + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		if (eMode == MB_REG_READ)
		{
			while (usNRegs-- > 0)
			{
				*pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
				*pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
				++iRegIndex;
			}
		}
		else if (eMode == MB_REG_WRITE)
		{
			while (usNRegs-- > 0)
			{
				usRegHoldingBuf[iRegIndex] = ((USHORT)(*pucRegBuffer++) << 8);
				usRegHoldingBuf[iRegIndex] |= (USHORT)(*pucRegBuffer++);
				++iRegIndex;
			}
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
	(void)pucRegBuffer;
	(void)usAddress;
	(void)usNCoils;
	(void)eMode;
	return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	(void)pucRegBuffer;
	(void)usAddress;
	(void)usNDiscrete;
	return MB_ENOREG;
}
// ------------------------------------------------------------------- //
