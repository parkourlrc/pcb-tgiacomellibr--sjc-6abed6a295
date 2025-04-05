#include "port.h"
#include "main.h"
#include "base/include/configs.h"
#include "dma.h"
#include "cmsis_os2.h"
#include "modules/include/debug_output.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
static volatile uint8_t rx_data = 0;

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
		eMBParity eParity)
{
	// Init Serial communication device
	LL_USART_EnableDMAReq_TX(MODBUS_SERIAL_INSTANCE);

	return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	if (xRxEnable)
	{
		/* switch 485 to receive mode */
		LL_USART_EnableIT_RXNE(MODBUS_SERIAL_INSTANCE);
		MODBUS_SET_RX_MODE();
	}
	else
	{
		/* switch 485 to transmit mode */
		LL_USART_DisableIT_RXNE(MODBUS_SERIAL_INSTANCE);
		MODBUS_SET_TX_MODE();
	}

	if (xTxEnable)
	{
		/* start serial transmit */
		pxMBFrameCBTransmitterEmpty();
	}
	else
	{
		/* stop serial transmit */
		LL_USART_DisableIT_TC(MODBUS_SERIAL_INSTANCE);
	}
}

void vMBPortClose(void)
{
	LL_USART_DisableIT_RXNE(MODBUS_SERIAL_INSTANCE);
	LL_USART_DisableIT_TC(MODBUS_SERIAL_INSTANCE);
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
	LL_USART_TransmitData8(MODBUS_SERIAL_INSTANCE, ucByte);
	while (!LL_USART_IsActiveFlag_TC(MODBUS_SERIAL_INSTANCE))
	{
	};
	return TRUE;
}

BOOL xMBPortSerialPutBytes(volatile UCHAR *ucByte, USHORT usSize)
{
	LL_DMA_DisableStream(MODBUS_SERIAL_DMA_INSTANCE, MODBUS_SERIAL_DMA_STREAM);

	LL_DMA_ConfigAddresses(MODBUS_SERIAL_DMA_INSTANCE, MODBUS_SERIAL_DMA_STREAM, (uint32_t)&ucByte[0], LL_USART_DMA_GetRegAddr(MODBUS_SERIAL_INSTANCE), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_SetDataLength(MODBUS_SERIAL_DMA_INSTANCE, MODBUS_SERIAL_DMA_STREAM, usSize);

	LL_DMA_ClearFlag_TE6(MODBUS_SERIAL_DMA_INSTANCE);
	LL_DMA_ClearFlag_HT6(MODBUS_SERIAL_DMA_INSTANCE);
	LL_DMA_ClearFlag_TC6(MODBUS_SERIAL_DMA_INSTANCE);
	LL_DMA_ClearFlag_FE6(MODBUS_SERIAL_DMA_INSTANCE);

	// Limpa TC antes de escutar
	LL_USART_ClearFlag_TC(MODBUS_SERIAL_INSTANCE);

	LL_DMA_EnableStream(MODBUS_SERIAL_DMA_INSTANCE, MODBUS_SERIAL_DMA_STREAM);

	// Só agora ativa interrupção por TC
	LL_USART_EnableIT_TC(MODBUS_SERIAL_INSTANCE);

	return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
	*pucByte = rx_data;
	return TRUE;
}

void USART2_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_RXNE(MODBUS_SERIAL_INSTANCE) && LL_USART_IsEnabledIT_RXNE(MODBUS_SERIAL_INSTANCE))
	{
		// Real RXNE clearing happens here:
		rx_data = LL_USART_ReceiveData8(MODBUS_SERIAL_INSTANCE);

		LL_USART_ClearFlag_RXNE(MODBUS_SERIAL_INSTANCE);
//		DebugOutput_systemPrintf("Byte recebido na UART\n");
		pxMBFrameCBByteReceived();
	}

	if (LL_USART_IsActiveFlag_TC(MODBUS_SERIAL_INSTANCE) && LL_USART_IsEnabledIT_TC(MODBUS_SERIAL_INSTANCE))
	{
		LL_USART_ClearFlag_TC(MODBUS_SERIAL_INSTANCE);
//		DebugOutput_systemPrintf("Transmissão concluída na UART\n");
		pxMBFrameCBTransmitterEmpty();
	}
}
