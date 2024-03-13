#include "port.h"
#include "main.h"
#include "configs.h"
#include "dma.h"
#include "cmsis_os2.h"
#include "debug_output.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/

#define UART_TX_BUFF_SIZE 256
static uint8_t ucDMABuffRegister[UART_TX_BUFF_SIZE];
static uint32_t ucDMABuffRegisterIdx = 0;

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
		eMBParity eParity)
{
	BOOL bStatus = TRUE;

	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_6, (uint32_t)&ucDMABuffRegister, LL_USART_DMA_GetRegAddr(USART2), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//	 LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);

	return bStatus;
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
		LL_USART_ClearFlag_TC(MODBUS_SERIAL_INSTANCE);
		LL_USART_EnableIT_TC(MODBUS_SERIAL_INSTANCE);
		pxMBFrameCBTransmitterEmpty();
	}
	else
	{
		/* stop serial transmit */
	}
}

void vMBPortClose(void)
{
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_6);
	LL_USART_DisableIT_RXNE(MODBUS_SERIAL_INSTANCE);
	LL_USART_DisableIT_TC(MODBUS_SERIAL_INSTANCE);

}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
	LL_USART_TransmitData8(MODBUS_SERIAL_INSTANCE, ucByte );
	while (!LL_USART_IsActiveFlag_TC(MODBUS_SERIAL_INSTANCE))
	{
	};
	return TRUE;
}

BOOL xMBPortSerialPutBytes(volatile UCHAR *ucByte, USHORT usSize)
{
	for (USHORT i = 0; i < usSize; i++)
	{
		ucDMABuffRegister[ucDMABuffRegisterIdx++] = ucByte[i];
	}

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, ucDMABuffRegisterIdx);

	LL_DMA_ClearFlag_TE6(DMA1);
	LL_DMA_ClearFlag_HT6(DMA1);
	LL_DMA_ClearFlag_TC6(DMA1);

	LL_USART_EnableDMAReq_TX(MODBUS_SERIAL_INSTANCE);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

	ucDMABuffRegisterIdx = 0;

	return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
	*pucByte = LL_USART_ReceiveData8(MODBUS_SERIAL_INSTANCE);
	return TRUE;
}

void DMA1_Stream6_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC6(DMA1) == 1) {
		LL_DMA_ClearFlag_TC6(DMA1);
	}

	if (LL_DMA_IsActiveFlag_HT6(DMA1) == 1) {
		LL_DMA_ClearFlag_HT6(DMA1);
	}

	if (LL_DMA_IsActiveFlag_TE6(DMA1) == 1) {
		LL_DMA_ClearFlag_TE6(DMA1);
	}
}

void USART2_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_RXNE(MODBUS_SERIAL_INSTANCE) & LL_USART_IsEnabledIT_RXNE(MODBUS_SERIAL_INSTANCE)) {
		LL_USART_ClearFlag_RXNE(MODBUS_SERIAL_INSTANCE);
		pxMBFrameCBByteReceived();
	}

	if (LL_USART_IsActiveFlag_TC(MODBUS_SERIAL_INSTANCE) & LL_USART_IsEnabledIT_TC(MODBUS_SERIAL_INSTANCE)) {
		LL_USART_ClearFlag_TC(MODBUS_SERIAL_INSTANCE);
		LL_USART_DisableIT_TC(MODBUS_SERIAL_INSTANCE);
		pxMBFrameCBTransmitterEmpty();
//		LL_USART_DisableIT_TC(MODBUS_SERIAL_INSTANCE);
	}

//	if (LL_USART_IsActiveFlag_IDLE(MODBUS_SERIAL_INSTANCE) & LL_USART_IsEnabledIT_IDLE(MODBUS_SERIAL_INSTANCE)) {
//			LL_USART_ClearFlag_IDLE(MODBUS_SERIAL_INSTANCE);
//
//		}

//	if (LL_USART_IsActiveFlag_TXE(USART2) & LL_USART_IsEnabledIT_TXE(USART2)) {
//		pxMBFrameCBTransmitterEmpty();
//	}
}
