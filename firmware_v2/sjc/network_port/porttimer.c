/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
	volatile uint32_t Period = usTim1Timerout50us;

	if (Period < 2)
	{
		Period = 2;
	}

	LL_TIM_SetAutoReload(MODBUS_TIMER_INSTANCE, Period - 1);

	return TRUE;
}

void vMBPortTimerClose(void)
{
}

void vMBPortTimersEnable()
{
	LL_TIM_SetCounter(MODBUS_TIMER_INSTANCE, 0);
	LL_TIM_EnableIT_UPDATE(MODBUS_TIMER_INSTANCE);
	LL_TIM_EnableCounter(MODBUS_TIMER_INSTANCE);
}

void vMBPortTimersDisable()
{
	LL_TIM_DisableCounter(MODBUS_TIMER_INSTANCE);
	LL_TIM_DisableIT_UPDATE(MODBUS_TIMER_INSTANCE);
}

void vMBPortTimersDelay(USHORT usTimeOutMS)
{
	vTaskDelay(pdMS_TO_TICKS(usTimeOutMS));
}

void prvvMBTimerIRQHandler(void)
{
	pxMBPortCBTimerExpired();
}
