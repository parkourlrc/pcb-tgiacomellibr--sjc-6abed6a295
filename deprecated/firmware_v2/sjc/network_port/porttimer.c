/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "stm32f4xx_ll_tim.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MODBUS_TIMER_INIT MX_TIM11_Init
#define MODBUS_TIMER_INSTANCE TIM11

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
	MODBUS_TIMER_INIT();

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
//	vTaskDelay(pdMS_TO_TICKS(usTimeOutMS));
}

void prvvMBTimerIRQHandler(void)
{
	pxMBPortCBTimerExpired();
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM11) & LL_TIM_IsEnabledIT_UPDATE(TIM11))
	{
		LL_TIM_ClearFlag_UPDATE(TIM11);
		prvvMBTimerIRQHandler();
	}
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

