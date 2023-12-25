/* ----------------------- System includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "cmsis_os2.h"

/* ----------------------- Variables ----------------------------------------*/
static int32_t lock;

/* ----------------------- Start implementation -----------------------------*/
void vMBPortEnterCritical(void)
{
    lock = osKernelLock();
}

void vMBPortExitCritical(void)
{
    osKernelRestoreLock(lock);
}
