/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "cmsis_os2.h"

/* ----------------------- Variables ----------------------------------------*/
static int32_t lock;

/* ----------------------- Start implementation -----------------------------*/
void EnterCriticalSection(void)
{
    lock = osKernelLock();
}

void ExitCriticalSection(void)
{
    osKernelRestoreLock(lock);
}
