#if defined (STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#endif

#include <string.h>
#include "system_interrupts.h"

void system_interrupts_init() {
    NVIC_SetPriorityGrouping(SYSTEM_INTERRUPTS_PRIORITY_GROUPING);
}
