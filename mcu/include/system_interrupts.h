#ifndef SYSTEM_INTERRUPTS_H
#define SYSTEM_INTERRUPTS_H

#if defined (STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#elif defined(STM32F7)
#include <stm32f7xx.h>
#endif

#define SYSTEM_INTERRUPTS_PRIORITY_GROUPING     0x02 /* 2 bits preemption, 2 bits sub-priority */

#define SYSTEM_INTERRUTPS_PRIORITY_BASE         (NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0))
#define SYSTEM_INTERRUTPS_PRIORITY_HIGH         (NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0))
#define SYSTEM_INTERRUTPS_PRIORITY_CRITICAL     (NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0))

void system_interrupts_init();

#endif /*  SYSTEM_INTERRUPTS_H */
