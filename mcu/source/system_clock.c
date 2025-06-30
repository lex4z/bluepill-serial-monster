/*
 * MIT License 
 * 
 * Copyright (c) 2020 Kirill Kotyagin
 */

#if defined (STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#elif defined(STM32F7)
#include <stm32f7xx.h>
#endif

#include "system_clock.h"
#include "usb.h"

void system_clock_init() {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;
    #if defined(STM32F1)
        FLASH->ACR |= FLASH_ACR_PRFTBE;
        FLASH->ACR |= FLASH_ACR_LATENCY_1;

        RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
        RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
    #elif defined(STM32F4) || defined(STM32F7)
        FLASH->ACR |= FLASH_ACR_PRFTEN;
        FLASH->ACR |= FLASH_ACR_LATENCY_1WS;

        RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)   // PLLM = 8
             | (192 << RCC_PLLCFGR_PLLN_Pos) // PLLN = 192
             | (0 << RCC_PLLCFGR_PLLP_Pos)   // PLLP = 2 (for SYSCLK = 96 MHz)
             | (4 << RCC_PLLCFGR_PLLQ_Pos)   // PLLQ = 4 (for 48 MHz)
             | RCC_PLLCFGR_PLLSRC_HSE;       // Use HSE as PLL source
    #endif
    
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  //AHB en div 1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; //APB2 en div 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB1 en div 2
    
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1)
        ;
    SystemCoreClockUpdate();
}
