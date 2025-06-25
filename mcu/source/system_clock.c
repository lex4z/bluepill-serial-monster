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
        FLASH->ACR |= FLASH_ACR_LATENCY_1WS;

        RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
        RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
    #elif defined(STM32F4) || defined(STM32F7)
        FLASH->ACR |= FLASH_ACR_PRFTEN;
        FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
        // Adjust PLL configuration for STM32F4/F7 if needed
        // Example for STM32F4:
        // RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM)) | (RCC_PLLCFGR_PLLSRC_HSE | (8 << RCC_PLLCFGR_PLLM_Pos));
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
