/*
 * MIT License 
 * 
 * Copyright (c) 2020 Kirill Kotyagin
 */

#if defined (STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#endif

#include "system_clock.h"
#include "usb.h"

void system_clock_init() {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR |= FLASH_ACR_LATENCY_1;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  //AHB en div 1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; //APB2 en div 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB1 en div 2
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1)
        ;
    SystemCoreClockUpdate();
}
