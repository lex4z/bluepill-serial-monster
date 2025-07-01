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

#define PLL_M 25
#define PLL_N 336
#define PLL_P 0
#define PLL_Q 7
#define SYS_FREQ 168000000

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
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS_Msk)
                | PWR_CR1_VOS_1;
        RCC->CR |= RCC_CR_HSEON;
        while ((RCC->CR & RCC_CR_HSERDY) == 0);
        
        RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE | (PLL_Q << 24);
        
        RCC->CR |= RCC_CR_PLLON;

        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk))
                | RCC_CFGR_HPRE_DIV1
                | RCC_CFGR_PPRE2_DIV2
                | RCC_CFGR_PPRE1_DIV4;
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN | FLASH_ACR_LATENCY_5WS;
        while ((RCC->CR & RCC_CR_PLLRDY) == 0);
        while ((PWR->CSR1 & PWR_CSR1_VOSRDY) == 0);
        RCC->CFGR &= RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
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
