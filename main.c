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
//#include <stm32f7xx_hal_rcc.h>
#define USB_AF 10
#endif

#include "system_clock.h"
#include "system_interrupts.h"
#include "status_led.h"
#include "device_config.h"
#include "usb.h"

RCC_TypeDef *Rcc_struct;

int main() {

    //uint32_t rcc_sys_clk = HAL_RCC_GetSysClockFreq();

    //uint32_t rcc_hclk_clk = HAL_RCC_GetHCLKFreq();
    //uint32_t rcc_p1_clk = HAL_RCC_GetPCLK1Freq();
    //uint32_t rcc_p2_clk = HAL_RCC_GetPCLK2Freq();
    
    system_clock_init();
    system_interrupts_init();
    device_config_init();
    status_led_init();
    usb_init();
    while (1) {
        //__NOP();
        usb_poll();
    }
}
