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

#include "status_led.h"
#include "usb_panic.h"

void usb_panic() {
    __disable_irq();
    status_led_set(1);
    while (1) {
        __NOP();
    }
}
