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
#include "system_interrupts.h"
#include "status_led.h"
#include "device_config.h"
#include "usb.h"

int main() {

    system_clock_init();
    system_interrupts_init();
    device_config_init();
    usb_pin_config();
    status_led_init();
    usb_init();
    while (1) {
        //__NOP();
        usb_poll();
    }
}
