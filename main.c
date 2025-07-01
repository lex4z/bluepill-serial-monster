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
#define USB_AF 10
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
    status_led_init();
    gpio_config_af(GPIOA,  9, USB_AF); // VBUS_DET
    gpio_config_af(GPIOA, 10, USB_AF); // ID
    gpio_config_af(GPIOA, 11, USB_AF); // DM
    gpio_config_af(GPIOA, 12, USB_AF); // DP
    usb_init();
    while (1) {
        //__NOP();
        usb_poll();
    }
}
