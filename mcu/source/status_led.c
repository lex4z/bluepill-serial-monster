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

#include "gpio.h"
#include "device_config.h"
#include "status_led.h"

static gpio_pin_t *status_led_pin;

void status_led_init() {
    status_led_pin = &device_config_get()->status_led_pin;
    gpio_pin_init(status_led_pin);
    gpio_pin_set(status_led_pin, 0);
}

void status_led_set(int on) {
    (void)status_led_set; /* This function does not have to be used */
    gpio_pin_set(status_led_pin, on);
}
