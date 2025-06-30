/*
 * MIT License 
 * 
 * Copyright (c) 2020 Kirill Kotyagin
 */

#ifndef SYSTEM_CLOCK_H
#define SYSTEM_CLOCK_H

#define FLASH_ACR_LATENCY_0WS 0x00
#define FLASH_ACR_LATENCY_1WS 0x01
#define FLASH_ACR_LATENCY_2WS 0x02

void system_clock_init();

#endif /* SYSTEM_CLOCK_H */
