/*
 * MIT License 
 * 
 * Copyright (c) 2020 Kirill Kotyagin
 */

#include "gpio.h"

#if defined(STM32F1)
static void _gpio_enable_port(GPIO_TypeDef *port) {
    int portnum = (((uint32_t)port - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE));
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN << portnum;
}
void gpio_pin_init(const gpio_pin_t *pin) {
    if (pin->port) {
        volatile uint32_t *crx = &pin->port->CRL + (pin->pin >> 3);
        uint8_t crx_offset = (pin->pin & 0x07) << 2;
        uint32_t modecfg = 0;
        _gpio_enable_port(pin->port);
        *crx &= ~((GPIO_CRL_CNF0 | GPIO_CRL_MODE0) << crx_offset);
        if (pin->dir == gpio_dir_input) {
            if (pin->pull == gpio_pull_floating) {
                modecfg |= GPIO_CRL_CNF0_0;
            } else {
                modecfg |= GPIO_CRL_CNF0_1;
                pin->port->BSRR = ((pin->pull == gpio_pull_up) ? GPIO_BSRR_BS0 : GPIO_BSRR_BR0) << pin->pin;
            }
        } else {
            switch (pin->speed) {
            case gpio_speed_unknown:
            case gpio_speed_low:
                modecfg |= GPIO_CRL_MODE0_1;
                break;
            case gpio_speed_medium:
                modecfg |= GPIO_CRL_MODE0_0;
                break;
            case gpio_speed_high:
                modecfg |= GPIO_CRL_MODE0;
                break;
            }
            if (pin->output == gpio_output_od) {
                modecfg |= GPIO_CRL_CNF0_0;
            }
            if (pin->func == gpio_func_alternate) {
                modecfg |= GPIO_CRL_CNF0_1;
            }
        }
        *crx |= (modecfg << crx_offset);
    }
}
int gpio_pin_get(const gpio_pin_t *pin) {
    if (pin->port) {
        return (!!(pin->port->IDR & (GPIO_IDR_IDR0 << pin->pin))) != (pin->polarity == gpio_polarity_low);
    }
    return 0;
}

#elif defined(STM32F4)||defined(STM32F7)
#include "stm32f7xx.h"

void gpio_pin_set(const gpio_pin_t *pin, int is_active) {
    if (pin->port) {
        pin->port->BSRR = (GPIO_BSRR_BS0 << pin->pin) 
            << (!!is_active != (pin->polarity == gpio_polarity_low) ? 0 : GPIO_BSRR_BR0_Pos);
    }
}

int gpio_pin_get(const gpio_pin_t *pin) {
    if (pin->port) {
        return (!!(pin->port->IDR & (GPIO_IDR_ID0 << pin->pin))) != (pin->polarity == gpio_polarity_low);
    }
    return 0;
}

static void _gpio_enable_port(GPIO_TypeDef *port) {
    int portnum = (((uint32_t)port - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE));
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN << portnum;
    
}

void gpio_config_mode(GPIO_TypeDef* gpio, unsigned pin, unsigned mode)
{
    gpio->MODER = (gpio->MODER & ~(3u << (2 * pin))) | (mode << (2 * pin));
}

void gpio_config_af(GPIO_TypeDef* gpio, unsigned pin, unsigned af)
{
    gpio_config_mode(gpio, pin, 2);
    unsigned pin_group = pin >> 3;
    unsigned pin_offset = pin & 7;
    gpio->AFR[pin_group] = (gpio->AFR[pin_group] & ~(0xf << (pin_offset * 4)))
            | (af << (pin_offset * 4));
}

void usb_pin_init(){
    gpio_config_af(GPIOA,  8, gpio_func_alternate10); //SOF
    gpio_config_af(GPIOA,  9, gpio_func_alternate10); // VBUS_DET
    gpio_config_af(GPIOA, 10, gpio_func_alternate10); // ID
    gpio_config_af(GPIOA, 11, gpio_func_alternate10); // DM
    gpio_config_af(GPIOA, 12, gpio_func_alternate10); // DP
}

void gpio_pin_init(const gpio_pin_t *pin) {
    if (!pin || !pin->port) return;

    _gpio_enable_port(pin->port);

    uint32_t pin_pos = pin->pin;
    uint32_t pin_mask = (1U << pin_pos);

    // --- Настройка MODER (режим пина) ---
    // 2 бита на пин: 00 - input, 01 - output, 10 - alternate function, 11 - analog
    uint32_t moder_val = pin->port->MODER;
    moder_val &= ~(0x3U << (pin_pos * 2)); // очистить биты
    if (pin->dir == gpio_dir_input) {
        moder_val |= (0x0U << (pin_pos * 2)); // input
    } else {
        if (pin->func != gpio_func_general) {
            moder_val |= (0x2U << (pin_pos * 2)); // alternate function
        } else {
            moder_val |= (0x1U << (pin_pos * 2)); // general purpose output
        }
    }
    pin->port->MODER |= moder_val;
    

    // --- Настройка OTYPER (тип выхода) ---
    uint32_t otyper_val = pin->port->OTYPER;
    if (pin->output == gpio_output_od) {
        otyper_val |= pin_mask; // open-drain
    } else {
        otyper_val &= ~pin_mask; // push-pull
    }
    pin->port->OTYPER = otyper_val;

    // --- Настройка OSPEEDR (скорость) ---
    uint32_t ospeedr_val = pin->port->OSPEEDR;
    ospeedr_val &= ~(0x3U << (pin_pos * 2)); // очистить биты скорости
    switch (pin->speed) {
        case gpio_speed_low:
            ospeedr_val |= (0x0U << (pin_pos * 2));
            break;
        case gpio_speed_medium:
            ospeedr_val |= (0x1U << (pin_pos * 2));
            break;
        case gpio_speed_high:
            ospeedr_val |= (0x2U << (pin_pos * 2));
            break;
        case gpio_speed_very_high:
            ospeedr_val |= (0x3U << (pin_pos * 2));
            break;
        default:
            break;
    }
    pin->port->OSPEEDR = ospeedr_val;

    // --- Настройка PUPDR (подтяжки) ---
    uint32_t pupdr_val = pin->port->PUPDR;
    pupdr_val &= ~(0x3U << (pin_pos * 2)); // очистить биты подтяжки
    switch (pin->pull) {
        case gpio_pull_unknown:
            pupdr_val |= (0x0U << (pin_pos * 2)); // no pull-up, no pull-down
            break;
        case gpio_pull_up:
            pupdr_val |= (0x1U << (pin_pos * 2)); // pull-up
            break;
        case gpio_pull_down:
            pupdr_val |= (0x2U << (pin_pos * 2)); // pull-down
            break;
        default:
            break;
    }
    pin->port->PUPDR = pupdr_val;

    // --- Настройка альтернативной функции (если требуется) ---
    if (pin->func != gpio_func_general) {
        uint32_t afr_index = pin_pos / 8;       // 0 для пинов 0..7, 1 для 8..15
        uint32_t afr_pos = (pin_pos % 8) * 4;  // 4 бита на пин

        uint32_t afr_val = pin->port->AFR[afr_index];
        afr_val &= ~(0xFU << afr_pos);         // очистить 4 бита
        afr_val |= ((pin->func & 0xFU) << afr_pos);
        pin->port->AFR[afr_index] = afr_val | (pin->func << afr_pos);
    }
}

volatile uint32_t *gpio_pin_get_bitband_clear_addr(const gpio_pin_t *pin) {
    if (!pin->port) return 0;  // Проверка на нулевой порт

    // Вычисляем смещение для бита в регистре BSRR
    uint32_t bit_offset = pin->pin;
    if (pin->polarity == gpio_polarity_high) {
        // Для активного высокого уровня: clear = BR (биты 16-31)
        bit_offset += 16;
    }
    // Для активного низкого: clear = BS (биты 0-15)

    // Вычисляем адрес бита в bit-band области
    uint32_t reg_addr = (uint32_t)&pin->port->BSRR;
    uint32_t byte_offset = reg_addr - PERIPH_BASE;
    uint32_t bit_band_addr = PERIPH_BASE + (byte_offset * 32) + (bit_offset * 4);

    return (volatile uint32_t*)bit_band_addr;
}




#endif