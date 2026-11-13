#ifndef __APP_H
#define __APP_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "main.h"

static const char name_project[] = {
    "Project: LoRa2021_STM32F401RCT6\n"
};

static const char info_project[] = {
    "MCU:      STM32F401RCT6\n"
    "Radio:    LR2021\n"
    "Mod.:     FSK\n"
    "Bit rate: 0.5 Mbit/s\n"
    "Role:     TX/RX\n\n"
};

void app();

void setSeparator(char ch, uint8_t qnt);

#endif /*__APP_H*/