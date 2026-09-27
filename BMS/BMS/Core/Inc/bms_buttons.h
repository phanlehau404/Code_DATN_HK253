/*
 * bms_button.h
 *
 *  Created on: Sep 22, 2026
 *      Author: ADMIN
 */

#ifndef BMS_BUTTONS_H
#define BMS_BUTTONS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Four independent button slots. Only BUTTON_1 is wired/configured here.
 * Do not assign the other pins until their actual board connections are known. */
typedef enum
{
    BMS_BUTTON_1_LOAD = 0u,   /* PB5: manual load toggle */
    BMS_BUTTON_2 = 1u,        /* reserved; disabled */
    BMS_BUTTON_3 = 2u,        /* reserved; disabled */
    BMS_BUTTON_4 = 3u,        /* reserved; disabled */
    BMS_BUTTON_COUNT = 4u
} BMS_ButtonId_t;

/* Diagnostics for STM32CubeIDE Live Expressions. The button module counts
 * accepted press edges, NOT whether an application action was permitted. */
extern volatile uint32_t g_bmsButtonDetectedPresses[BMS_BUTTON_COUNT];
extern volatile uint32_t g_bmsButtonDroppedPresses[BMS_BUTTON_COUNT];
extern volatile uint8_t g_bmsButtonPending[BMS_BUTTON_COUNT];
extern volatile GPIO_PinState g_bmsButtonStableState[BMS_BUTTON_COUNT];

/* Call after MX_GPIO_Init(), before starting the 1 kHz TIM2 interrupt. */
void BMS_Buttons_Init(void);

/* Call only from a genuine 1 ms timebase (TIM2 update callback here).
 * No I2C, UART, LCD or output switching is performed in interrupt context. */
void BMS_Buttons_Tick1ms(void);

/* Nonblocking: consume one debounced press in the main/application context. */
bool BMS_Buttons_TakePress(BMS_ButtonId_t button);

/* Return latest debounced GPIO level (SET for currently disabled slots). */
GPIO_PinState BMS_Buttons_GetStableState(BMS_ButtonId_t button);

#endif /* BMS_BUTTONS_H */
