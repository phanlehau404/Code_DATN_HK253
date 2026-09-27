/*
 * bms_button.c
 *
 *  Created on: Sep 22, 2026
 *      Author: ADMIN
 */


/* bms_buttons.c - independent, reusable 4-button debounce/event module. */
#include "bms_buttons.h"

#define BMS_BUTTON_DEFAULT_DEBOUNCE_MS 30u

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pressedLevel;
    uint8_t debounceMs;
    bool enabled;
} BMS_ButtonConfig_t;

typedef struct
{
    GPIO_PinState lastRaw;
    GPIO_PinState stable;
    uint8_t stableTicks;
} BMS_ButtonRuntime_t;

/* 3 spare slots intentionally have no GPIO assignment. Later: add the
 * actual port/pin and set enabled=true AFTER configuring those GPIOs in .ioc. */
static const BMS_ButtonConfig_t s_buttonConfig[BMS_BUTTON_COUNT] =
{
    [BMS_BUTTON_1_LOAD] = { GPIOB, GPIO_PIN_5, GPIO_PIN_RESET,
                            BMS_BUTTON_DEFAULT_DEBOUNCE_MS, true },
    [BMS_BUTTON_2]      = { NULL, 0u, GPIO_PIN_RESET,
                            BMS_BUTTON_DEFAULT_DEBOUNCE_MS, false },
    [BMS_BUTTON_3]      = { NULL, 0u, GPIO_PIN_RESET,
                            BMS_BUTTON_DEFAULT_DEBOUNCE_MS, false },
    [BMS_BUTTON_4]      = { NULL, 0u, GPIO_PIN_RESET,
                            BMS_BUTTON_DEFAULT_DEBOUNCE_MS, false }
};

static BMS_ButtonRuntime_t s_buttonRuntime[BMS_BUTTON_COUNT];

volatile uint32_t g_bmsButtonDetectedPresses[BMS_BUTTON_COUNT];
volatile uint32_t g_bmsButtonDroppedPresses[BMS_BUTTON_COUNT];
volatile uint8_t g_bmsButtonPending[BMS_BUTTON_COUNT];
volatile GPIO_PinState g_bmsButtonStableState[BMS_BUTTON_COUNT];

void BMS_Buttons_Init(void)
{
    for (uint8_t i = 0u; i < BMS_BUTTON_COUNT; ++i)
    {
        const BMS_ButtonConfig_t *cfg = &s_buttonConfig[i];
        GPIO_PinState level = GPIO_PIN_SET;
        if (cfg->enabled && cfg->port != NULL && cfg->pin != 0u)
            level = HAL_GPIO_ReadPin(cfg->port, cfg->pin);

        /* Seed both states from hardware: holding a button during boot
         * does NOT generate a spurious press. Release, then press again. */
        s_buttonRuntime[i].lastRaw = level;
        s_buttonRuntime[i].stable = level;
        s_buttonRuntime[i].stableTicks = 0u;
        g_bmsButtonStableState[i] = level;
        g_bmsButtonDetectedPresses[i] = 0u;
        g_bmsButtonDroppedPresses[i] = 0u;
        g_bmsButtonPending[i] = 0u;
    }
}

void BMS_Buttons_Tick1ms(void)
{
    for (uint8_t i = 0u; i < BMS_BUTTON_COUNT; ++i)
    {
        const BMS_ButtonConfig_t *cfg = &s_buttonConfig[i];
        BMS_ButtonRuntime_t *state = &s_buttonRuntime[i];
        if (!cfg->enabled || cfg->port == NULL || cfg->pin == 0u)
            continue;

        const GPIO_PinState raw = HAL_GPIO_ReadPin(cfg->port, cfg->pin);
        if (raw != state->lastRaw)
        {
            state->lastRaw = raw;
            state->stableTicks = 0u;
            continue;
        }

        if (state->stableTicks < cfg->debounceMs)
            ++state->stableTicks;
        if (state->stableTicks < cfg->debounceMs || raw == state->stable)
            continue;

        state->stable = raw;
        g_bmsButtonStableState[i] = raw;
        if (raw == cfg->pressedLevel)
        {
            ++g_bmsButtonDetectedPresses[i];
            if (g_bmsButtonPending[i] < UINT8_MAX)
                ++g_bmsButtonPending[i];
            else
                ++g_bmsButtonDroppedPresses[i];
        }
    }
}

bool BMS_Buttons_TakePress(BMS_ButtonId_t button)
{
    if ((unsigned int)button >= BMS_BUTTON_COUNT)
        return false;

    /* Single producer (TIM2 interrupt), single consumer (main loop).
     * Mask IRQs for the brief read-modify-write so ISR cannot race it. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool pressed = (g_bmsButtonPending[button] != 0u);
    if (pressed)
        --g_bmsButtonPending[button];
    if (primask == 0u)
        __enable_irq();
    return pressed;
}

GPIO_PinState BMS_Buttons_GetStableState(BMS_ButtonId_t button)
{
    if ((unsigned int)button >= BMS_BUTTON_COUNT)
        return GPIO_PIN_SET;
    return g_bmsButtonStableState[button];
}
