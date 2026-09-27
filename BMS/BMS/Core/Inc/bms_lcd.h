/*
 * bms_lcd.h
 *
 *  Created on: Sep 21, 2026
 *      Author: ADMIN
 */

#ifndef BMS_LCD_H
#define BMS_LCD_H

#include "bms_telemetry.h"
#include <stdbool.h>
#include <stdint.h>

/* HD44780 20x4 + PCF8574 LCD (reuse existing i2c-lcd.c/.h).
 * All functions run in the main loop, never from UART/TIM callbacks.
 * The screen is optional: a missing LCD must not disable BMS/UART. */
void BMS_LCD_Init(uint32_t now_ms);
void BMS_LCD_Process(const BMS_Telemetry_t *telemetry,
                     uint32_t now_ms, bool application_ready);

#endif /* BMS_LCD_H */

