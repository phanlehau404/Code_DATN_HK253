/*
 * bms_telemetry.h
 *
 *  Created on: Sep 20, 2026
 *      Author: ADMIN
 */

#ifndef BMS_TELEMETRY_H
#define BMS_TELEMETRY_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* On-wire BMS protocol, distinct from MPPT node 0x01.
 * Request ESP32->STM32: AA 00 03 01 seq_lo seq_hi CRC8
 * Response STM32->ESP32: AA 02 15 91 seq(2) cell_mV[4](8)
 *   pack_mV(2) current_mA(2 signed) soc_centi_pct(2)
 *   pack_temperature_cC(2 signed) output_enabled(1) validity_flags(1) CRC8
 * Little endian, CRC-8 poly 0x07 init 0x00 over SRC, LEN and PAYLOAD.
 */
#define BMS_FLAG_VOLTAGE_VALID  0x01u
#define BMS_FLAG_TEMP_VALID     0x02u
#define BMS_FLAG_SOC_VALID      0x04u
#define BMS_FLAG_CURRENT_VALID  0x08u
#define BMS_FLAG_FAULT_LATCHED  0x10u

typedef struct {
    uint16_t cell_mV[4];
    uint16_t pack_mV;
    int16_t current_mA; /* positive = discharge (verify sensor orientation) */
    uint16_t soc_centi_pct; /* PACK SOC; not SOC for individual cells */
    int16_t pack_temp_cC; /* One LM75, NOT an individual-cell sensor */
    uint8_t output_enabled;
    uint8_t flags;
    uint32_t sampled_at_ms;
} BMS_Telemetry_t;

HAL_StatusTypeDef BMS_Comm_Init(UART_HandleTypeDef *uart);
void BMS_Comm_OnRxByte(UART_HandleTypeDef *uart);
void BMS_Comm_OnUartError(UART_HandleTypeDef *uart);
void BMS_Comm_Service(const BMS_Telemetry_t *latest, uint32_t now_ms);

#endif
