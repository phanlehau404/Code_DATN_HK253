/*
 * bms_telemetry.c
 *
 *  Created on: Sep 20, 2026
 *      Author: ADMIN
 */


#include "bms_telemetry.h"
#include <string.h>

#define BMS_SOF               0xAAu
#define BMS_ESP_ID            0x00u
#define BMS_SELF_ID           0x02u
#define BMS_REQ_GET          0x01u
#define BMS_RESP_TLM         0x91u
#define BMS_REQ_PAYLOAD_LEN   3u
#define BMS_RESP_PAYLOAD_LEN 21u
#define BMS_RX_MAX_PAYLOAD   32u
#define BMS_SAMPLE_TTL_MS  1500u

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static uint8_t s_rx_frame[3u + BMS_RX_MAX_PAYLOAD + 1u];
static volatile uint8_t s_rx_count;
static volatile uint8_t s_request_pending;
static volatile uint16_t s_request_seq;

static uint8_t BMS_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                                : (uint8_t)(crc << 1);
    }
    return crc;
}

static void BMS_PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

HAL_StatusTypeDef BMS_Comm_Init(UART_HandleTypeDef *uart)
{
    s_uart = uart;
    s_rx_count = 0u;
    s_request_pending = 0u;
    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

/* Only byte reception + request extraction here. TX takes place in main loop. */
void BMS_Comm_OnRxByte(UART_HandleTypeDef *uart)
{
    if (!s_uart || uart != s_uart) return;

    const uint8_t b = s_rx_byte;
    uint8_t count = s_rx_count;
    if (count == 0u) {
        if (b == BMS_SOF) s_rx_frame[count++] = b;
    } else {
        if (count >= sizeof(s_rx_frame)) count = 0u;
        if (count != 0u) s_rx_frame[count++] = b;
        if (count == 3u && s_rx_frame[2] > BMS_RX_MAX_PAYLOAD)
            count = 0u;
        if (count >= 3u && count == (uint8_t)(s_rx_frame[2] + 4u)) {
            const uint8_t n = s_rx_frame[2];
            if (s_rx_frame[1] == BMS_ESP_ID && n == BMS_REQ_PAYLOAD_LEN &&
                s_rx_frame[3] == BMS_REQ_GET &&
                BMS_Crc8(&s_rx_frame[1], (uint8_t)(n + 2u)) == s_rx_frame[n + 3u]) {
                s_request_seq = (uint16_t)s_rx_frame[4] |
                                ((uint16_t)s_rx_frame[5] << 8);
                s_request_pending = 1u;
            }
            count = 0u;
        }
    }
    s_rx_count = count;
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

void BMS_Comm_OnUartError(UART_HandleTypeDef *uart)
{
    if (!s_uart || uart != s_uart) return;
    s_rx_count = 0u;
    (void)HAL_UART_AbortReceive(s_uart);
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

void BMS_Comm_Service(const BMS_Telemetry_t *latest, uint32_t now_ms)
{
    if (!s_uart || !latest || !s_request_pending) return;
    uint16_t seq;
    __disable_irq();
    seq = s_request_seq;
    s_request_pending = 0u;
    __enable_irq();

    BMS_Telemetry_t snapshot = *latest;
    if ((now_ms - snapshot.sampled_at_ms) > BMS_SAMPLE_TTL_MS) {
        /* Never misrepresent stale sensor values as valid. */
        snapshot.flags &= (uint8_t)~(BMS_FLAG_VOLTAGE_VALID |
            BMS_FLAG_CURRENT_VALID | BMS_FLAG_TEMP_VALID | BMS_FLAG_SOC_VALID);
    }

    uint8_t frame[3u + BMS_RESP_PAYLOAD_LEN + 1u];
    uint8_t *p = &frame[3];
    frame[0] = BMS_SOF;
    frame[1] = BMS_SELF_ID;
    frame[2] = BMS_RESP_PAYLOAD_LEN;
    p[0] = BMS_RESP_TLM;
    BMS_PutU16(&p[1], seq);
    for (uint8_t i = 0u; i < 4u; ++i)
        BMS_PutU16(&p[3u + 2u * i], snapshot.cell_mV[i]);
    BMS_PutU16(&p[11], snapshot.pack_mV);
    BMS_PutU16(&p[13], (uint16_t)snapshot.current_mA);
    BMS_PutU16(&p[15], snapshot.soc_centi_pct);
    BMS_PutU16(&p[17], (uint16_t)snapshot.pack_temp_cC);
    p[19] = snapshot.output_enabled;
    p[20] = snapshot.flags;
    frame[3u + BMS_RESP_PAYLOAD_LEN] =
        BMS_Crc8(&frame[1], (uint8_t)(BMS_RESP_PAYLOAD_LEN + 2u));
    (void)HAL_UART_Transmit(s_uart, frame, sizeof(frame), 10u);
}

