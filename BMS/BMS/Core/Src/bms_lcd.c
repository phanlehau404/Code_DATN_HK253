/* bms_lcd.c
 *
 * SolGrid BMS - HD44780 20x4 LCD + PCF8574 I2C backpack.
 *
 * Revision: LCD/I2C self-recovery
 * - Retry an LCD that disappears instead of disabling it permanently.
 * - Reset/re-initialize I2C1 after repeated probe failures.
 * - Re-initialize the HD44780 periodically to recover silent 4-bit desync.
 * - Force a full redraw periodically so a lost write cannot remain forever.
 * - Keep LCD optional: BMS/UART continue even when the LCD is absent.
 */

#include "bms_lcd.h"
#include "i2c-lcd.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

/* -------------------------------------------------------------------------- */
/* LCD configuration                                                          */
/* -------------------------------------------------------------------------- */

#define LCD_ADDR_HAL                 (0x27u << 1)   /* Must match i2c-lcd.c */
#define LCD_COLS                     20u
#define LCD_ROWS                      4u

#define SPLASH_MIN_MS              3000u
#define SPLASH_ANIM_MS              250u

#define LCD_REFRESH_MS             1000u
#define LCD_PAGE_OVERVIEW_MS       6000u
#define LCD_PAGE_CELLS_MS          4000u

#define LCD_PROBE_TIMEOUT_MS          5u

/* Recovery policy. */
#define LCD_RETRY_OFFLINE_MS        1000u
#define LCD_FULL_REDRAW_MS          5000u
#define LCD_PERIODIC_RESYNC_MS     30000u
#define LCD_PROBE_FAILS_BEFORE_I2C_RESET  2u

/* -------------------------------------------------------------------------- */
/* HD44780 custom-character allocation                                        */
/* -------------------------------------------------------------------------- */

#define GLYPH_FILL_1          0u
#define GLYPH_FILL_5          4u
#define GLYPH_BOLT            5u
#define GLYPH_THERMOMETER     6u

/* -------------------------------------------------------------------------- */
/* Module state                                                               */
/* -------------------------------------------------------------------------- */

static uint8_t s_present;
static uint8_t s_splashDone;
static uint8_t s_page;              /* 0 = overview, 1 = cell voltages */
static uint8_t s_probeFailStreak;

static uint8_t s_prev[LCD_ROWS][LCD_COLS];

static uint32_t s_beginMs;
static uint32_t s_lastDrawMs;
static uint32_t s_pageStartMs;
static uint32_t s_lastRetryMs;
static uint32_t s_lastFullRedrawMs;
static uint32_t s_lastResyncMs;

/* Live Expressions / diagnostics. */
volatile uint8_t  g_bmsLcdPresent;
volatile uint32_t g_bmsLcdProbeFailCnt;
volatile uint32_t g_bmsLcdRecoverCnt;
volatile uint32_t g_bmsLcdI2cResetCnt;
volatile uint32_t g_bmsLcdPeriodicResyncCnt;

/* Forward declaration: recovery code re-installs CGRAM glyphs. */
static void lcdInstallGlyphs(void);

/* -------------------------------------------------------------------------- */
/* Recovery helpers                                                           */
/* -------------------------------------------------------------------------- */

static void lcdInvalidateShadow(void)
{
    /* 0xFF cannot be a normal ASCII/custom-glyph row value used here, so this
     * guarantees the next renderer pass writes all four rows. */
    memset(s_prev, 0xFF, sizeof(s_prev));
}

static bool lcdProbe(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c1,
                                 LCD_ADDR_HAL,
                                 1u,
                                 LCD_PROBE_TIMEOUT_MS) == HAL_OK;
}

static bool lcdResetI2cPeripheral(void)
{
    /* This runs only from main context. The BMS sensor accesses and LCD access
     * are sequential in BMS_App_Process(), so no concurrent I2C transaction is
     * aborted here. */
    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1u);
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_Delay(1u);

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        return false;

    g_bmsLcdI2cResetCnt++;
    return true;
}

static void lcdMarkOffline(uint32_t now_ms)
{
    s_present = 0u;
    g_bmsLcdPresent = 0u;
    s_lastRetryMs = now_ms;
    lcdInvalidateShadow();
}

static bool lcdControllerReinit(uint32_t now_ms, bool recovery)
{
    if (!lcdProbe())
    {
        g_bmsLcdProbeFailCnt++;
        if (s_probeFailStreak < 255u)
            s_probeFailStreak++;

        /* A transient NACK is not enough reason to disturb the shared bus.
         * Reset I2C1 only after repeated failures. */
        if (s_probeFailStreak >= LCD_PROBE_FAILS_BEFORE_I2C_RESET)
        {
            if (!lcdResetI2cPeripheral() || !lcdProbe())
            {
                lcdMarkOffline(now_ms);
                return false;
            }
        }
        else
        {
            lcdMarkOffline(now_ms);
            return false;
        }
    }

    /* Re-send the full HD44780 4-bit initialization sequence. This is the
     * important part for a display whose PCF8574 still ACKs but whose LCD
     * controller has lost nibble alignment due to EMI. */
    lcd_init();
    lcdInstallGlyphs();

    /* Confirm that the backpack is still reachable after initialization. */
    if (!lcdProbe())
    {
        g_bmsLcdProbeFailCnt++;
        if (s_probeFailStreak < 255u)
            s_probeFailStreak++;
        lcdMarkOffline(now_ms);
        return false;
    }

    s_present = 1u;
    g_bmsLcdPresent = 1u;
    s_probeFailStreak = 0u;
    s_lastResyncMs = now_ms;
    s_lastFullRedrawMs = now_ms;
    lcdInvalidateShadow();

    if (recovery)
        g_bmsLcdRecoverCnt++;

    return true;
}

static bool lcdCheckOrMarkOffline(uint32_t now_ms)
{
    if (lcdProbe())
    {
        s_probeFailStreak = 0u;
        return true;
    }

    g_bmsLcdProbeFailCnt++;
    if (s_probeFailStreak < 255u)
        s_probeFailStreak++;

    lcdMarkOffline(now_ms);
    return false;
}

/* -------------------------------------------------------------------------- */
/* Low-level custom glyph helpers                                             */
/* -------------------------------------------------------------------------- */

static void lcdWriteGlyph(uint8_t slot, const uint8_t rows[8])
{
    if ((slot > 7u) || (rows == NULL))
        return;

    lcd_send_cmd((char)(0x40u | (slot * 8u)));

    for (uint8_t i = 0u; i < 8u; ++i)
        lcd_send_data((char)(rows[i] & 0x1Fu));
}

static void lcdInstallGlyphs(void)
{
    for (uint8_t width = 1u; width <= 5u; ++width)
    {
        uint8_t rows[8];
        const uint8_t bitmap =
            (uint8_t)((0x1Fu << (5u - width)) & 0x1Fu);

        for (uint8_t y = 0u; y < 8u; ++y)
            rows[y] = bitmap;

        lcdWriteGlyph((uint8_t)(width - 1u), rows);
    }

    static const uint8_t bolt[8] = {
        0b00100,
        0b01000,
        0b10000,
        0b11111,
        0b00001,
        0b00010,
        0b00100,
        0b00000
    };

    static const uint8_t thermometer[8] = {
        0b00100,
        0b01010,
        0b01010,
        0b01010,
        0b01010,
        0b10001,
        0b11111,
        0b01110
    };

    lcdWriteGlyph(GLYPH_BOLT, bolt);
    lcdWriteGlyph(GLYPH_THERMOMETER, thermometer);
}

/* -------------------------------------------------------------------------- */
/* Row-buffer helpers                                                         */
/* -------------------------------------------------------------------------- */

static void rowBlank(uint8_t row[LCD_COLS])
{
    memset(row, ' ', LCD_COLS);
}

static void rowText(uint8_t row[LCD_COLS], uint8_t col, const char *text)
{
    if ((row == NULL) || (text == NULL) || (col >= LCD_COLS))
        return;

    while ((*text != '\0') && (col < LCD_COLS))
        row[col++] = (uint8_t)*text++;
}

static void lcdDrawRow(uint8_t index, const uint8_t row[LCD_COLS])
{
    if ((index >= LCD_ROWS) || (row == NULL))
        return;

    if (memcmp(s_prev[index], row, LCD_COLS) == 0)
        return;

    lcd_goto_XY((int)index + 1, 0);

    /* Do not use lcd_send_string() here because CGRAM slot 0 is byte 0x00. */
    for (uint8_t col = 0u; col < LCD_COLS; ++col)
        lcd_send_data((char)row[col]);

    memcpy(s_prev[index], row, LCD_COLS);
}

/* -------------------------------------------------------------------------- */
/* Splash screen                                                              */
/* -------------------------------------------------------------------------- */

static void lcdDrawSplash(uint32_t elapsedMs)
{
    uint8_t row[LCD_COLS];

    uint8_t blocks = (uint8_t)(elapsedMs / 300u);
    if (blocks > 10u)
        blocks = 10u;

    rowBlank(row);
    rowText(row, 0u, "+------------------+");
    lcdDrawRow(0u, row);

    rowBlank(row);
    row[5] = GLYPH_BOLT;
    rowText(row, 7u, "SolGrid");
    lcdDrawRow(1u, row);

    rowBlank(row);
    rowText(row, 2u, "SMART ENERGY BMS");
    lcdDrawRow(2u, row);

    rowBlank(row);
    rowText(row, 0u, "BOOT [");

    for (uint8_t i = 0u; i < 10u; ++i)
        row[6u + i] = (uint8_t)((i < blocks) ? '#' : '-');

    row[16] = ']';
    lcdDrawRow(3u, row);
}

/* -------------------------------------------------------------------------- */
/* SOC bar                                                                    */
/* -------------------------------------------------------------------------- */

static void rowSoc(uint8_t row[LCD_COLS], const BMS_Telemetry_t *t)
{
    rowBlank(row);

    rowText(row, 0u, "SOC [");
    row[14] = ']';
    row[15] = ' ';

    if (((t->flags & BMS_FLAG_SOC_VALID) == 0u) ||
        (t->soc_centi_pct > 10000u))
    {
        rowText(row, 16u, "---%");
        return;
    }

    uint16_t pct =
        (uint16_t)((t->soc_centi_pct + 50u) / 100u);

    uint16_t pixels =
        (uint16_t)(((uint32_t)t->soc_centi_pct * 45u + 5000u) /
                   10000u);

    if (pixels > 45u)
        pixels = 45u;

    for (uint8_t c = 0u; c < 9u; ++c)
    {
        uint8_t fill = 0u;
        const uint16_t firstPixel = (uint16_t)c * 5u;

        if (pixels > firstPixel)
        {
            uint16_t available = (uint16_t)(pixels - firstPixel);
            fill = (uint8_t)((available > 5u) ? 5u : available);
        }

        row[5u + c] =
            (fill == 0u)
                ? (uint8_t)' '
                : (uint8_t)(GLYPH_FILL_1 + fill - 1u);
    }

    char text[8];
    (void)snprintf(text, sizeof(text), "%3u%%", (unsigned)pct);
    rowText(row, 16u, text);
}

/* -------------------------------------------------------------------------- */
/* Temperature formatting                                                     */
/* -------------------------------------------------------------------------- */

static void tempText(char text[12], const BMS_Telemetry_t *t)
{
    if ((t->flags & BMS_FLAG_TEMP_VALID) == 0u)
    {
        (void)snprintf(text, 12u, "--.-C");
        return;
    }

    int32_t cc = (int32_t)t->pack_temp_cC;
    const char *sign = (cc < 0) ? "-" : "";

    if (cc < 0)
        cc = -cc;

    uint32_t deci = (uint32_t)(cc + 5) / 10u;

    (void)snprintf(text,
                   12u,
                   "%s%lu.%luC",
                   sign,
                   (unsigned long)(deci / 10u),
                   (unsigned long)(deci % 10u));
}

/* -------------------------------------------------------------------------- */
/* Overview page                                                              */
/* -------------------------------------------------------------------------- */

static void rowHeaderOverview(uint8_t row[LCD_COLS],
                              const BMS_Telemetry_t *t)
{
    rowBlank(row);

    row[0] = GLYPH_BOLT;
    rowText(row, 2u, "SolGrid BMS");

    if (t->flags & BMS_FLAG_FAULT_LATCHED)
        rowText(row, 16u, "FLT!");
    else
        rowText(row, 16u, t->output_enabled ? "ON  " : "OFF ");
}

static void rowOverviewPower(uint8_t row[LCD_COLS],
                             const BMS_Telemetry_t *t)
{
    rowBlank(row);

    char text[22];

    if (t->flags & BMS_FLAG_VOLTAGE_VALID)
    {
        (void)snprintf(text,
                       sizeof(text),
                       "V:%lu.%02luV",
                       (unsigned long)(t->pack_mV / 1000u),
                       (unsigned long)((t->pack_mV % 1000u) / 10u));
        rowText(row, 0u, text);
    }
    else
    {
        rowText(row, 0u, "V:--.--V");
    }

    if (t->flags & BMS_FLAG_CURRENT_VALID)
    {
        int32_t mA = (int32_t)t->current_mA;
        const char sign = (mA < 0) ? '-' : '+';

        if (mA < 0)
            mA = -mA;

        (void)snprintf(text,
                       sizeof(text),
                       "I:%c%lu.%02luA",
                       sign,
                       (unsigned long)(mA / 1000),
                       (unsigned long)((mA % 1000) / 10));

        rowText(row, 10u, text);
    }
    else
    {
        rowText(row, 10u, "I:--.--A");
    }
}

static void rowOverviewTemp(uint8_t row[LCD_COLS],
                            const BMS_Telemetry_t *t)
{
    char text[12];

    rowBlank(row);

    row[0] = GLYPH_THERMOMETER;
    tempText(text, t);
    rowText(row, 2u, text);

    rowText(row,
            12u,
            t->output_enabled ? "OUT:ON" : "OUT:OFF");
}

/* -------------------------------------------------------------------------- */
/* Cell-voltage page                                                          */
/* -------------------------------------------------------------------------- */

static void rowHeaderCells(uint8_t row[LCD_COLS],
                           const BMS_Telemetry_t *t)
{
    char text[12];

    rowBlank(row);

    row[0] = GLYPH_THERMOMETER;
    tempText(text, t);
    rowText(row, 2u, text);

    rowText(row, 11u, "CELLS");

    if (t->flags & BMS_FLAG_FAULT_LATCHED)
        rowText(row, 17u, "FLT");
    else if (t->flags & BMS_FLAG_VOLTAGE_VALID)
        rowText(row, 17u, "OK ");
    else
        rowText(row, 17u, "ERR");
}

static void rowTwoCells(uint8_t row[LCD_COLS],
                        const BMS_Telemetry_t *t,
                        uint8_t firstIndex)
{
    rowBlank(row);

    for (uint8_t n = 0u; n < 2u; ++n)
    {
        const uint8_t index = (uint8_t)(firstIndex + n);
        const uint8_t col = (n == 0u) ? 0u : 10u;

        char text[14];

        if (t->flags & BMS_FLAG_VOLTAGE_VALID)
        {
            const uint16_t value = t->cell_mV[index];

            (void)snprintf(text,
                           sizeof(text),
                           "C%u:%lu.%02luV",
                           (unsigned)(index + 1u),
                           (unsigned long)(value / 1000u),
                           (unsigned long)((value % 1000u) / 10u));
        }
        else
        {
            (void)snprintf(text,
                           sizeof(text),
                           "C%u:--.--V",
                           (unsigned)(index + 1u));
        }

        rowText(row, col, text);
    }
}

/* -------------------------------------------------------------------------- */
/* Normal-page renderer                                                       */
/* -------------------------------------------------------------------------- */

static void lcdDrawNormal(const BMS_Telemetry_t *t)
{
    uint8_t row[LCD_COLS];

    if (s_page == 0u)
        rowHeaderOverview(row, t);
    else
        rowHeaderCells(row, t);
    lcdDrawRow(0u, row);

    rowSoc(row, t);
    lcdDrawRow(1u, row);

    if (s_page == 0u)
        rowOverviewPower(row, t);
    else
        rowTwoCells(row, t, 0u);
    lcdDrawRow(2u, row);

    if (s_page == 0u)
        rowOverviewTemp(row, t);
    else
        rowTwoCells(row, t, 2u);
    lcdDrawRow(3u, row);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void BMS_LCD_Init(uint32_t now_ms)
{
    s_present = 0u;
    s_splashDone = 0u;
    s_page = 0u;
    s_probeFailStreak = 0u;

    s_beginMs = now_ms;
    s_lastDrawMs = now_ms - SPLASH_ANIM_MS;
    s_pageStartMs = now_ms;
    s_lastRetryMs = now_ms - LCD_RETRY_OFFLINE_MS;
    s_lastFullRedrawMs = now_ms;
    s_lastResyncMs = now_ms;

    g_bmsLcdPresent = 0u;
    g_bmsLcdProbeFailCnt = 0u;
    g_bmsLcdRecoverCnt = 0u;
    g_bmsLcdI2cResetCnt = 0u;
    g_bmsLcdPeriodicResyncCnt = 0u;

    lcdInvalidateShadow();

    /* LCD is optional. A missing LCD no longer becomes a permanent state;
     * BMS_LCD_Process() retries it every LCD_RETRY_OFFLINE_MS. */
    if (!lcdControllerReinit(now_ms, false))
        return;

    lcdDrawSplash(0u);

    s_beginMs = HAL_GetTick();
    s_lastDrawMs = s_beginMs;
}

void BMS_LCD_Process(const BMS_Telemetry_t *telemetry,
                     uint32_t now_ms,
                     bool application_ready)
{
    if (telemetry == NULL)
        return;

    /* ---------------------------------------------------------------------- */
    /* Offline/recovery state                                                 */
    /* ---------------------------------------------------------------------- */

    if (!s_present)
    {
        if ((uint32_t)(now_ms - s_lastRetryMs) < LCD_RETRY_OFFLINE_MS)
            return;

        s_lastRetryMs = now_ms;

        if (!lcdControllerReinit(now_ms, true))
            return;

        /* Do not require a MCU reset after recovery. Rebuild whatever page
         * should currently be visible. */
        if (!s_splashDone)
        {
            lcdDrawSplash(now_ms - s_beginMs);
            s_lastDrawMs = now_ms;
        }
        else
        {
            lcdDrawNormal(telemetry);
            s_lastDrawMs = now_ms;
        }

        return;
    }

    /* ---------------------------------------------------------------------- */
    /* Periodic HD44780 resynchronization                                     */
    /* ---------------------------------------------------------------------- */

    if ((uint32_t)(now_ms - s_lastResyncMs) >= LCD_PERIODIC_RESYNC_MS)
    {
        /* PCF8574 can continue ACKing even when the HD44780 has lost 4-bit
         * nibble alignment. A complete lcd_init() is therefore used as a
         * preventive software resynchronization. */
        if (!lcdControllerReinit(now_ms, true))
            return;

        g_bmsLcdPeriodicResyncCnt++;

        if (s_splashDone)
            lcdDrawNormal(telemetry);
        else
            lcdDrawSplash(now_ms - s_beginMs);

        s_lastDrawMs = now_ms;
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* Startup splash                                                         */
    /* ---------------------------------------------------------------------- */

    if (!s_splashDone)
    {
        const bool minTimeElapsed =
            ((uint32_t)(now_ms - s_beginMs) >= SPLASH_MIN_MS);

        if ((!minTimeElapsed) || (!application_ready))
        {
            if ((uint32_t)(now_ms - s_lastDrawMs) >= SPLASH_ANIM_MS)
            {
                s_lastDrawMs = now_ms;

                if (!lcdCheckOrMarkOffline(now_ms))
                    return;

                lcdDrawSplash(now_ms - s_beginMs);
            }

            return;
        }

        s_splashDone = 1u;
        s_lastDrawMs = now_ms - LCD_REFRESH_MS;
        s_pageStartMs = now_ms;
        lcdInvalidateShadow();
    }

    /* ---------------------------------------------------------------------- */
    /* Automatic page switching                                               */
    /* ---------------------------------------------------------------------- */

    const uint32_t pageLimit =
        (s_page == 0u)
            ? LCD_PAGE_OVERVIEW_MS
            : LCD_PAGE_CELLS_MS;

    if ((uint32_t)(now_ms - s_pageStartMs) >= pageLimit)
    {
        s_page ^= 1u;
        s_pageStartMs = now_ms;
        s_lastDrawMs = now_ms - LCD_REFRESH_MS;
        lcdInvalidateShadow();
    }

    /* Even if the displayed values have not changed, periodically rewrite the
     * entire DDRAM image. This repairs a lost data write without waiting for a
     * value to change. No lcd_clear_display() is used, so flicker stays low. */
    if ((uint32_t)(now_ms - s_lastFullRedrawMs) >= LCD_FULL_REDRAW_MS)
    {
        s_lastFullRedrawMs = now_ms;
        lcdInvalidateShadow();
    }

    /* ---------------------------------------------------------------------- */
    /* Periodic display refresh                                               */
    /* ---------------------------------------------------------------------- */

    if ((uint32_t)(now_ms - s_lastDrawMs) < LCD_REFRESH_MS)
        return;

    s_lastDrawMs = now_ms;

    if (!lcdCheckOrMarkOffline(now_ms))
        return;

    lcdDrawNormal(telemetry);
}
