/*
 * SolGrid — ESP32-S3 telemetry publisher + battery protection (AWS IoT Core),
 * thiết bị Trạm 02.
 *
 * Kết nối WiFi → TLS tới AWS IoT Core bằng X.509 cert của thiết bị → publish
 * JSON telemetry lên topic solgrid/<station>/telemetry theo chu kỳ, đồng thời
 * subscribe solgrid/<thing_name>/command để nhận cấu hình ngưỡng bảo vệ pin
 * ({type:"battery_config"}, xem supabase/functions/send-battery-config).
 * AWS IoT Rule chuyển tiếp telemetry về Supabase Edge Function (xem docs/IOT.md).
 *
 * NGUỒN SỐ LIỆU PV (UART ↔ STM32F103C8T6): ESP32 vẫn hỏi kit STM32F103C8T6 qua
 * Serial1 (115200 8N1, khung SOF/ID/LEN/PAYLOAD/CRC8) mỗi 2 s để giữ chẩn đoán
 * đường truyền và các số đo vin/iin/vout/iout/duty. Riêng solar_kw đang dùng số
 * mô phỏng 5–20 W phục vụ demo. Dữ liệu pin/SOC/nhiệt độ được lấy từ STM32F103
 * điều khiển BMS qua UART2 riêng.
 *
 * Đấu dây (chung GND, mức logic 3.3 V):
 *   Buck-boost: ESP32 GPIO17 (TX1) ── PB11 (USART3_RX) STM32 buck-boost
 *               ESP32 GPIO18 (RX1) ── PB10 (USART3_TX) STM32 buck-boost
 *   BMS:        ESP32 GPIO15 - D15 (TX2) ── PA3  (USART2_RX) STM32 BMS
 *               ESP32 GPIO16 - D16 (RX2) ── PA2  (USART2_TX) STM32 BMS
 *   GND ───────────────────────────── GND chung
 *
 * ĐIỂM PHÁT WIFI CỤC BỘ (SoftAP): thiết bị chạy WIFI_AP_STA — vừa nối uplink,
 * vừa tự phát mạng AP_SSID/AP_PASSWORD (secrets.h) để truy cập tại chỗ qua
 * http://192.168.4.1 khi trạm mất internet. Thiết bị là NGUỒN SỰ THẬT của cấu
 * hình này: nó báo SSID/mật khẩu lên cloud qua telemetry (bản tin đầu sau mỗi
 * lần reconnect) và dashboard chỉ hiển thị, không sửa được — muốn đổi thì sửa
 * secrets.h rồi nạp lại. Xem supabase/migrations/0014_ap_reported_by_device.sql.
 *
 * BẢO VỆ SẠC/XẢ: giống bản esp32-solgrid — applyProtection() tự đóng/cắt relay
 * đường sạc/xả theo ngưỡng SOC/điện áp có hysteresis, chạy tại thiết bị nên an
 * toàn cả khi mất mạng; ngưỡng nhận từ cloud được lưu NVS để giữ qua reboot.
 * Bản S3 này KHÔNG điều khiển tải người dùng (không có bảng relays[]) — nếu cần
 * thêm, tham khảo firmware/esp32-solgrid/esp32-solgrid.ino.
 *
 * CẬP NHẬT QUA MẠNG (OTA): dùng AWS IoT Jobs để nhận presigned URL Amazon S3 +
 * SHA-256, tải qua HTTPS, kiểm hash rồi mới kích hoạt phân vùng OTA và reboot.
 * Lệnh {type:"ota"} cũ vẫn được hỗ trợ. Báo `fw_version`, `fw_status` và
 * `fw_progress` (0..100) lên cloud, đồng thời cập nhật trạng thái AWS Job.
 *
 * BẮT BUỘC trước khi dùng OTA:
 *   1. Tools → Partition Scheme: chọn sơ đồ CÓ HAI phân vùng app ("Default 4MB
 *      with spiffs", "Minimal SPIFFS", "8M with spiffs"...). Sơ đồ "Huge APP"
 *      chỉ có MỘT app partition → Update.begin() luôn thất bại.
 *   2. Firmware trên S3 phải build cho đúng FW_BOARD; AWS job document phải có
 *      operation, board, version, url, sha256 và size.
 *   3. Tăng FW_VERSION trong secrets.h mỗi lần build một bản mới — thiết bị dùng
 *      chuỗi này để biết mình đã ở đúng bản chưa và bỏ qua lệnh đẩy lặp lại.
 *
 * Board (Arduino IDE): "ESP32S3 Dev Module" — cùng core ESP32 nên
 * WiFiClientSecure/PubSubClient/ArduinoJson/Preferences chạy y hệt bản ESP32
 * thường. Lưu ý riêng cho S3:
 *   - Nếu board dùng cổng USB-native mà Serial Monitor không hiện gì, bật
 *     Tools → "USB CDC On Boot: Enabled".
 *   - Một số board S3 cần "USB Mode" = "Hardware CDC and JTAG".
 *   - Serial1 (link STM32) độc lập với Serial (log USB), không đụng nhau.
 *
 * Thư viện (Library Manager): PubSubClient (Nick O'Leary), ArduinoJson
 * (Benoit Blanchon).
 *
 * Trước khi nạp: thiết bị PHẢI đã tồn tại trong Supabase (bảng `devices`,
 * gắn với Trạm 01) và đã có cert AWS IoT — xem docs/IOT.md mục 2 và 4.2.
 * Điền secrets.h (xem secrets.example.h) — KHÔNG commit cert thật.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
// Topic + payload JSON (kèm trạng thái bảo vệ + số liệu PV) có thể vượt 256B
// mặc định của PubSubClient — đặt trước khi include.
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <math.h>
#include <time.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include "mbedtls/version.h"
#include "secrets.h"

#ifndef FW_VERSION
  #error "Define FW_VERSION in secrets.h, for example: #define FW_VERSION \"v2.7.1\""
#endif

#ifndef FW_BOARD
  #error "Define FW_BOARD in secrets.h, for example: #define FW_BOARD \"esp32s3-solgrid\""
#endif

// mbedtls 3.x (Arduino-ESP32 core 3.x / IDF 5.x) bỏ hậu tố _ret của 2.x. Bọc
// lại để sketch biên dịch được trên cả hai core.
#if MBEDTLS_VERSION_MAJOR >= 3
  #define SHA256_STARTS(ctx)            mbedtls_sha256_starts(ctx, 0)
  #define SHA256_UPDATE(ctx, buf, len)  mbedtls_sha256_update(ctx, buf, len)
  #define SHA256_FINISH(ctx, out)       mbedtls_sha256_finish(ctx, out)
#else
  #define SHA256_STARTS(ctx)            mbedtls_sha256_starts_ret(ctx, 0)
  #define SHA256_UPDATE(ctx, buf, len)  mbedtls_sha256_update_ret(ctx, buf, len)
  #define SHA256_FINISH(ctx, out)       mbedtls_sha256_finish_ret(ctx, out)
#endif

// Thing name = MQTT client id = devices.aws_thing_name trong Supabase.
static const char *THING_NAME = AWS_THING_NAME;
static const char *STATION_SLUG = STATION_SLUG_CFG;   // "tram01" — chỉ dùng để dựng topic
static const uint16_t AWS_IOT_PORT = 8883;
static const unsigned long PUBLISH_INTERVAL_MS = 10000;
static const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
static const unsigned long WIFI_RECONFIGURE_DELAY_MS = 300;
static const time_t TLS_MIN_VALID_EPOCH = 1704067200; // 2024-01-01 UTC

// ---------------------------------------------------------------------
// Định danh bản build. FW_VERSION được đặt trong secrets.h và phải TRÙNG `version` của hàng
// firmware_releases khi tải bản này lên dashboard — cloud so hai chuỗi để biết
// thiết bị đã nạp xong hay chưa (xem ingest-telemetry mục 8).
// FW_BOARD phải trùng `board` của bản phát hành: ảnh build cho board khác có
// thể nạp lọt nhưng không boot được, nên đây là chốt chặn bắt buộc — cloud
// KHÔNG kiểm được việc này, chỉ thiết bị mới biết chắc nó là board gì.
// ---------------------------------------------------------------------
// URL OTA do Supabase Storage cấp, vì vậy TUYỆT ĐỐI không dùng AWS_ROOT_CA để
// xác thực URL này. Nếu secrets.h có OTA_DOWNLOAD_ROOT_CA, client sẽ kiểm tra
// chuỗi TLS bằng CA đó. Nếu không, transport TLS không ghim CA; tính toàn vẹn
// và xác thực ảnh vẫn được chốt bằng SHA-256 nhận qua AWS IoT mutual-TLS trước
// khi Update.end() cho phép boot ảnh mới. Cách này cũng chịu được redirect sang
// host Storage có chuỗi CA khác.

static const size_t OTA_URL_MAX = 1024;       // presigned S3 URL can be long
static const size_t OTA_CHUNK = 1024;         // buffer đọc từ stream HTTPS
static const unsigned long OTA_STALL_MS = 20000;  // im lặng quá lâu → coi như đứt
static const size_t AWS_JOB_ID_MAX = 65;
// Không publish theo từng chunk: ảnh vài MB sẽ tạo hàng nghìn bản tin. Báo khi
// phần trăm nguyên thay đổi nhưng không nhanh hơn nhịp này; 100% luôn được ép gửi.
static const unsigned long OTA_PROGRESS_INTERVAL_MS = 1000;

// Lệnh OTA nhận được, chờ CloudTask xử lý. KHÔNG tải ngay trong callback MQTT:
// callback chạy bên trong mqtt.loop(), một lần tải vài MB sẽ chặn chính callback và
// làm đứt phiên MQTT giữa chừng.
//
// Khai ở đây, TRƯỚC hàm đầu tiên của sketch, vì runOtaUpdate() nhận nó làm
// tham số — xem chú thích cùng lý do ở struct BatteryReading.
struct OtaJob {
  bool pending = false;
  char url[OTA_URL_MAX];
  char sha256[65];
  char version[40];
  size_t size = 0;
  char awsJobId[AWS_JOB_ID_MAX] = "";
};
OtaJob otaJob;

struct AwsJobStatusUpdate {
  bool pending = false;
  char jobId[AWS_JOB_ID_MAX] = "";
  char status[16] = "";
  char detail[48] = "";
  int progress = -1;
};

AwsJobStatusUpdate awsJobStatus;
bool awsStartNextRequested = false;

// Ghim đích OTA qua lần reboot. Nếu chỉ dựa vào RAM thì trạng thái cuối cùng
// của bản cũ là "applying"; bản mới không còn biết job/version nào vừa đưa nó
// lên để xác nhận thành công. Marker nhỏ này được xoá chỉ sau khi telemetry và
// AWS Job status đã publish thành công trên bản firmware mới.
static const char *OTA_PREF_NAMESPACE = "ota-state";
struct OtaCompletionMarker {
  bool pending = false;
  char version[40] = "";
  char awsJobId[AWS_JOB_ID_MAX] = "";
};
OtaCompletionMarker otaCompletion;
bool otaCompletionReporting = false;

// ---------------------------------------------------------------------
// Link UART tới STM32F103C8T6 (USART3 phía STM32, Serial1 phía ESP32).
//
// Khung tin (giống hệt phía STM32):
//   [0]=SOF 0xAA  [1]=ID người gửi  [2]=LEN payload  [3..]=payload
//   [3+LEN]=CRC8 tính trên ID+LEN+payload (LEN+2 byte), poly 0x07, init 0x00
// ESP32 là master ID 0x00, STM32 là node ID 0x01 (STM32 tự bỏ qua khung mang
// ID của chính nó nên hai đầu không tự lặp).
//
// ESP32 là bên CHỦ ĐỘNG hỏi: STM32 không tự đẩy, nên mất khung chỉ mất đúng
// một chu kỳ chứ không cần đồng bộ lại trạng thái.
// ---------------------------------------------------------------------
static const int BUCKBOOST_UART_TX_PIN = 17;   // ESP32 TX -> STM32 PB11 (USART3_RX)
static const int BUCKBOOST_UART_RX_PIN = 18;   // ESP32 RX <- STM32 PB10 (USART3_TX)
static const uint32_t BUCKBOOST_UART_BAUD = 115200;

static const uint8_t FRAME_SOF = 0xAA;
static const uint8_t MASTER_ID = 0x00;
static const uint8_t NODE_ID = 0x01;
static const uint8_t CMD_GET_TLM = 0x01;
static const uint8_t MAX_PAYLOAD = 32;

// 2 s: nhanh hơn chu kỳ publish (10 s) để mỗi bản tin mang số vừa đo, nhưng
// không nhanh tới mức chiếm BuckboostTask — mỗi lần hỏi chặn tối đa
// BUCKBOOST_REPLY_TIMEOUT_MS.
static const unsigned long BUCKBOOST_POLL_INTERVAL_MS = 2000;
static const unsigned long BUCKBOOST_REPLY_TIMEOUT_MS = 150;    // chờ trả lời
static const unsigned long BUCKBOOST_STALE_MS = 60000;          // quá hạn -> coi mất link


// ---------------------------------------------------------------------
// Link UART tới STM32F103C8T6 điều khiển BMS (USART2 phía STM32, Serial2 ESP32).
// Protocol khớp bms_telemetry.c:
//   Request : [AA][00][03][01][seqLo][seqHi][CRC8]
//   Response: [AA][02][21][91][seqLo][seqHi]...[flags][CRC8]
// Một phản hồi chỉ được coi là kết nối thành công khi SOF/ID/LEN/CRC/type/seq
// đều hợp lệ. Vì vậy thông báo "Connected successfully" phản ánh đúng việc
// hai MCU đã trao đổi protocol, không chỉ việc UART đã được begin().
// ---------------------------------------------------------------------
static const int BMS_UART_TX_PIN = 15;        // ESP32 TX2 -> STM32 PA3  (USART2_RX)
static const int BMS_UART_RX_PIN = 16;        // ESP32 RX2 <- STM32 PA2  (USART2_TX)
static const uint32_t BMS_UART_BAUD = 115200;
static const uint8_t BMS_NODE_ID = 0x02;
static const uint8_t MSG_GET_BMS = 0x01;
static const uint8_t MSG_BMS_RESULT = 0x91;
static const uint8_t BMS_RESULT_LEN = 21;
static const unsigned long BMS_POLL_INTERVAL_MS = 1000;
static const unsigned long BMS_REPLY_TIMEOUT_MS = 220;
static const unsigned long BMS_STALE_MS = 3000;
static const unsigned long BMS_STATUS_REMINDER_MS = 10000;
static const unsigned long BMS_TELEMETRY_LOG_MS = 5000;

static const uint8_t BMS_FLAG_VOLTAGE_VALID = 0x01;
static const uint8_t BMS_FLAG_TEMP_VALID = 0x02;
static const uint8_t BMS_FLAG_SOC_VALID = 0x04;
static const uint8_t BMS_FLAG_CURRENT_VALID = 0x08;
static const uint8_t BMS_FLAG_FAULT_LATCHED = 0x10;

struct BmsTelemetry {
  uint16_t cell_mV[4] = {0, 0, 0, 0};
  uint16_t pack_mV = 0;
  int16_t current_mA = 0;        // STM32 convention: +discharge, -charge
  uint16_t soc_centi_pct = 0;
  int16_t pack_temp_cC = 0;
  uint8_t output_enabled = 0;
  uint8_t flags = 0;
  uint32_t received_ms = 0;
  bool connected = false;
};

BmsTelemetry bmsTlm;
uint32_t bmsFrameOk = 0;
uint32_t bmsFrameFail = 0;
uint16_t bmsSequence = 0;
unsigned long lastBmsStatusReminder = 0;
unsigned long lastBmsTelemetryLog = 0;
static portMUX_TYPE g_bmsTelemetryMux = portMUX_INITIALIZER_UNLOCKED;

// Bố cục PHẢI khớp Telemetry_t bên STM32 (cả hai đều little-endian).
struct __attribute__((packed)) BuckboostTelemetry {
  uint16_t vin_mV;        // điện áp tấm pin
  uint16_t iin_mA;        // dòng tấm pin
  uint16_t vout_mV;       // điện áp bus ắc quy
  uint16_t iout_mA;       // dòng ra bộ chuyển đổi
  uint16_t duty_permil;   // duty cycle, phần nghìn
};

BuckboostTelemetry buckboostTlm = {0, 0, 0, 0, 0};
bool buckboostLinkOk = false;
float solarKw = 0.0f;
unsigned long lastBuckboostOk = 0;

// ---------------------------------------------------------------------
// Bảo vệ sạc/xả pin (xem chú thích chi tiết ở esp32-solgrid.ino). Đặt = 255
// để vô hiệu một đường. Mức HIGH = cho phép, LOW = ngắt.
// ---------------------------------------------------------------------
static const uint8_t CHARGE_RELAY_PIN = 4;
static const uint8_t DISCHARGE_RELAY_PIN = 5;

static const float SOC_CHARGE_RESUME_MARGIN = 3.0f;
static const float SOC_DISCHARGE_RESUME_MARGIN = 5.0f;
static const float VOLT_CHARGE_RESUME_MARGIN = 0.4f;
// Safe charge-stop window for the 12 V pack. A threshold outside it (e.g. a
// leftover 48 V config) would never be reached, so charging would never stop
// on voltage. Keep in sync with PACK_MAX_VOLTAGE_* in src/pages/DevConsole.jsx.
static const float PACK_MAX_VOLTAGE_MIN = 12.0f;
static const float PACK_MAX_VOLTAGE_MAX = 15.0f;
static const float CURRENT_CHARGE_RESUME_MARGIN = 2.0f;

struct BatteryConfig {
  float minSoc = 20.0f;
  float maxSoc = 90.0f;
  float maxVoltage = 14.4f;
  float maxCurrent = 25.0f;
  bool deepDischargeProtect = true;
};
BatteryConfig batteryCfg;

static const size_t CONFIG_ID_MAX = 40;
static const size_t CONFIG_HASH_MAX = 65;

struct BmsConfigAckSnapshot {
  bool pending = false;
  char configId[CONFIG_ID_MAX] = "";
  uint64_t configVersion = 0;
  char configHash[CONFIG_HASH_MAX] = "";
  char status[16] = "";
  char detail[64] = "";
};

BmsConfigAckSnapshot bmsConfigAck;
static portMUX_TYPE g_bmsConfigAckMux = portMUX_INITIALIZER_UNLOCKED;

// PHẢI khai trước hàm ĐẦU TIÊN của sketch. Arduino tự sinh prototype cho mọi
// hàm và chèn tất cả ngay trước hàm đầu tiên trong file — kiểu tự định nghĩa
// mà nằm sau chỗ đó thì prototype `BatteryReading readBattery();` xuất hiện
// trước cả định nghĩa struct và build hỏng với "does not name a type".
struct BatteryReading {
  float soc;      // %
  float voltage;  // V
  float current;  // A (dương = nạp, âm = xả)
  float tempC;    // °C
};

BatteryReading lastBatteryReading = {0.0f, 0.0f, 0.0f, 0.0f};
bool chargeEnabled = true;
bool dischargeEnabled = true;
const char *protectReason = "ok";

// Giới hạn WPA2 — bắt sai lúc biên dịch thay vì để WiFi.softAP() lặng lẽ từ
// chối ngoài hiện trường. Dashboard chỉ hiển thị chứ không validate được nữa
// (thiết bị là nguồn sự thật), nên đây là chốt chặn duy nhất.
// sizeof(literal) - 1 = số byte thực, đúng cả khi SSID có dấu tiếng Việt.
static_assert(sizeof(AP_SSID) - 1 > 0, "AP_SSID không được để trống");
static_assert(sizeof(AP_SSID) - 1 <= 32, "AP_SSID tối đa 32 byte");
static_assert(sizeof(AP_PASSWORD) - 1 >= 8, "AP_PASSWORD tối thiểu 8 ký tự");
static_assert(sizeof(AP_PASSWORD) - 1 <= 63, "AP_PASSWORD tối đa 63 ký tự");

Preferences prefs;

// Wi-Fi uplink is provisioned by the local portal and persisted in the ESP32
// NVS flash. It is intentionally separate from AP_SSID/AP_PASSWORD: changing
// the Internet router must never make the device's local setup AP disappear.
static const char *WIFI_PREF_NAMESPACE = "wifi_cfg";
static const char *WIFI_PREF_SSID = "ssid";
static const char *WIFI_PREF_PASSWORD = "password";

struct WiFiCredentials {
  char ssid[33] = "";       // IEEE 802.11 SSID: at most 32 bytes
  char password[64] = "";   // WPA2/WPA3 passphrase: 8..63 bytes
  bool configured = false;
};

WiFiCredentials wifiCredentials;
static portMUX_TYPE g_wifiConfigMux = portMUX_INITIALIZER_UNLOCKED;
bool wifiReconfigureRequested = false;
bool wifiConnectScheduled = false;  // owned by CloudTask
unsigned long wifiConnectAt = 0;    // owned by CloudTask
bool networkTimeSyncStarted = false;
unsigned long lastTimeWaitLog = 0;

WiFiClientSecure net;
PubSubClient mqtt(net);
WebServer localServer(80);
DNSServer captiveDns;
// false = bản tin telemetry kế tiếp sẽ kèm ap_ssid/ap_password. Đặt lại false
// mỗi lần kết nối lại AWS (xem connectAWS) để cloud luôn có giá trị mới nhất.
bool apReported = false;
// Cùng cơ chế với apReported: báo phiên bản đang chạy ở bản tin đầu sau mỗi lần
// (re)connect. Cloud chỉ ghi khi giá trị đổi nên báo lặp là vô ích.
bool fwReported = false;
unsigned long lastPublish = 0;
unsigned long lastMqttAttempt = 0;
bool wifiWasConnected = false;


/* =========================================================
 *  FREERTOS APPLICATION ARCHITECTURE
 * =========================================================
 * Current responsibilities are separated without growing loop(). The two STM32
 * controllers use independent UART links: Serial1 for buck-boost, Serial2 for BMS.
 *
 * Core 0: network-facing work
 *   - CloudTask       : Wi-Fi STA + AWS MQTT + telemetry + OTA dispatch
 *   - LocalServerTask : SoftAP captive DNS + HTTP dashboard/API
 *
 * Core 1: device/control-facing work
 *   - BuckboostTask : STM32F103C8T6 buck-boost/MPPT UART transaction
 *   - BMSTask       : STM32F103 BMS UART + battery protection
 *
 * MQTT ownership rule: ONLY CloudTask may call PubSubClient APIs.  This is
 * important because PubSubClient is not designed for concurrent access.
 */
static const BaseType_t APP_NETWORK_CORE = 0;
static const BaseType_t APP_DEVICE_CORE  = 1;

static const UBaseType_t CLOUD_TASK_PRIORITY       = 2u;
static const UBaseType_t LOCAL_TASK_PRIORITY       = 2u;
static const UBaseType_t BUCKBOOST_TASK_PRIORITY = 4u;
static const UBaseType_t BMS_TASK_PRIORITY       = 3u;

// 16 KB: CloudTask chạy cả TLS handshake của MQTT lẫn HTTPS OTA, rồi giữa vòng
// tải còn gọi publishTelemetry() (~3 KB JSON + payload trên stack). 8 KB tràn
// ngay lần báo tiến trình OTA đầu tiên → Guru Meditation "Double exception".
static const uint32_t CLOUD_TASK_STACK_BYTES       = 16384u;
static const uint32_t LOCAL_TASK_STACK_BYTES       = 6144u;
static const uint32_t BUCKBOOST_TASK_STACK_BYTES = 4096u;
static const uint32_t BMS_TASK_STACK_BYTES       = 4096u;

static const TickType_t CLOUD_TASK_IDLE_TICKS = pdMS_TO_TICKS(20u);
static const TickType_t LOCAL_TASK_IDLE_TICKS = pdMS_TO_TICKS(5u);

static TaskHandle_t g_cloudTaskHandle = nullptr;
static TaskHandle_t g_localTaskHandle = nullptr;
static TaskHandle_t g_buckboostTaskHandle = nullptr;
static TaskHandle_t g_bmsTaskHandle = nullptr;

/* Short cross-core state snapshots use spinlocks instead of long mutex holds. */
static portMUX_TYPE g_pvStateMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_batteryStateMux = portMUX_INITIALIZER_UNLOCKED;

/* Local HTTP never touches the PubSubClient object directly. */
static volatile bool g_mqttOnline = false;

// --- Chẩn đoán phần cứng (migration 0017, hiện ở DevConsole → Tổng quan
// thiết bị). Gửi kèm MỌI bản tin telemetry, khác ap_ssid/fw_version vốn chỉ
// gửi sau mỗi lần reconnect: mấy giá trị này đổi liên tục, đó chính là điểm
// khiến chúng đáng theo dõi.

// Số lần boot, giữ trong NVS nên sống qua mất điện. Chuỗi reboot dồn dập là
// dấu hiệu brownout/watchdog — không suy ra được từ uptime vì uptime reset
// theo đúng những lần đó.
uint32_t bootCount = 0;

// Uptime dùng esp_timer_get_time() 64-bit để an toàn khi chạy dài ngày.
uint64_t uptimeSeconds() {
  /* esp_timer_get_time() is already 64-bit, so there is no 49.7-day millis()
   * wrap bookkeeping and concurrent readers cannot corrupt wrap state. */
  return (uint64_t)(esp_timer_get_time() / 1000000LL);
}

void loadBootCount() {
  prefs.begin("diag", false);
  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);
  prefs.end();
}

bool validWiFiCredentials(const char *ssid, const char *password) {
  if (!ssid || !password) return false;
  const size_t ssidLength = strlen(ssid);
  const size_t passwordLength = strlen(password);
  return ssidLength >= 1 && ssidLength <= 32 &&
         passwordLength >= 8 && passwordLength <= 63;
}

WiFiCredentials loadWiFiCredentialsSnapshot() {
  WiFiCredentials snapshot;
  portENTER_CRITICAL(&g_wifiConfigMux);
  snapshot = wifiCredentials;
  portEXIT_CRITICAL(&g_wifiConfigMux);
  return snapshot;
}

void storeWiFiCredentialsSnapshot(const WiFiCredentials &credentials) {
  portENTER_CRITICAL(&g_wifiConfigMux);
  wifiCredentials = credentials;
  portEXIT_CRITICAL(&g_wifiConfigMux);
}

void requestWiFiReconfigure() {
  portENTER_CRITICAL(&g_wifiConfigMux);
  wifiReconfigureRequested = true;
  portEXIT_CRITICAL(&g_wifiConfigMux);
}

bool takeWiFiReconfigureRequest() {
  portENTER_CRITICAL(&g_wifiConfigMux);
  const bool requested = wifiReconfigureRequested;
  wifiReconfigureRequested = false;
  portEXIT_CRITICAL(&g_wifiConfigMux);
  return requested;
}

// Open read/write even while loading so first boot creates the namespace
// cleanly instead of Preferences logging NOT_FOUND for a read-only open.
bool loadWiFiCredentials() {
  WiFiCredentials loaded;
  Preferences wifiPrefs;
  if (!wifiPrefs.begin(WIFI_PREF_NAMESPACE, false)) {
    Serial.println("WiFi config: cannot open NVS namespace");
    storeWiFiCredentialsSnapshot(loaded);
    return false;
  }

  if (wifiPrefs.isKey(WIFI_PREF_SSID) &&
      wifiPrefs.isKey(WIFI_PREF_PASSWORD)) {
    wifiPrefs.getString(WIFI_PREF_SSID, loaded.ssid, sizeof(loaded.ssid));
    wifiPrefs.getString(WIFI_PREF_PASSWORD, loaded.password,
                        sizeof(loaded.password));
    loaded.configured = validWiFiCredentials(loaded.ssid, loaded.password);
  }
  wifiPrefs.end();

  storeWiFiCredentialsSnapshot(loaded);
  return loaded.configured;
}

bool saveWiFiCredentials(const char *ssid, const char *password) {
  if (!validWiFiCredentials(ssid, password)) return false;

  Preferences wifiPrefs;
  if (!wifiPrefs.begin(WIFI_PREF_NAMESPACE, false)) return false;
  const bool ssidSaved =
      wifiPrefs.putString(WIFI_PREF_SSID, ssid) == strlen(ssid);
  const bool passwordSaved =
      wifiPrefs.putString(WIFI_PREF_PASSWORD, password) == strlen(password);
  wifiPrefs.end();
  if (!ssidSaved || !passwordSaved) return false;

  WiFiCredentials saved;
  strlcpy(saved.ssid, ssid, sizeof(saved.ssid));
  strlcpy(saved.password, password, sizeof(saved.password));
  saved.configured = true;
  storeWiFiCredentialsSnapshot(saved);
  requestWiFiReconfigure();
  return true;
}

bool clearWiFiCredentials() {
  Preferences wifiPrefs;
  if (!wifiPrefs.begin(WIFI_PREF_NAMESPACE, false)) return false;
  const bool cleared = wifiPrefs.clear();
  wifiPrefs.end();
  if (!cleared) return false;

  storeWiFiCredentialsSnapshot(WiFiCredentials{});
  requestWiFiReconfigure();
  return true;
}

// Trạng thái OTA chờ gửi kèm bản tin telemetry kế tiếp. nullptr = không có gì
// để báo. Dùng con trỏ hằng vì chỉ nhận các chuỗi literal cố định.
const char *pendingFwStatus = nullptr;
char pendingFwDetail[48] = "";
int pendingFwProgress = -1;  // -1 = bản tin trạng thái này không mang phần trăm

// Snapshot dài hạn cho dashboard local. Khác pendingFw*: bản pending bị xoá
// ngay sau một publish MQTT thành công, còn snapshot phải giữ nguyên để
// /api/telemetry vẫn đọc được trong suốt quá trình tải (hai task khác core).
struct OtaRuntimeSnapshot {
  char status[16] = "idle";
  char detail[48] = "";
  int progress = -1;
};
OtaRuntimeSnapshot otaRuntime;
static portMUX_TYPE g_otaStateMux = portMUX_INITIALIZER_UNLOCKED;

// Prototype tường minh: Arduino tự sinh prototype ở đầu sketch, trước cả định
// nghĩa struct, và sẽ làm các hàm trả về snapshot không biên dịch được.
OtaRuntimeSnapshot loadOtaRuntimeSnapshot();
OtaRuntimeSnapshot loadOtaRuntimeSnapshot() {
  OtaRuntimeSnapshot snapshot;
  portENTER_CRITICAL(&g_otaStateMux);
  snapshot = otaRuntime;
  portEXIT_CRITICAL(&g_otaStateMux);
  return snapshot;
}

// Ảnh mới chỉ được xác nhận "chạy tốt" sau khi đã nối lại được AWS IoT — xem
// confirmFirmwareIfPending().
bool firmwareConfirmed = false;

// Tạo dao động trơn, có chu kỳ và luôn nằm trong [minValue, maxValue]. Cách này
// làm dữ liệu demo thay đổi tự nhiên hơn random độc lập ở từng lần đọc.
float simulatedWave(float minValue, float maxValue,
                    unsigned long periodMs, float phaseOffset = 0.0f) {
  const float phase = ((millis() % periodMs) / (float)periodMs) * TWO_PI
                      + phaseOffset;
  const float normalized = (sinf(phase) + 1.0f) * 0.5f;
  return minValue + (maxValue - minValue) * normalized;
}

float simulatedSolarKw() {
  // Telemetry dùng kW, nên 5–20 W tương ứng 0.005–0.020 kW.
  const float solarW = simulatedWave(5.0f, 20.0f, 60000UL, 0.35f);
  return roundf(solarW) / 1000.0f;  // độ phân giải 1 W
}

float simulatedLoadW() {
  // Tải tiêu thụ dao động chậm và lệch pha với solar, độ phân giải 0.1 W.
  return roundf(simulatedWave(3.0f, 12.0f, 45000UL, 4.10f) * 10.0f)
         / 10.0f;
}

static BmsTelemetry loadBmsTelemetrySnapshot();

static BmsTelemetry loadBmsTelemetrySnapshot() {
  BmsTelemetry out;
  portENTER_CRITICAL(&g_bmsTelemetryMux);
  out = bmsTlm;
  portEXIT_CRITICAL(&g_bmsTelemetryMux);
  return out;
}

// Chuyển telemetry STM32-BMS sang kiểu BatteryReading mà phần bảo vệ/cloud
// hiện tại đang dùng. STM32 quy ước +I=xả, -I=sạc; ESP32 hiện tại quy ước
// +I=sạc, -I=xả, do đó phải đảo dấu đúng một lần tại đây.
BatteryReading readBattery() {
  const BmsTelemetry tlm = loadBmsTelemetrySnapshot();
  BatteryReading r = {0.0f, 0.0f, 0.0f, 0.0f};

  if (!tlm.connected || (millis() - tlm.received_ms) > BMS_STALE_MS)
    return r;

  if (tlm.flags & BMS_FLAG_SOC_VALID)
    r.soc = tlm.soc_centi_pct / 100.0f;
  if (tlm.flags & BMS_FLAG_VOLTAGE_VALID)
    r.voltage = tlm.pack_mV / 1000.0f;
  if (tlm.flags & BMS_FLAG_CURRENT_VALID)
    r.current = -(tlm.current_mA / 1000.0f);
  if (tlm.flags & BMS_FLAG_TEMP_VALID)
    r.tempC = tlm.pack_temp_cC / 100.0f;
  return r;
}


struct PvStateSnapshot {
  BuckboostTelemetry tlm;
  bool linkOk;
  float solarKw;
};

struct ProtectionStateSnapshot {
  BatteryReading reading;
  BatteryConfig cfg;
  bool chargeEnabled;
  bool dischargeEnabled;
  const char *reason;
};

static PvStateSnapshot loadPvState();
static ProtectionStateSnapshot loadProtectionState();

static PvStateSnapshot loadPvState() {
  PvStateSnapshot out;
  portENTER_CRITICAL(&g_pvStateMux);
  out.tlm = buckboostTlm;
  out.linkOk = buckboostLinkOk;
  out.solarKw = solarKw;
  portEXIT_CRITICAL(&g_pvStateMux);
  return out;
}

static ProtectionStateSnapshot loadProtectionState() {
  ProtectionStateSnapshot out;
  portENTER_CRITICAL(&g_batteryStateMux);
  out.reading = lastBatteryReading;
  out.cfg = batteryCfg;
  out.chargeEnabled = chargeEnabled;
  out.dischargeEnabled = dischargeEnabled;
  out.reason = protectReason;
  portEXIT_CRITICAL(&g_batteryStateMux);
  return out;
}

static BatteryConfig loadBatteryConfigSnapshot() {
  BatteryConfig out;
  portENTER_CRITICAL(&g_batteryStateMux);
  out = batteryCfg;
  portEXIT_CRITICAL(&g_batteryStateMux);
  return out;
}

static BmsConfigAckSnapshot loadBmsConfigAckSnapshot() {
  BmsConfigAckSnapshot out;
  portENTER_CRITICAL(&g_bmsConfigAckMux);
  out = bmsConfigAck;
  portEXIT_CRITICAL(&g_bmsConfigAckMux);
  return out;
}

static void clearBmsConfigAckIfUnchanged(const BmsConfigAckSnapshot &sent) {
  portENTER_CRITICAL(&g_bmsConfigAckMux);
  if (bmsConfigAck.pending &&
      bmsConfigAck.configVersion == sent.configVersion &&
      strcmp(bmsConfigAck.configId, sent.configId) == 0 &&
      strcmp(bmsConfigAck.configHash, sent.configHash) == 0) {
    bmsConfigAck.pending = false;
  }
  portEXIT_CRITICAL(&g_bmsConfigAckMux);
}

static void queueBmsConfigAck(const char *configId, uint64_t configVersion,
                              const char *configHash, const char *status,
                              const char *detail) {
  portENTER_CRITICAL(&g_bmsConfigAckMux);
  bmsConfigAck.pending = true;
  strlcpy(bmsConfigAck.configId, configId ? configId : "", sizeof(bmsConfigAck.configId));
  bmsConfigAck.configVersion = configVersion;
  strlcpy(bmsConfigAck.configHash, configHash ? configHash : "", sizeof(bmsConfigAck.configHash));
  strlcpy(bmsConfigAck.status, status ? status : "rejected", sizeof(bmsConfigAck.status));
  strlcpy(bmsConfigAck.detail, detail ? detail : "", sizeof(bmsConfigAck.detail));
  portEXIT_CRITICAL(&g_bmsConfigAckMux);
}

// ---------------------------------------------------------------------
// UART STM32 buck-boost + BMS
// ---------------------------------------------------------------------

// CRC8 poly 0x07, init 0x00 — cùng thuật toán với bảng CRC bên STM32.
uint8_t crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

void buckboostSendRequest(uint8_t cmd) {
  uint8_t f[5];
  f[0] = FRAME_SOF;
  f[1] = MASTER_ID;
  f[2] = 1;
  f[3] = cmd;
  f[4] = crc8(&f[1], 3);

  while (Serial1.available()) Serial1.read();   // xả rác còn lại trong buffer
  Serial1.write(f, sizeof(f));
  Serial1.flush();
}

// Đọc một khung hợp lệ từ STM32. Trả payload qua out/outLen và ID người gửi
// qua `from` — pollBuckboost() kiểm ID để không nhận nhầm khung của node khác nếu
// về sau bus có thêm thiết bị.
bool buckboostReadFrame(uint8_t *out, uint8_t *outLen, uint8_t *from, unsigned long timeoutMs) {
  uint8_t buf[MAX_PAYLOAD + 4];
  uint16_t n = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!Serial1.available()) {
      delay(1);
      continue;
    }
    uint8_t b = (uint8_t)Serial1.read();

    if (n == 0 && b != FRAME_SOF) continue;   // đồng bộ theo SOF
    buf[n++] = b;

    if (n == 2 && buf[1] == MASTER_ID) {      // khung do chính mình phát ra
      n = 0;
      continue;
    }
    if (n >= 3) {
      uint8_t plen = buf[2];
      if (plen > MAX_PAYLOAD) {               // LEN vô lý -> bỏ, tìm SOF mới
        n = 0;
        continue;
      }
      if (n == (uint16_t)plen + 4) {
        if (crc8(&buf[1], (uint16_t)(plen + 2)) != buf[plen + 3]) {
          n = 0;
          continue;
        }
        memcpy(out, &buf[3], plen);
        *outLen = plen;
        *from = buf[1];
        return true;
      }
    }
  }
  return false;
}

// Số lần hỏi hụt liên tiếp. Chỉ để giảm rác trên Serial: STM32 rút dây ra là
// hỏng mỗi 2 giây, in hết thì log OTA/MQTT không còn đọc được nữa.
uint16_t buckboostFailStreak = 0;

void pollBuckboost() {
  uint8_t payload[MAX_PAYLOAD];
  uint8_t len = 0;
  uint8_t from = 0xFF;

  buckboostSendRequest(CMD_GET_TLM);

  if (buckboostReadFrame(payload, &len, &from, BUCKBOOST_REPLY_TIMEOUT_MS) &&
      from == NODE_ID && len == sizeof(BuckboostTelemetry)) {
    BuckboostTelemetry newTlm;
    memcpy(&newTlm, payload, sizeof(newTlm));
    const float measuredSolarKw =
        ((float)newTlm.vin_mV * (float)newTlm.iin_mA) / 1000000.0f / 1000.0f;
    const float demoSolarKw = simulatedSolarKw();

    portENTER_CRITICAL(&g_pvStateMux);
    buckboostTlm = newTlm;
    solarKw = demoSolarKw;
    buckboostLinkOk = true;
    portEXIT_CRITICAL(&g_pvStateMux);

    lastBuckboostOk = millis();
    buckboostFailStreak = 0;
    Serial.printf("buckboost-f103 vin=%.2fV iin=%.2fA -> measured=%.3fkW demo=%.3fkW | vout=%.2fV iout=%.2fA duty=%.1f%%\n",
                  newTlm.vin_mV / 1000.0f, newTlm.iin_mA / 1000.0f,
                  measuredSolarKw, demoSolarKw,
                  newTlm.vout_mV / 1000.0f, newTlm.iout_mA / 1000.0f,
                  newTlm.duty_permil / 10.0f);
    return;
  }

  if (buckboostFailStreak == 0 || buckboostFailStreak % 15 == 0) {
    Serial.printf("buckboost-f103: khong nhan duoc khung hop le (from=0x%02X len=%u, hut %u lan)\n",
                  from, len, (unsigned)buckboostFailStreak + 1);
  }
  buckboostFailStreak++;

  const bool linkIsStale =
      lastBuckboostOk == 0 || millis() - lastBuckboostOk > BUCKBOOST_STALE_MS;
  const float demoSolarKw = simulatedSolarKw();
  portENTER_CRITICAL(&g_pvStateMux);
  solarKw = demoSolarKw;
  if (linkIsStale) {
    buckboostLinkOk = false;
    // Mất STM32 vẫn giữ giá trị solar demo; stm32_link=false cho phép dashboard
    // phân biệt rõ dữ liệu mô phỏng với một liên kết phần cứng đang hoạt động.
  }
  portEXIT_CRITICAL(&g_pvStateMux);
}

static uint16_t getU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t getI16LE(const uint8_t *p) {
  return (int16_t)getU16LE(p);
}

static void bmsSendRequest(uint16_t seq) {
  uint8_t f[7];
  f[0] = FRAME_SOF;
  f[1] = MASTER_ID;
  f[2] = 3;
  f[3] = MSG_GET_BMS;
  f[4] = (uint8_t)(seq & 0xFFu);
  f[5] = (uint8_t)(seq >> 8);
  f[6] = crc8(&f[1], 5);

  while (Serial2.available()) Serial2.read();
  Serial2.write(f, sizeof(f));
  Serial2.flush();
}

static bool bmsReadFrame(uint8_t *out, uint8_t *outLen, uint8_t *from,
                         unsigned long timeoutMs) {
  uint8_t buf[MAX_PAYLOAD + 4];
  uint16_t n = 0;
  const unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!Serial2.available()) {
      delay(1);
      continue;
    }

    const uint8_t b = (uint8_t)Serial2.read();
    if (n == 0 && b != FRAME_SOF) continue;
    buf[n++] = b;

    if (n == 2 && buf[1] == MASTER_ID) {
      n = 0;
      continue;
    }

    if (n >= 3) {
      const uint8_t plen = buf[2];
      if (plen > MAX_PAYLOAD) {
        n = 0;
        continue;
      }
      if (n == (uint16_t)plen + 4u) {
        if (crc8(&buf[1], (uint16_t)(plen + 2u)) != buf[plen + 3u]) {
          n = 0;
          continue;
        }
        memcpy(out, &buf[3], plen);
        *outLen = plen;
        *from = buf[1];
        return true;
      }
    }
  }
  return false;
}

static bool parseBmsResult(const uint8_t *payload, uint8_t len,
                           uint16_t expectedSeq, BmsTelemetry &out) {
  if (!payload || len != BMS_RESULT_LEN || payload[0] != MSG_BMS_RESULT)
    return false;

  const uint16_t seq = getU16LE(&payload[1]);
  if (seq != expectedSeq) return false;

  for (uint8_t i = 0; i < 4; ++i)
    out.cell_mV[i] = getU16LE(&payload[3 + 2 * i]);
  out.pack_mV = getU16LE(&payload[11]);
  out.current_mA = getI16LE(&payload[13]);
  out.soc_centi_pct = getU16LE(&payload[15]);
  out.pack_temp_cC = getI16LE(&payload[17]);
  out.output_enabled = payload[19];
  out.flags = payload[20];

  if (out.soc_centi_pct > 10000u) return false;
  out.received_ms = millis();
  out.connected = true;
  return true;
}

static void logBmsTelemetryIfDue(const BmsTelemetry &tlm) {
  const unsigned long now = millis();
  if (now - lastBmsTelemetryLog < BMS_TELEMETRY_LOG_MS) return;
  lastBmsTelemetryLog = now;

  Serial.printf(
      "[BMS] SOC=%.2f%% | Vpack=%.3fV | I=%.3fA(STM32 sign) | T=%.2fC | "
      "OUT=%u | flags=0x%02X | cells=[%.3f %.3f %.3f %.3f]V\n",
      tlm.soc_centi_pct / 100.0f,
      tlm.pack_mV / 1000.0f,
      tlm.current_mA / 1000.0f,
      tlm.pack_temp_cC / 100.0f,
      (unsigned)tlm.output_enabled,
      (unsigned)tlm.flags,
      tlm.cell_mV[0] / 1000.0f,
      tlm.cell_mV[1] / 1000.0f,
      tlm.cell_mV[2] / 1000.0f,
      tlm.cell_mV[3] / 1000.0f);
}

static void pollBms() {
  uint8_t payload[MAX_PAYLOAD];
  uint8_t len = 0;
  uint8_t from = 0xFF;
  const uint16_t seq = ++bmsSequence;
  const BmsTelemetry before = loadBmsTelemetrySnapshot();
  const bool wasConnected = before.connected &&
                            (millis() - before.received_ms <= BMS_STALE_MS);

  bmsSendRequest(seq);

  BmsTelemetry parsed;
  if (bmsReadFrame(payload, &len, &from, BMS_REPLY_TIMEOUT_MS) &&
      from == BMS_NODE_ID && parseBmsResult(payload, len, seq, parsed)) {
    portENTER_CRITICAL(&g_bmsTelemetryMux);
    bmsTlm = parsed;
    portEXIT_CRITICAL(&g_bmsTelemetryMux);

    ++bmsFrameOk;
    if (!wasConnected) {
      Serial.println("[BMS] Connected successfully - STM32F103 BMS UART link is active");
      Serial.printf("[BMS] node=0x%02X, baud=%lu, valid frames=%lu\n",
                    BMS_NODE_ID, (unsigned long)BMS_UART_BAUD,
                    (unsigned long)bmsFrameOk);
    }
    logBmsTelemetryIfDue(parsed);
    return;
  }

  ++bmsFrameFail;
  const unsigned long now = millis();
  BmsTelemetry current = loadBmsTelemetrySnapshot();
  const bool stale = !current.connected ||
                     (now - current.received_ms > BMS_STALE_MS);

  if (stale && current.connected) {
    portENTER_CRITICAL(&g_bmsTelemetryMux);
    bmsTlm.connected = false;
    portEXIT_CRITICAL(&g_bmsTelemetryMux);
    Serial.printf("[BMS] Connection lost - no valid response for > %lu ms\n",
                  BMS_STALE_MS);
  }

  if (now - lastBmsStatusReminder >= BMS_STATUS_REMINDER_MS) {
    lastBmsStatusReminder = now;
    const BmsTelemetry state = loadBmsTelemetrySnapshot();
    if (!state.connected) {
      Serial.printf("[BMS] Waiting for STM32F103 BMS... fail=%lu, last from=0x%02X len=%u\n",
                    (unsigned long)bmsFrameFail, from, len);
    }
  }
}

void loadBatteryConfig() {
  BatteryConfig cfg;
  portENTER_CRITICAL(&g_batteryStateMux);
  cfg = batteryCfg;
  portEXIT_CRITICAL(&g_batteryStateMux);

  // Read/write mode creates the namespace quietly on first boot. Only read
  // keys that already exist so factory-new flash uses the defaults above.
  if (prefs.begin("battcfg", false)) {
    if (prefs.isKey("minSoc"))
      cfg.minSoc = prefs.getFloat("minSoc", cfg.minSoc);
    if (prefs.isKey("maxSoc"))
      cfg.maxSoc = prefs.getFloat("maxSoc", cfg.maxSoc);
    if (prefs.isKey("maxVoltage")) {
      const float storedMaxVoltage = prefs.getFloat("maxVoltage", cfg.maxVoltage);
      if (storedMaxVoltage >= PACK_MAX_VOLTAGE_MIN && storedMaxVoltage <= PACK_MAX_VOLTAGE_MAX)
        cfg.maxVoltage = storedMaxVoltage;
    }
    if (prefs.isKey("maxCurrent"))
      cfg.maxCurrent = prefs.getFloat("maxCurrent", cfg.maxCurrent);
    if (prefs.isKey("ddp"))
      cfg.deepDischargeProtect = prefs.getBool("ddp", cfg.deepDischargeProtect);
    prefs.end();
  }

  portENTER_CRITICAL(&g_batteryStateMux);
  batteryCfg = cfg;
  portEXIT_CRITICAL(&g_batteryStateMux);
}

bool saveBatteryConfig() {
  BatteryConfig cfg = loadBatteryConfigSnapshot();
  if (!prefs.begin("battcfg", false)) return false;
  const bool saved = prefs.putFloat("minSoc", cfg.minSoc) > 0 &&
                     prefs.putFloat("maxSoc", cfg.maxSoc) > 0 &&
                     prefs.putFloat("maxVoltage", cfg.maxVoltage) > 0 &&
                     prefs.putFloat("maxCurrent", cfg.maxCurrent) > 0 &&
                     prefs.putBool("ddp", cfg.deepDischargeProtect) > 0;
  prefs.end();
  return saved;
}

void writeProtectRelays() {
  ProtectionStateSnapshot p = loadProtectionState();
  if (CHARGE_RELAY_PIN != 255)
    digitalWrite(CHARGE_RELAY_PIN, p.chargeEnabled ? HIGH : LOW);
  if (DISCHARGE_RELAY_PIN != 255)
    digitalWrite(DISCHARGE_RELAY_PIN, p.dischargeEnabled ? HIGH : LOW);
}

void applyProtection() {
  const BmsTelemetry tlm = loadBmsTelemetrySnapshot();
  const bool bmsFresh = tlm.connected &&
                        (millis() - tlm.received_ms <= BMS_STALE_MS);
  const bool protectionDataValid =
      (tlm.flags & (BMS_FLAG_VOLTAGE_VALID |
                    BMS_FLAG_CURRENT_VALID |
                    BMS_FLAG_SOC_VALID)) ==
      (BMS_FLAG_VOLTAGE_VALID |
       BMS_FLAG_CURRENT_VALID |
       BMS_FLAG_SOC_VALID);
  const bool bmsFaultLatched = (tlm.flags & BMS_FLAG_FAULT_LATCHED) != 0;
  const BatteryReading r = readBattery();

  portENTER_CRITICAL(&g_batteryStateMux);
  lastBatteryReading = r;

  // Fail-safe: mất BMS / dữ liệu sensor hết hạn / BMS đã latch fault thì ESP32
  // ngắt cả hai relay. STM32 BMS vẫn có lớp bảo vệ độc lập của chính nó.
  if (!bmsFresh) {
    chargeEnabled = false;
    dischargeEnabled = false;
    protectReason = "bms_offline";
  } else if (!protectionDataValid) {
    chargeEnabled = false;
    dischargeEnabled = false;
    protectReason = "bms_invalid";
  } else if (bmsFaultLatched) {
    chargeEnabled = false;
    dischargeEnabled = false;
    protectReason = "bms_fault";
  } else {
    if (r.current >= batteryCfg.maxCurrent) {
      chargeEnabled = false;
      protectReason = "overcurrent";
    } else if (r.voltage >= batteryCfg.maxVoltage) {
      chargeEnabled = false;
      protectReason = "overvoltage";
    } else if (r.soc >= batteryCfg.maxSoc) {
      chargeEnabled = false;
      protectReason = "full";
    } else if (!chargeEnabled) {
      const bool voltOk = r.voltage < batteryCfg.maxVoltage - VOLT_CHARGE_RESUME_MARGIN;
      const bool socOk = r.soc < batteryCfg.maxSoc - SOC_CHARGE_RESUME_MARGIN;
      const bool currentOk = r.current < batteryCfg.maxCurrent - CURRENT_CHARGE_RESUME_MARGIN;
      if (voltOk && socOk && currentOk) chargeEnabled = true;
    }

    if (batteryCfg.deepDischargeProtect && r.soc <= batteryCfg.minSoc) {
      dischargeEnabled = false;
      protectReason = "deep_discharge";
    } else if (!dischargeEnabled) {
      if (r.soc > batteryCfg.minSoc + SOC_DISCHARGE_RESUME_MARGIN)
        dischargeEnabled = true;
    }

    if (chargeEnabled && dischargeEnabled) protectReason = "ok";
  }

  portEXIT_CRITICAL(&g_batteryStateMux);
  writeProtectRelays();
}

void publishTelemetry();

// Gửi ACK cấu hình ngoài callback MQTT. Callback chỉ validate/lưu NVS và xếp
// ACK vào snapshot; CloudTask là owner duy nhất của PubSubClient.
void flushBmsConfigAck() {
  const BmsConfigAckSnapshot ack = loadBmsConfigAckSnapshot();
  if (!ack.pending || !mqtt.connected()) return;
  publishTelemetry();
  lastPublish = millis();
}

// Ghi nhận trạng thái OTA để gửi kèm bản tin telemetry kế tiếp.
//
// Cố tình KHÔNG publish tại chỗ: hàm này được gọi từ trong callback MQTT, mà
// PubSubClient dùng CHUNG một buffer cho gói vào và gói ra — publish ngay đó
// sẽ ghi đè lên chính payload đang đọc dở. CloudTask gọi flushFwStatus() ngoài callback để
// gửi, chỉ chậm hơn vài micro giây.
void setFwStatus(const char *status, const char *detail, int progress = -1) {
  pendingFwStatus = status;
  strlcpy(pendingFwDetail, detail ? detail : "", sizeof(pendingFwDetail));
  pendingFwProgress = progress;

  portENTER_CRITICAL(&g_otaStateMux);
  strlcpy(otaRuntime.status, status, sizeof(otaRuntime.status));
  strlcpy(otaRuntime.detail, detail ? detail : "", sizeof(otaRuntime.detail));
  otaRuntime.progress = progress;
  portEXIT_CRITICAL(&g_otaStateMux);
  if (progress >= 0) {
    Serial.printf(
      "fw_status %s%s%s / %d%%\n",
      status, detail ? " / " : "", detail ? detail : "", progress
    );
  } else {
    Serial.printf("fw_status %s%s%s\n", status, detail ? " / " : "", detail ? detail : "");
  }
}

// Đẩy trạng thái OTA đi ngay thay vì đợi hết chu kỳ 10s — người vừa bấm "Đẩy
// OTA" đang nhìn dashboard chờ phản hồi.
void flushFwStatus() {
  if (!pendingFwStatus || !mqtt.connected()) return;
  publishTelemetry();
  lastPublish = millis();
}

// Defer Jobs publishes until after the MQTT callback returns because
// PubSubClient uses one shared packet buffer for receive and transmit.
void queueAwsJobStatus(const char *jobId, const char *status, const char *detail,
                       int progress = -1) {
  if (!jobId || !jobId[0]) return;  // legacy solgrid/<thing>/command OTA
  strlcpy(awsJobStatus.jobId, jobId, sizeof(awsJobStatus.jobId));
  strlcpy(awsJobStatus.status, status, sizeof(awsJobStatus.status));
  strlcpy(awsJobStatus.detail, detail ? detail : "", sizeof(awsJobStatus.detail));
  awsJobStatus.progress = progress;
  awsJobStatus.pending = true;
}

bool flushAwsJobStatus() {
  if (!awsJobStatus.pending || !mqtt.connected()) return false;

  char topic[192];
  snprintf(topic, sizeof(topic), "$aws/things/%s/jobs/%s/update",
           THING_NAME, awsJobStatus.jobId);
  StaticJsonDocument<256> doc;
  doc["status"] = awsJobStatus.status;
  JsonObject details = doc.createNestedObject("statusDetails");
  details["detail"] = awsJobStatus.detail;
  details["firmwareVersion"] = FW_VERSION;
  // AWS IoT Jobs yêu cầu statusDetails là map string -> string.
  char progressText[4];
  if (awsJobStatus.progress >= 0) {
    snprintf(progressText, sizeof(progressText), "%d", awsJobStatus.progress);
    details["progressPercent"] = progressText;
  }
  doc["includeJobExecutionState"] = false;

  char payload[256];
  const size_t length = serializeJson(doc, payload, sizeof(payload));
  const bool ok = mqtt.publish(topic, (const uint8_t *)payload, (unsigned int)length);
  if (ok) {
    Serial.printf("AWS Job %s -> %s (%s)\n", awsJobStatus.jobId,
                  awsJobStatus.status, awsJobStatus.detail);
    awsJobStatus.pending = false;
  }
  return ok;
}

bool saveOtaCompletionMarker(const OtaJob &job) {
  Preferences otaPrefs;
  if (!otaPrefs.begin(OTA_PREF_NAMESPACE, false)) return false;
  const bool versionOk = otaPrefs.putString("version", job.version) > 0;
  const bool jobOk = otaPrefs.putString("job", job.awsJobId) > 0 || !job.awsJobId[0];
  // Ghi cờ sau cùng: mất điện giữa chừng không được tạo marker nửa vời.
  const bool pendingOk = otaPrefs.putBool("pending", true) > 0;
  otaPrefs.end();

  if (versionOk && jobOk && pendingOk) {
    otaCompletion.pending = true;
    strlcpy(otaCompletion.version, job.version, sizeof(otaCompletion.version));
    strlcpy(otaCompletion.awsJobId, job.awsJobId, sizeof(otaCompletion.awsJobId));
    return true;
  }
  return false;
}

void loadOtaCompletionMarker() {
  Preferences otaPrefs;
  if (!otaPrefs.begin(OTA_PREF_NAMESPACE, false)) return;
  otaCompletion.pending = otaPrefs.getBool("pending", false);
  if (otaCompletion.pending) {
    const String version = otaPrefs.getString("version", "");
    const String jobId = otaPrefs.getString("job", "");
    strlcpy(otaCompletion.version, version.c_str(), sizeof(otaCompletion.version));
    strlcpy(otaCompletion.awsJobId, jobId.c_str(), sizeof(otaCompletion.awsJobId));
  }
  otaPrefs.end();

  if (!otaCompletion.pending) return;
  if (strcmp(otaCompletion.version, FW_VERSION) == 0) {
    setFwStatus("applying", "validating_boot", 100);
  } else {
    // Thiết bị đã quay lại bản cũ (rollback hoặc nạp tay một ảnh khác).
    setFwStatus("failed", "rollback", 100);
  }
}

void clearOtaCompletionMarker() {
  Preferences otaPrefs;
  if (otaPrefs.begin(OTA_PREF_NAMESPACE, false)) {
    otaPrefs.clear();
    otaPrefs.end();
  }
  otaCompletion = OtaCompletionMarker{};
  otaCompletionReporting = false;
}

// Gọi sau khi bản mới đã nối lại AWS. Chưa xoá marker tại đây: nếu một publish
// lỗi, CloudTask sẽ thử lại và chỉ clear NVS sau khi cả hai kênh đã gửi xong.
void reportOtaCompletionAfterReconnect() {
  if (!otaCompletion.pending || otaCompletionReporting) return;
  const bool runningTarget = strcmp(otaCompletion.version, FW_VERSION) == 0;
  if (runningTarget) {
    setFwStatus("success", FW_VERSION, 100);
    queueAwsJobStatus(otaCompletion.awsJobId, "SUCCEEDED", "new_firmware_online", 100);
  } else {
    setFwStatus("failed", "rollback", 100);
    queueAwsJobStatus(otaCompletion.awsJobId, "FAILED", "rollback", 100);
  }
  otaCompletionReporting = true;
}

void clearOtaCompletionMarkerIfReported() {
  if (!otaCompletionReporting) return;
  if (pendingFwStatus || awsJobStatus.pending) return;
  clearOtaCompletionMarker();
}

bool flushAwsStartNextRequest() {
  if (!awsStartNextRequested || !mqtt.connected()) return false;
  char topic[160];
  snprintf(topic, sizeof(topic), "$aws/things/%s/jobs/start-next", THING_NAME);
  const bool ok = mqtt.publish(topic, "{}");
  if (ok) awsStartNextRequested = false;
  return ok;
}

// Chuỗi hex 64 ký tự → 32 byte. false nếu không đúng định dạng.
bool hexToBytes(const char *hex, uint8_t *out, size_t outLen) {
  for (size_t i = 0; i < outLen; i++) {
    char hi = hex[i * 2], lo = hex[i * 2 + 1];
    if (!isxdigit((int)hi) || !isxdigit((int)lo)) return false;
    auto nib = [](char c) -> uint8_t {
      if (c >= '0' && c <= '9') return c - '0';
      return (uint8_t)((c | 0x20) - 'a' + 10);
    };
    out[i] = (uint8_t)((nib(hi) << 4) | nib(lo));
  }
  return true;
}

// Tải ảnh firmware và ghi vào phân vùng OTA rảnh.
//
// Thứ tự quan trọng: vừa ghi flash vừa băm, và CHỈ kích hoạt phân vùng mới
// (Update.end) khi hash khớp. Ghi trước - kiểm sau là an toàn vì ảnh đã ghi
// không được boot cho tới khi phân vùng được đánh dấu; hash sai thì
// Update.abort() bỏ luôn, thiết bị vẫn chạy bản cũ.
//
// Vì sao hash là đủ, kể cả khi kênh HTTPS bị nghi ngờ: chuỗi sha256 tới qua
// MQTT trên kênh mutual-TLS X.509 với AWS IoT — kênh đã xác thực. Kẻ can thiệp
// đường tải HTTPS không thể đổi nội dung ảnh mà vẫn khớp hash đó.
bool runOtaUpdate(const OtaJob &job) {
  if (WiFi.status() != WL_CONNECTED) {
    setFwStatus("failed", "wifi_down");
    queueAwsJobStatus(job.awsJobId, "FAILED", "wifi_down", 0);
    return false;
  }

  uint8_t expected[32];
  if (strlen(job.sha256) != 64 || !hexToBytes(job.sha256, expected, sizeof(expected))) {
    setFwStatus("failed", "bad_sha256_field");
    queueAwsJobStatus(job.awsJobId, "REJECTED", "bad_sha256_field", 0);
    return false;
  }

  WiFiClientSecure otaNet;
#ifdef OTA_DOWNLOAD_ROOT_CA
  otaNet.setCACert(OTA_DOWNLOAD_ROOT_CA);
  Serial.println("OTA HTTPS: verify server with OTA_DOWNLOAD_ROOT_CA");
#else
  // Không dùng AWS_ROOT_CA ở đây: CA của AWS IoT không xác thực được hostname
  // Supabase Storage và làm HTTPClient trả HTTPC_ERROR_CONNECTION_REFUSED (-1).
  otaNet.setInsecure();
  Serial.println("OTA HTTPS: CA pin disabled; firmware image is verified by SHA-256");
#endif
  otaNet.setHandshakeTimeout(15);
  HTTPClient http;
  // Signed URL có thể redirect sang backend object storage khác hostname.
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);
  if (!http.begin(otaNet, job.url)) {
    setFwStatus("failed", "http_begin");
    queueAwsJobStatus(job.awsJobId, "FAILED", "http_begin", 0);
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    char detail[32];
    if (code == HTTPC_ERROR_CONNECTION_REFUSED) {
      strlcpy(detail, "https_connection_failed", sizeof(detail));
    } else {
      snprintf(detail, sizeof(detail), "http_%d", code);
    }
    const String transportError = HTTPClient::errorToString(code);
    Serial.printf("OTA GET failed: code=%d, error=%s\n",
                  code, transportError.length() ? transportError.c_str() : "HTTP response");
    http.end();
    setFwStatus("failed", detail);
    queueAwsJobStatus(job.awsJobId, "FAILED", detail, 0);
    return false;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0) {
    http.end();
    setFwStatus("failed", "no_content_length");
    queueAwsJobStatus(job.awsJobId, "FAILED", "no_content_length", 0);
    return false;
  }
  // URL ký sẵn trỏ tới đúng object của bản phát hành, nên độ dài phải khớp cỡ
  // đã ghi trong catalog. Lệch = đang tải nhầm thứ khác → dừng trước khi chạm
  // vào flash.
  if (job.size > 0 && (size_t)contentLength != job.size) {
    http.end();
    setFwStatus("failed", "size_mismatch");
    queueAwsJobStatus(job.awsJobId, "FAILED", "size_mismatch", 0);
    return false;
  }

  if (!Update.begin((size_t)contentLength)) {
    // Hay gặp nhất: Partition Scheme chỉ có một app partition, hoặc ảnh lớn
    // hơn phân vùng OTA.
    Serial.printf("Update.begin failed: %s\n", Update.errorString());
    http.end();
    setFwStatus("failed", "no_space");
    queueAwsJobStatus(job.awsJobId, "FAILED", "no_space", 0);
    return false;
  }

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  SHA256_STARTS(&sha);

  WiFiClient *stream = http.getStreamPtr();
  // static: không đặt 1 KB lên stack CloudTask (chỉ CloudTask gọi hàm này).
  static uint8_t buf[OTA_CHUNK];
  int remaining = contentLength;
  int currentProgress = 0;
  int lastProgress = 0;
  unsigned long lastProgressAt = millis();
  unsigned long lastData = millis();
  unsigned long lastMqttLoop = millis();
  const char *failDetail = nullptr;

  // Xác nhận thiết bị đã thực sự bắt đầu xử lý ảnh, không chỉ nhận lệnh MQTT.
  setFwStatus("downloading", job.version, 0);
  flushFwStatus();

  while (remaining > 0) {
    size_t avail = stream->available();
    if (avail == 0) {
      if (!http.connected() || millis() - lastData > OTA_STALL_MS) {
        failDetail = "stalled";
        break;
      }
      delay(10);   // nhường CPU cho WiFi stack + watchdog
      if (millis() - lastMqttLoop >= 500) {
        mqtt.loop();
        lastMqttLoop = millis();
      }
      continue;
    }

    int n = stream->readBytes(buf, avail > sizeof(buf) ? sizeof(buf) : avail);
    if (n <= 0) continue;
    lastData = millis();

    if (Update.write(buf, n) != (size_t)n) {
      failDetail = "write_failed";
      break;
    }
    SHA256_UPDATE(&sha, buf, n);
    remaining -= n;
    if (millis() - lastMqttLoop >= 500) {
      mqtt.loop();
      lastMqttLoop = millis();
    }

    currentProgress = (int)(((int64_t)(contentLength - remaining) * 100) / contentLength);
    const unsigned long now = millis();
    if (currentProgress > lastProgress &&
        (currentProgress == 100 || now - lastProgressAt >= OTA_PROGRESS_INTERVAL_MS)) {
      lastProgress = currentProgress;
      lastProgressAt = now;
      setFwStatus("downloading", job.version, currentProgress);
      flushFwStatus();
      queueAwsJobStatus(job.awsJobId, "IN_PROGRESS", "downloading", currentProgress);
      flushAwsJobStatus();
    }
  }
  http.end();

  uint8_t actual[32];
  SHA256_FINISH(&sha, actual);
  mbedtls_sha256_free(&sha);

  if (failDetail) {
    Update.abort();
    setFwStatus("failed", failDetail, currentProgress);
    queueAwsJobStatus(job.awsJobId, "FAILED", failDetail, currentProgress);
    return false;
  }
  // memcmp thường là đủ ở đây (hash công khai, không phải bí mật), nhưng so
  // sánh hằng thời gian không tốn gì thêm.
  uint8_t diff = 0;
  for (size_t i = 0; i < sizeof(actual); i++) diff |= actual[i] ^ expected[i];
  if (diff != 0) {
    Update.abort();
    setFwStatus("failed", "sha_mismatch", 100);
    queueAwsJobStatus(job.awsJobId, "FAILED", "sha_mismatch", 100);
    return false;
  }

  if (!Update.end(true)) {
    Serial.printf("Update.end failed: %s\n", Update.errorString());
    setFwStatus("failed", "activate_failed", 100);
    queueAwsJobStatus(job.awsJobId, "FAILED", "activate_failed", 100);
    return false;
  }

  if (!saveOtaCompletionMarker(job)) {
    // Không chặn reboot: cloud vẫn suy ra success từ fw_version, còn AWS Jobs
    // vẫn gặp lại job qua start-next. Marker chỉ làm xác nhận sau reboot chắc chắn hơn.
    Serial.println("OTA: cannot persist completion marker");
  }

  // Từ đây phân vùng mới đã được đánh dấu boot. Báo 'applying' rồi khởi động
  // lại; bản mới sẽ tự báo fw_version và cloud suy ra 'success'.
  // Flush tại chỗ trước reboot để CloudTask cố gửi trạng thái cuối.
  setFwStatus("applying", job.version, 100);
  // Keep the job IN_PROGRESS until the new image reconnects. start-next then
  // returns the same job and the version guard marks it SUCCEEDED.
  queueAwsJobStatus(job.awsJobId, "IN_PROGRESS", "rebooting_new_firmware", 100);
  flushFwStatus();
  flushAwsJobStatus();
  delay(400);          // kịp đẩy gói MQTT cuối ra đường truyền
  ESP.restart();
  return true;         // không bao giờ tới đây
}

// Xử lý lệnh {type:"ota"} — chỉ ghi nhận, việc tải để CloudTask làm.
void handleOtaCommand(JsonObjectConst doc, const char *awsJobId = nullptr) {
  const char *board = doc["board"] | "";
  const char *version = doc["version"] | "";
  const char *url = doc["url"] | "";
  const char *sha256 = doc["sha256"] | "";

  // Ảnh của board khác nạp vào là mất thiết bị. Cloud không kiểm được (bảng
  // `devices` không có cột board) nên chốt chặn nằm ở đây.
  if (strcmp(board, FW_BOARD) != 0) {
    setFwStatus("failed", "board_mismatch");
    queueAwsJobStatus(awsJobId, "REJECTED", "board_mismatch");
    return;
  }
  // Lệnh cũ bị gửi lại (đẩy trùng, hoặc thiết bị vừa nạp xong đã nhận lại bản
  // tin retained) — nếu không chặn, thiết bị sẽ tải rồi reboot vòng lặp vô tận.
  if (strcmp(version, FW_VERSION) == 0) {
    setFwStatus("success", "already_running", 100);
    queueAwsJobStatus(awsJobId, "SUCCEEDED", "already_running", 100);
    return;
  }
  if (strlen(url) == 0 || strlen(url) >= OTA_URL_MAX || strlen(sha256) != 64) {
    setFwStatus("failed", "bad_command");
    queueAwsJobStatus(awsJobId, "REJECTED", "bad_job_document");
    return;
  }
  strlcpy(otaJob.url, url, sizeof(otaJob.url));
  strlcpy(otaJob.sha256, sha256, sizeof(otaJob.sha256));
  strlcpy(otaJob.version, version, sizeof(otaJob.version));
  strlcpy(otaJob.awsJobId, awsJobId ? awsJobId : "", sizeof(otaJob.awsJobId));
  otaJob.size = doc["size"] | 0;
  otaJob.pending = true;
  // otaJob.version chứ không phải `version`: con trỏ kia trỏ vào doc, vốn có
  // thể trỏ tiếp vào buffer MQTT sẽ bị ghi đè ở lần publish kế.
  setFwStatus("downloading", otaJob.version, 0);
  queueAwsJobStatus(awsJobId, "IN_PROGRESS", "downloading", 0);
}

void handleAwsStartNextAccepted(JsonDocument &response) {
  JsonObjectConst execution = response["execution"].as<JsonObjectConst>();
  if (execution.isNull()) return;  // no queued/in-progress job

  const char *jobId = execution["jobId"] | "";
  JsonObjectConst job = execution["jobDocument"].as<JsonObjectConst>();
  if (!jobId[0] || job.isNull()) {
    Serial.println("AWS Jobs: malformed start-next response");
    return;
  }

  const char *operation = job["operation"] | "";
  if (strcmp(operation, "ota") != 0 && strcmp(operation, "install") != 0) {
    queueAwsJobStatus(jobId, "REJECTED", "unsupported_operation");
    return;
  }

  Serial.printf("AWS Jobs: received %s\n", jobId);
  handleOtaCommand(job, jobId);
}

// Lệnh tới trên solgrid/<thing>/command: cấu hình bảo vệ pin hoặc lệnh OTA.
void onCommand(char *topic, byte *payload, unsigned int length) {
  char jobsNotifyNext[160];
  char jobsStartAccepted[176];
  char jobsStartRejected[176];
  snprintf(jobsNotifyNext, sizeof(jobsNotifyNext),
           "$aws/things/%s/jobs/notify-next", THING_NAME);
  snprintf(jobsStartAccepted, sizeof(jobsStartAccepted),
           "$aws/things/%s/jobs/start-next/accepted", THING_NAME);
  snprintf(jobsStartRejected, sizeof(jobsStartRejected),
           "$aws/things/%s/jobs/start-next/rejected", THING_NAME);

  if (strcmp(topic, jobsNotifyNext) == 0) {
    awsStartNextRequested = true;
    return;
  }

  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, payload, length)) return;

  if (strcmp(topic, jobsStartAccepted) == 0) {
    handleAwsStartNextAccepted(doc);
    return;
  }
  if (strcmp(topic, jobsStartRejected) == 0) {
    Serial.println("AWS Jobs: start-next rejected");
    return;
  }

  const char *type = doc["type"] | "";
  if (strcmp(type, "ota") == 0) {
    handleOtaCommand(doc.as<JsonObjectConst>());
    return;
  }
  if (strcmp(type, "battery_config") != 0) return;

  const char *configId = doc["configId"] | "";
  const uint64_t configVersion = doc["configVersion"].as<uint64_t>();
  const char *configHash = doc["configHash"] | "";
  if (!configId[0] || configVersion == 0 || strlen(configHash) != 64) {
    Serial.println("battery_config rejected: missing identity");
    return;
  }

  BatteryConfig nextCfg = loadBatteryConfigSnapshot();
  nextCfg.minSoc = doc["minSoc"] | nextCfg.minSoc;
  nextCfg.maxSoc = doc["maxSoc"] | nextCfg.maxSoc;
  nextCfg.maxVoltage = doc["maxVoltage"] | nextCfg.maxVoltage;
  nextCfg.maxCurrent = doc["maxCurrent"] | nextCfg.maxCurrent;
  nextCfg.deepDischargeProtect = doc["deepDischargeProtect"] | nextCfg.deepDischargeProtect;

  if (nextCfg.minSoc < 0.0f || nextCfg.minSoc > 100.0f ||
      nextCfg.maxSoc < 0.0f || nextCfg.maxSoc > 100.0f ||
      nextCfg.minSoc >= nextCfg.maxSoc ||
      nextCfg.maxVoltage < PACK_MAX_VOLTAGE_MIN || nextCfg.maxVoltage > PACK_MAX_VOLTAGE_MAX ||
      nextCfg.maxCurrent <= 0.0f) {
    queueBmsConfigAck(configId, configVersion, configHash, "rejected", "invalid_thresholds");
    Serial.println("battery_config rejected: invalid thresholds");
    return;
  }

  const BatteryConfig previousCfg = loadBatteryConfigSnapshot();
  portENTER_CRITICAL(&g_batteryStateMux);
  batteryCfg = nextCfg;
  portEXIT_CRITICAL(&g_batteryStateMux);

  if (!saveBatteryConfig()) {
    portENTER_CRITICAL(&g_batteryStateMux);
    batteryCfg = previousCfg;
    portEXIT_CRITICAL(&g_batteryStateMux);
    queueBmsConfigAck(configId, configVersion, configHash, "rejected", "nvs_save_failed");
    Serial.println("battery_config rejected: NVS save failed");
    return;
  }

  if (g_bmsTaskHandle != nullptr)
    xTaskNotifyGive(g_bmsTaskHandle);  // apply new limits immediately

  queueBmsConfigAck(configId, configVersion, configHash, "received", "esp32_nvs_saved");

  Serial.printf("battery_config mode=%s minSoc=%.0f maxSoc=%.0f maxV=%.1f maxI=%.1f ddp=%d\n",
                doc["mode"] | "?", nextCfg.minSoc, nextCfg.maxSoc,
                nextCfg.maxVoltage, nextCfg.maxCurrent, nextCfg.deepDischargeProtect);
}

// Xác nhận ảnh vừa nạp là "chạy được" — gọi sau khi đã nối lại AWS IoT, tức là
// WiFi + TLS + cert + MQTT đều còn hoạt động trên bản mới.
//
// GIỚI HẠN CẦN BIẾT: rollback tự động chỉ thật sự có tác dụng khi ảnh được
// build với CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE (không bật mặc định trong
// Arduino-ESP32). Không bật thì lệnh dưới đây vô hại nhưng cũng vô tác dụng:
// một ảnh hợp lệ nhưng hỏng logic sẽ ở lại cho tới khi nạp tay qua USB. Chốt
// chặn thật sự luôn hoạt động là kiểm SHA-256 trước khi kích hoạt phân vùng —
// ảnh tải lỗi thì không bao giờ được boot.
bool confirmFirmwareIfPending() {
  if (firmwareConfirmed) return true;

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) != ESP_OK) return false;
  if (state != ESP_OTA_IMG_PENDING_VERIFY) {
    firmwareConfirmed = true;
    return true;
  }

  if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
    firmwareConfirmed = true;
    Serial.println("firmware moi da duoc xac nhan (huy rollback)");
    return true;
  }
  Serial.println("khong the xac nhan firmware moi; chua bao OTA success");
  return false;
}

// ---------------------------------------------------------------------
// Dashboard cục bộ
// ---------------------------------------------------------------------
// Trang này được nhúng thẳng vào flash, không phụ thuộc CDN, Supabase hay file
// trên điện thoại. Sau khi kết nối AP, trình duyệt chỉ trao đổi với ESP32 qua
// HTTP nội bộ. JS poll API mỗi giây; 1 Hz đủ mượt cho màn hình giám sát và nhẹ
// hơn đáng kể so với giữ WebSocket/SSE trên một thiết bị còn phải chạy MQTT.
static const char LOCAL_DASHBOARD_HTML[] PROGMEM = R"SOLGRID(
<!doctype html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <meta name="theme-color" content="#07130f">
  <title>SolGrid Local</title>
  <style>
    :root{color-scheme:dark;--bg:#07130f;--panel:#10231c;--line:#244338;--muted:#91aa9f;--text:#effbf5;--green:#56e39f;--sun:#ffd166;--blue:#75c9ff;--red:#ff7b7b}
    *{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 85% -10%,#174b36 0,transparent 36%),var(--bg);color:var(--text);font-family:Inter,system-ui,-apple-system,"Segoe UI",sans-serif;min-height:100vh}
    main{width:min(960px,100%);margin:auto;padding:22px 16px 40px}.top{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;margin-bottom:24px}.brand{display:flex;align-items:center;gap:12px}.logo{display:grid;place-items:center;width:46px;height:46px;border-radius:14px;background:linear-gradient(145deg,#46d890,#186947);box-shadow:0 10px 30px #0008;font-size:24px}.brand h1{font-size:21px;margin:0}.sub{color:var(--muted);font-size:12px;margin-top:3px}.pill{border:1px solid var(--line);background:#0b1b15cc;border-radius:99px;padding:8px 11px;font-size:12px;white-space:nowrap}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--green);margin-right:6px;box-shadow:0 0 10px var(--green)}
    .hero{padding:22px;border:1px solid var(--line);border-radius:22px;background:linear-gradient(135deg,#122d22dd,#0b1c16dd);margin-bottom:14px}.hero-row{display:flex;justify-content:space-between;gap:18px;align-items:end}.eyebrow{font-size:11px;text-transform:uppercase;letter-spacing:.14em;color:var(--muted)}.power{font-size:48px;line-height:1;font-weight:750;letter-spacing:-.04em;margin-top:8px}.power small{font-size:17px;color:var(--sun);letter-spacing:0}.updated{color:var(--muted);font-size:12px;text-align:right}
    .grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.card{background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:17px;min-height:116px}.label{color:var(--muted);font-size:12px}.value{font-size:25px;font-weight:700;margin-top:9px}.unit{font-size:13px;color:var(--muted);font-weight:500}.meta{font-size:11px;color:var(--muted);margin-top:7px}.bar{height:7px;border-radius:10px;background:#07140f;margin-top:12px;overflow:hidden}.bar>i{display:block;height:100%;width:0;background:linear-gradient(90deg,#37c980,#8ff0bd);border-radius:inherit;transition:width .35s}.status{display:flex;justify-content:space-between;align-items:center;margin-top:14px;padding:15px 17px;border:1px solid var(--line);background:#0b1c16;border-radius:17px}.status-left{display:flex;align-items:center;gap:10px}.status-dot{width:10px;height:10px;border-radius:50%;background:var(--green)}.status-dot.bad{background:var(--red)}.status strong{font-size:13px}.status span{display:block;font-size:11px;color:var(--muted);margin-top:2px}.footer{text-align:center;color:#668077;font-size:11px;margin-top:22px}.error{display:none;background:#391b1b;border:1px solid #7b3434;color:#ffc2c2;border-radius:13px;padding:11px 14px;margin-bottom:12px;font-size:12px}
    .ota{display:none;margin-top:14px;padding:16px 17px;border:1px solid var(--line);border-radius:17px;background:#0b1c16}.ota.show{display:block}.ota-head{display:flex;justify-content:space-between;gap:16px;align-items:center}.ota-title{font-size:13px;font-weight:700}.ota-detail{font-size:11px;color:var(--muted);margin-top:3px}.ota-percent{font-size:18px;font-weight:750;color:var(--green)}.ota.failed .ota-percent{color:var(--red)}
    .settings{margin-top:14px;padding:19px;border:1px solid var(--line);border-radius:18px;background:var(--panel)}.settings h2{font-size:16px;margin:0 0 5px}.settings p{font-size:12px;color:var(--muted);margin:0 0 15px}.wifi-form{display:grid;grid-template-columns:1fr 1fr auto;gap:10px;align-items:end}.field label{display:block;font-size:11px;color:var(--muted);margin:0 0 6px}.field input{width:100%;border:1px solid var(--line);border-radius:11px;background:#07140f;color:var(--text);padding:11px 12px;outline:none}.field input:focus{border-color:var(--green)}button{border:0;border-radius:11px;padding:12px 16px;background:var(--green);color:#062116;font-weight:750;cursor:pointer}button:disabled{opacity:.55;cursor:wait}.secondary{background:transparent;color:var(--muted);border:1px solid var(--line);margin-left:7px}.wifi-message{font-size:12px;color:var(--muted);margin-top:12px;min-height:16px}.wifi-message.ok{color:var(--green)}.wifi-message.bad{color:var(--red)}
    @media(max-width:680px){main{padding-top:16px}.grid{grid-template-columns:repeat(2,1fr)}.hero-row{align-items:flex-start;flex-direction:column}.updated{text-align:left}.power{font-size:42px}.card{min-height:108px}.wifi-form{grid-template-columns:1fr}.wifi-actions{display:flex}}
    @media(max-width:390px){.grid{grid-template-columns:1fr}.pill{display:none}}
  </style>
</head>
<body><main>
  <div class="top"><div class="brand"><div class="logo">☀</div><div><h1>SolGrid Local</h1><div class="sub" id="station">Đang kết nối với thiết bị…</div></div></div><div class="pill"><i class="dot"></i>Kết nối cục bộ</div></div>
  <div class="error" id="error">Không nhận được dữ liệu. Hãy kiểm tra kết nối Wi‑Fi SolGrid.</div>
  <section class="hero"><div class="hero-row"><div><div class="eyebrow">Công suất điện mặt trời</div><div class="power"><span id="solar">—</span> <small>kW</small></div></div><div class="updated"><span id="clock">—</span><br>cập nhật trực tiếp từ ESP32</div></div></section>
  <section class="grid">
    <article class="card"><div class="label">Dung lượng pin</div><div class="value"><span id="soc">—</span><span class="unit"> %</span></div><div class="bar"><i id="socbar"></i></div></article>
    <article class="card"><div class="label">Điện áp pin</div><div class="value"><span id="bv">—</span><span class="unit"> V</span></div><div class="meta" id="batteryMode">—</div></article>
    <article class="card"><div class="label">Dòng pin</div><div class="value"><span id="ba">—</span><span class="unit"> A</span></div><div class="meta">Dương: đang sạc · Âm: đang xả</div></article>
    <article class="card"><div class="label">Điện áp PV</div><div class="value"><span id="pv">—</span><span class="unit"> V</span></div><div class="meta" id="stm">Liên kết Buck-Boost STM32F103</div></article>
    <article class="card"><div class="label">Dòng PV</div><div class="value"><span id="pa">—</span><span class="unit"> A</span></div><div class="meta">Đo trực tiếp qua MPPT</div></article>
    <article class="card"><div class="label">Công suất tải</div><div class="value"><span id="load">—</span><span class="unit"> W</span></div><div class="meta">Tải tức thời của hệ thống</div></article>
    <article class="card"><div class="label">Nhiệt độ pin</div><div class="value"><span id="temp">—</span><span class="unit"> °C</span></div><div class="meta">Nhiệt độ pack pin</div></article>
    <article class="card"><div class="label">Thời gian hoạt động</div><div class="value" id="uptime">—</div><div class="meta" id="firmware">Firmware —</div></article>
    <article class="card"><div class="label">Thiết bị đang truy cập</div><div class="value"><span id="clients">—</span><span class="unit"> máy</span></div><div class="meta">Kết nối vào Wi‑Fi local</div></article>
  </section>
  <section class="status"><div class="status-left"><i class="status-dot" id="cloudDot"></i><div><strong id="cloudTitle">Đang kiểm tra cloud…</strong><span id="cloudText">Dashboard local vẫn hoạt động khi cloud ngoại tuyến</span></div></div><div class="pill" id="rssi">— dBm</div></section>
  <section class="ota" id="otaPanel"><div class="ota-head"><div><div class="ota-title" id="otaTitle">Đang cập nhật firmware</div><div class="ota-detail" id="otaDetail">Đang chuẩn bị…</div></div><div class="ota-percent" id="otaPercent">0%</div></div><div class="bar"><i id="otaBar"></i></div></section>
  <section class="settings">
    <h2>Cài đặt Wi-Fi Internet</h2>
    <p>Kết nối ESP32 với router 2.4 GHz. Cấu hình được lưu trên thiết bị và vẫn còn sau khi khởi động lại.</p>
    <form class="wifi-form" id="wifiForm">
      <div class="field"><label for="wifiSsid">Tên mạng Wi-Fi (SSID)</label><input id="wifiSsid" name="ssid" maxlength="32" required autocomplete="off" placeholder="Nhập tên mạng"></div>
      <div class="field"><label for="wifiPassword">Mật khẩu</label><input id="wifiPassword" name="password" type="password" minlength="8" maxlength="63" required autocomplete="new-password" placeholder="Từ 8 đến 63 ký tự"></div>
      <div class="wifi-actions"><button id="wifiSave" type="submit">Lưu &amp; kết nối</button><button class="secondary" id="wifiForget" type="button">Xóa</button></div>
    </form>
    <div class="wifi-message" id="wifiMessage">Đang đọc cấu hình Wi-Fi…</div>
  </section>
  <div class="footer">Dữ liệu đi trực tiếp từ thiết bị đến máy của bạn, không qua Internet.</div>
</main>
<script>
  const $=id=>document.getElementById(id), n=(v,d=1)=>Number.isFinite(Number(v))?Number(v).toFixed(d):'—';
  const duration=s=>{s=Number(s)||0;const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return d?`${d} ngày ${h} giờ`:`${h} giờ ${m} phút`};
  const renderOta=d=>{const status=d.fw_status||'idle',progress=Number.isFinite(Number(d.fw_progress))?Math.max(0,Math.min(100,Math.round(Number(d.fw_progress)))):0,visible=['downloading','applying','failed'].includes(status),labels={downloading:'Đang tải và ghi firmware',applying:'Đang xác minh và khởi động lại',failed:'Cập nhật firmware thất bại'};const panel=$('otaPanel');panel.classList.toggle('show',visible);panel.classList.toggle('failed',status==='failed');$('otaTitle').textContent=labels[status]||'Cập nhật firmware';$('otaDetail').textContent=d.fw_status_detail||'Thiết bị đang xử lý bản cập nhật';$('otaPercent').textContent=`${progress}%`;$('otaBar').style.width=`${progress}%`};
  let misses=0;
  async function refresh(){try{const c=new AbortController(),t=setTimeout(()=>c.abort(),2500);const r=await fetch('/api/telemetry',{cache:'no-store',signal:c.signal});clearTimeout(t);if(!r.ok)throw Error(r.status);const d=await r.json();misses=0;$('error').style.display='none';$('station').textContent=`${d.station} · ${d.device}`;$('solar').textContent=n(d.solar_kw,3);$('soc').textContent=n(d.battery_pct,0);$('socbar').style.width=`${Math.max(0,Math.min(100,d.battery_pct||0))}%`;$('bv').textContent=n(d.battery_voltage,1);$('ba').textContent=n(d.battery_current,1);$('pv').textContent=n(d.pv_voltage,2);$('pa').textContent=n(d.pv_current,2);$('load').textContent=n(d.load_w,0);$('temp').textContent=n(d.temp_c,1);$('uptime').textContent=duration(d.uptime_s);$('firmware').textContent=`Firmware ${d.fw_version} · boot #${d.boot_count}`;$('clients').textContent=d.ap_clients;$('clock').textContent=new Date().toLocaleTimeString('vi-VN');$('stm').textContent=d.stm32_link?'Buck-Boost STM32F103 đang phản hồi':'Mất liên kết Buck-Boost STM32F103';const reasons={ok:'Bình thường',full:'Pin đầy',overvoltage:'Quá áp',deep_discharge:'Chống xả sâu'};$('batteryMode').textContent=`Sạc ${d.charge_enabled?'bật':'ngắt'} · Xả ${d.discharge_enabled?'bật':'ngắt'} · ${reasons[d.protect_reason]||d.protect_reason}`;const online=d.mqtt_connected;$('cloudDot').classList.toggle('bad',!online);$('cloudTitle').textContent=online?'Cloud đang kết nối':'Đang hoạt động ngoại tuyến';$('cloudText').textContent=online?'Dữ liệu cũng đang được đồng bộ lên cloud':'Dữ liệu hiện chỉ truyền trong mạng local';$('rssi').textContent=d.wifi_uplink?`${d.rssi} dBm`:'Không có Internet';renderOta(d)}catch(e){if(++misses>2)$('error').style.display='block'}}
  const wifiForm=$('wifiForm'),wifiMessage=$('wifiMessage'),wifiSave=$('wifiSave');
  async function loadWifiStatus(){try{const r=await fetch('/api/wifi',{cache:'no-store'}),d=await r.json();if(d.configured&&!$('wifiSsid').value)$('wifiSsid').value=d.ssid||'';wifiMessage.className='wifi-message '+(d.connected?'ok':'');wifiMessage.textContent=d.connected?`Đã kết nối ${d.ssid} · ${d.ip}`:(d.configured?`Đã lưu ${d.ssid}, đang chờ kết nối…`:'Chưa có mạng Internet được cài đặt.')}catch(e){wifiMessage.className='wifi-message bad';wifiMessage.textContent='Không đọc được trạng thái Wi-Fi.'}}
  wifiForm.addEventListener('submit',async e=>{e.preventDefault();wifiSave.disabled=true;wifiMessage.className='wifi-message';wifiMessage.textContent='Đang lưu cấu hình…';try{const body=new URLSearchParams(new FormData(wifiForm)),r=await fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}),d=await r.json();if(!r.ok)throw Error(d.message||`HTTP ${r.status}`);$('wifiPassword').value='';wifiMessage.className='wifi-message ok';wifiMessage.textContent=d.message;setTimeout(loadWifiStatus,1500)}catch(e){wifiMessage.className='wifi-message bad';wifiMessage.textContent=e.message||'Không thể lưu cấu hình.'}finally{wifiSave.disabled=false}});
  $('wifiForget').addEventListener('click',async()=>{if(!confirm('Xóa mạng Wi-Fi Internet đã lưu?'))return;try{const r=await fetch('/api/wifi/forget',{method:'POST'}),d=await r.json();if(!r.ok)throw Error(d.message||`HTTP ${r.status}`);wifiForm.reset();wifiMessage.className='wifi-message';wifiMessage.textContent=d.message}catch(e){wifiMessage.className='wifi-message bad';wifiMessage.textContent=e.message||'Không thể xóa cấu hình.'}});
  refresh();loadWifiStatus();setInterval(refresh,1000);setInterval(loadWifiStatus,5000);
</script></body></html>
)SOLGRID";

void addLocalResponseHeaders() {
  // Cho phép bản Electron/PWA gọi cùng API khi chạy ở chế độ local. Trang
  // nhúng ở trên là phương án luôn hoạt động, kể cả browser chặn mixed-content.
  localServer.sendHeader("Access-Control-Allow-Origin", "*");
  localServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  localServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  // Chromium/Electron mới có thể gửi Private Network Access preflight khi
  // app loopback gọi sang địa chỉ LAN của ESP32.
  localServer.sendHeader("Access-Control-Allow-Private-Network", "true");
  localServer.sendHeader("Cache-Control", "no-store, max-age=0");
}

void serveLocalDashboard() {
  addLocalResponseHeaders();
  localServer.send_P(200, "text/html; charset=utf-8", LOCAL_DASHBOARD_HTML);
}

void serveLocalTelemetry() {
  StaticJsonDocument<768> doc;
  const PvStateSnapshot pv = loadPvState();
  const ProtectionStateSnapshot protection = loadProtectionState();
  const OtaRuntimeSnapshot ota = loadOtaRuntimeSnapshot();
  const BatteryReading r = protection.reading;

  doc["mode"] = "local";
  doc["station"] = STATION_SLUG;
  doc["device"] = THING_NAME;
  doc["fw_version"] = FW_VERSION;
  doc["fw_status"] = ota.status;
  if (ota.detail[0]) doc["fw_status_detail"] = ota.detail;
  if (ota.progress >= 0) doc["fw_progress"] = ota.progress;
  doc["ap_ssid"] = AP_SSID;
  doc["solar_kw"] = roundf(pv.solarKw * 1000.0f) / 1000.0f;
  doc["pv_voltage"] = roundf(pv.tlm.vin_mV / 10.0f) / 100.0f;
  doc["pv_current"] = roundf(pv.tlm.iin_mA / 10.0f) / 100.0f;
  // Giữ key stm32_link để tương thích dashboard/app hiện tại.
  doc["stm32_link"] = pv.linkOk;
  doc["battery_pct"] = (int)(r.soc + 0.5f);
  doc["battery_voltage"] = r.voltage;
  doc["battery_current"] = r.current;
  doc["load_w"] = simulatedLoadW();
  doc["temp_c"] = r.tempC;
  doc["charge_enabled"] = protection.chargeEnabled;
  doc["discharge_enabled"] = protection.dischargeEnabled;
  doc["protect_reason"] = protection.reason;
  doc["uptime_s"] = uptimeSeconds();
  doc["boot_count"] = bootCount;
  doc["ap_clients"] = WiFi.softAPgetStationNum();
  doc["wifi_uplink"] = WiFi.status() == WL_CONNECTED;
  doc["mqtt_connected"] = g_mqttOnline;
  doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;

  String body;
  body.reserve(640);
  serializeJson(doc, body);
  addLocalResponseHeaders();
  localServer.send(200, "application/json; charset=utf-8", body);
}

void sendWiFiApiMessage(int statusCode, bool ok, const char *message) {
  StaticJsonDocument<192> doc;
  doc["ok"] = ok;
  doc["message"] = message;
  String body;
  serializeJson(doc, body);
  addLocalResponseHeaders();
  localServer.send(statusCode, "application/json; charset=utf-8", body);
}

void serveWiFiStatus() {
  const WiFiCredentials credentials = loadWiFiCredentialsSnapshot();
  const bool connected = WiFi.status() == WL_CONNECTED;
  StaticJsonDocument<256> doc;
  doc["configured"] = credentials.configured;
  doc["connected"] = connected;
  doc["ssid"] = credentials.configured ? credentials.ssid : "";
  doc["ip"] = connected ? WiFi.localIP().toString() : "";
  doc["rssi"] = connected ? WiFi.RSSI() : 0;

  String body;
  serializeJson(doc, body);
  addLocalResponseHeaders();
  localServer.send(200, "application/json; charset=utf-8", body);
}

void saveWiFiFromPortal() {
  if (!localServer.hasArg("ssid") || !localServer.hasArg("password")) {
    sendWiFiApiMessage(400, false, "Thiếu tên mạng hoặc mật khẩu.");
    return;
  }

  const String ssid = localServer.arg("ssid");
  const String password = localServer.arg("password");
  if (ssid.length() < 1 || ssid.length() > 32) {
    sendWiFiApiMessage(400, false, "SSID phải dài từ 1 đến 32 byte.");
    return;
  }
  if (password.length() < 8 || password.length() > 63) {
    sendWiFiApiMessage(400, false, "Mật khẩu phải dài từ 8 đến 63 ký tự.");
    return;
  }
  if (!saveWiFiCredentials(ssid.c_str(), password.c_str())) {
    sendWiFiApiMessage(500, false, "Không thể lưu cấu hình vào bộ nhớ.");
    return;
  }

  Serial.printf("WiFi config saved for SSID: %s\n", ssid.c_str());
  sendWiFiApiMessage(200, true, "Đã lưu. ESP32 đang thử kết nối mạng mới.");
}

void forgetWiFiFromPortal() {
  if (!clearWiFiCredentials()) {
    sendWiFiApiMessage(500, false, "Không thể xóa cấu hình Wi-Fi.");
    return;
  }
  Serial.println("WiFi config cleared from NVS");
  sendWiFiApiMessage(200, true, "Đã xóa cấu hình Wi-Fi Internet.");
}

void startLocalServer() {
  IPAddress apIp = WiFi.softAPIP();
  // Mọi hostname đều trỏ về ESP32, giúp Android/iOS/Windows nhận ra captive
  // portal và đề nghị mở trang local ngay sau khi vào AP.
  captiveDns.start(53, "*", apIp);

  localServer.on("/", HTTP_GET, serveLocalDashboard);
  localServer.on("/api/telemetry", HTTP_GET, serveLocalTelemetry);
  localServer.on("/api/telemetry", HTTP_OPTIONS, []() {
    addLocalResponseHeaders();
    localServer.send(204);
  });
  localServer.on("/api/wifi", HTTP_GET, serveWiFiStatus);
  localServer.on("/api/wifi", HTTP_POST, saveWiFiFromPortal);
  localServer.on("/api/wifi", HTTP_OPTIONS, []() {
    addLocalResponseHeaders();
    localServer.send(204);
  });
  localServer.on("/api/wifi/forget", HTTP_POST, forgetWiFiFromPortal);
  localServer.on("/api/wifi/forget", HTTP_OPTIONS, []() {
    addLocalResponseHeaders();
    localServer.send(204);
  });
  // Endpoint thăm dò captive portal phổ biến.
  localServer.on("/generate_204", HTTP_GET, serveLocalDashboard);       // Android
  localServer.on("/hotspot-detect.html", HTTP_GET, serveLocalDashboard); // Apple
  localServer.on("/connecttest.txt", HTTP_GET, serveLocalDashboard);    // Windows
  localServer.on("/ncsi.txt", HTTP_GET, serveLocalDashboard);           // Windows
  localServer.onNotFound(serveLocalDashboard);
  localServer.begin();
  Serial.printf("Local dashboard: http://%s\n", apIp.toString().c_str());
}

// SoftAP: mạng cục bộ thiết bị tự phát để xem/cài đặt tại chỗ khi trạm mất
// internet. Bật cùng lúc với STA (WIFI_AP_STA) nên AP vẫn còn khi uplink chết.
//
// Đánh đổi cần biết: AP và STA dùng CHUNG một radio và bị ép về cùng kênh —
// khi có client bám vào AP, thông lượng/độ ổn định của đường MQTT giảm. Chấp
// nhận được vì AP chủ yếu để dùng lúc mất mạng, nhưng nếu về sau thấy MQTT
// chập chờn thì chuyển sang bật AP theo yêu cầu (nút nhấn / sau N phút mất
// kết nối) thay vì phát thường trực.
void startAccessPoint() {
  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("SoftAP %s: ssid=%s ip=%s\n", ok ? "up" : "FAILED", AP_SSID,
                WiFi.softAPIP().toString().c_str());
}

void startWiFiUplink() {
  // WiFi.begin() là bất đồng bộ. Tuyệt đối không chờ trong while ở đây: nếu
  // router/internet mất thì LocalServerTask vẫn phải phục vụ dashboard local.
  // Không gọi WiFi.mode(WIFI_STA) vì sẽ tắt AP vừa bật.
  const WiFiCredentials credentials = loadWiFiCredentialsSnapshot();
  if (!credentials.configured) {
    Serial.println("WiFi uplink is not configured; open the local portal");
    return;
  }

  WiFi.setAutoReconnect(true);
  WiFi.begin(credentials.ssid, credentials.password);
  Serial.printf("WiFi uplink connecting to %s (non-blocking)\n",
                credentials.ssid);
}

bool networkTimeReady() {
  return time(nullptr) >= TLS_MIN_VALID_EPOCH;
}

void startNetworkTimeSync() {
  if (networkTimeReady() || networkTimeSyncStarted) return;
  // TLS validity checks need a sane wall clock after power-on. configTime()
  // is non-blocking, so the captive portal remains responsive.
  configTime(0, 0, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
  networkTimeSyncStarted = true;
  lastTimeWaitLog = 0;
  Serial.println("NTP time synchronization started");
}

// PubSubClient chính chủ nhận một kích thước buffer; TBPubSubClient (một fork
// tương thích thường được Arduino IDE chọn khi cả hai cùng cung cấp header
// PubSubClient.h) tách receive/send thành hai tham số. Hai overload SFINAE này
// giữ sketch biên dịch được với cả hai mà không phụ thuộc macro riêng của fork.
template <typename T>
auto setMqttBufferCompat(T &client, uint16_t size, int)
  -> decltype(((T *)nullptr)->setBufferSize(0, 0), bool()) {
  return client.setBufferSize(size, size);
}

template <typename T>
auto setMqttBufferCompat(T &client, uint16_t size, long)
  -> decltype(((T *)nullptr)->setBufferSize(0), bool()) {
  return client.setBufferSize(size);
}

void configureAWS() {
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(DEVICE_CERT);
  net.setPrivateKey(DEVICE_PRIVATE_KEY);
  net.setTimeout(5000);
  net.setHandshakeTimeout(5);
  mqtt.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  // #define MQTT_MAX_PACKET_SIZE trong sketch không tác dụng với PubSubClient
  // >=2.8 (thư viện biên dịch riêng, vẫn dùng 256B) — phải set lúc runtime.
  // AWS IoT Jobs injects a long presigned S3 URL into the incoming document.
  setMqttBufferCompat(mqtt, 3072, 0);
  mqtt.setSocketTimeout(5);
  mqtt.setCallback(onCommand);
}

bool connectAWSOnce() {
  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.print("AWS IoT");
  if (!mqtt.connect(THING_NAME)) {
    Serial.printf(" failed rc=%d (local dashboard remains available)\n", mqtt.state());
    return false;
  }

  Serial.println(" connected");
  // Kết nối mới → báo lại cấu hình AP + phiên bản ở bản tin kế tiếp.
  apReported = false;
  fwReported = false;
  // Nối lại được AWS IoT nghĩa là ảnh đang chạy hoạt động đủ tốt.
  const bool firmwareHealthy = confirmFirmwareIfPending();
  char commandTopic[64];
  snprintf(commandTopic, sizeof(commandTopic), "solgrid/%s/command", THING_NAME);
  mqtt.subscribe(commandTopic);

  char jobsNotifyNext[160];
  char jobsStartAccepted[176];
  char jobsStartRejected[176];
  snprintf(jobsNotifyNext, sizeof(jobsNotifyNext),
           "$aws/things/%s/jobs/notify-next", THING_NAME);
  snprintf(jobsStartAccepted, sizeof(jobsStartAccepted),
           "$aws/things/%s/jobs/start-next/accepted", THING_NAME);
  snprintf(jobsStartRejected, sizeof(jobsStartRejected),
           "$aws/things/%s/jobs/start-next/rejected", THING_NAME);
  mqtt.subscribe(jobsNotifyNext);
  mqtt.subscribe(jobsStartAccepted);
  mqtt.subscribe(jobsStartRejected);
  if (firmwareHealthy) reportOtaCompletionAfterReconnect();
  awsStartNextRequested = true;  // discover jobs queued while offline
  Serial.printf("clientId=[%s]\ncmdTopic=[solgrid/%s/command]\n", THING_NAME, THING_NAME);
  return true;
}

void maintainCloudConnections() {
  unsigned long now = millis();

  // The HTTP task only persists new credentials and posts this request. All
  // STA driver operations stay here in CloudTask, preventing cross-task calls
  // to WiFi.begin()/disconnect().
  if (takeWiFiReconfigureRequest()) {
    if (mqtt.connected()) mqtt.disconnect();
    g_mqttOnline = false;
    wifiWasConnected = false;
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);

    const WiFiCredentials credentials = loadWiFiCredentialsSnapshot();
    wifiConnectScheduled = credentials.configured;
    wifiConnectAt = now + WIFI_RECONFIGURE_DELAY_MS;
    if (credentials.configured)
      Serial.printf("WiFi uplink switching to %s\n", credentials.ssid);
    else
      Serial.println("WiFi uplink disabled; waiting for local provisioning");
  }

  if (wifiConnectScheduled) {
    if ((long)(now - wifiConnectAt) >= 0) {
      wifiConnectScheduled = false;
      startWiFiUplink();
    }
    return;
  }

  // A disconnect is asynchronous. After "forget", do not let a short-lived
  // WL_CONNECTED state reconnect MQTT with credentials that no longer exist.
  if (!loadWiFiCredentialsSnapshot().configured) return;

  bool wifiConnected = WiFi.status() == WL_CONNECTED;

  if (!wifiConnected) {
    if (wifiWasConnected) {
      wifiWasConnected = false;
      if (mqtt.connected()) mqtt.disconnect();
      Serial.println("WiFi uplink lost; switched to local-only mode");
    }
    return;
  }

  if (!wifiWasConnected) {
    wifiWasConnected = true;
    lastMqttAttempt = now - MQTT_RETRY_INTERVAL_MS; // thử MQTT ngay
    Serial.printf("WiFi uplink connected: %s\n", WiFi.localIP().toString().c_str());
    startNetworkTimeSync();
  }

  if (!mqtt.connected() && now - lastMqttAttempt >= MQTT_RETRY_INTERVAL_MS) {
    lastMqttAttempt = now;
    if (!networkTimeReady()) {
      if (lastTimeWaitLog == 0 || now - lastTimeWaitLog >= 10000) {
        lastTimeWaitLog = now;
        Serial.println("AWS IoT waiting for valid NTP time");
      }
      return;
    }
    connectAWSOnce();
  }
}

void publishTelemetry() {
  // 1536: bản tin nặng nhất là bản đầu sau reconnect — số đo + PV + chẩn đoán
  // phần cứng + trạng thái bảo vệ + ap_ssid/ap_password + fw_version +
  // fw_status/detail. (Trước là 768, hết chỗ khi thêm 4 trường chẩn đoán:
  // ArduinoJson im lặng bỏ trường khi tràn, nên thiếu chỗ = mất dữ liệu chứ
  // không có lỗi nào báo ra.)
  StaticJsonDocument<1536> doc;
  const PvStateSnapshot pv = loadPvState();
  const ProtectionStateSnapshot protection = loadProtectionState();
  const BatteryReading r = protection.reading;
  const BmsConfigAckSnapshot configAck = loadBmsConfigAckSnapshot();

  // Số liệu PV do STM32F103C8T6 buck-boost đo/điều khiển, lấy qua UART (pollBuckboost). Làm tròn TRƯỚC khi gửi:
  // solar_kw là cột numeric nên chấp nhận số lẻ, nhưng gửi đủ 6-7 chữ số thập
  // phân của float chỉ làm payload dài thêm mà không thêm thông tin thật.
  doc["solar_kw"] = roundf(pv.solarKw * 1000.0f) / 1000.0f;
  // pv_voltage/pv_current/stm32_link CHƯA có cột riêng trong `telemetry` nên
  // ingest-telemetry gom chúng vào jsonb `extra` — ghi được ngay, không cần
  // migration. Muốn vẽ đồ thị theo chúng thì phải nâng thành cột thật (thêm
  // vào NUMERIC_FIELDS của ingest-telemetry).
  doc["pv_voltage"] = roundf(pv.tlm.vin_mV / 10.0f) / 100.0f;
  doc["pv_current"] = roundf(pv.tlm.iin_mA / 10.0f) / 100.0f;
  // Phân biệt "PV đúng bằng 0" (đêm) với "không hỏi được STM32": thiếu cờ này
  // thì mất link trông y hệt trời tối.
  doc["stm32_link"] = pv.linkOk;

  doc["battery_pct"] = (int)(r.soc + 0.5f);
  doc["battery_voltage"] = r.voltage;
  doc["battery_current"] = r.current;
  doc["load_w"] = simulatedLoadW();
  // Nhiệt độ PACK PIN (không phải nhiệt độ lõi MCU — đại lượng đó đã bỏ hẳn ở
  // migration 0029, firmware không còn gửi `mcu_temp_c` nữa).
  doc["temp_c"] = r.tempC;
  doc["rssi"] = WiFi.RSSI();

  // Chẩn đoán phần cứng → DevConsole → Tổng quan thiết bị (migration 0017).
  doc["uptime_s"] = uptimeSeconds();
  doc["boot_count"] = bootCount;

  doc["charge_enabled"] = protection.chargeEnabled;
  doc["discharge_enabled"] = protection.dischargeEnabled;
  doc["protect_reason"] = protection.reason;

  // Báo cấu hình AP lên cloud để dashboard hiển thị đúng mạng đang phát thật.
  // Chỉ gửi ở bản tin ĐẦU TIÊN sau mỗi lần (re)connect, không gửi mỗi 10s:
  // giá trị gần như không đổi, và đây là mật khẩu — gửi lặp vô ích chỉ tốn
  // băng thông và tăng số lần nó đi qua đường truyền. Reconnect sẽ tự báo
  // lại, nên dữ liệu tự phục hồi nếu cloud mất bản ghi.
  if (!apReported) {
    doc["ap_ssid"] = AP_SSID;
    doc["ap_password"] = AP_PASSWORD;
  }

  // Phiên bản đang chạy — cùng lý do "chỉ gửi sau mỗi lần reconnect" như AP.
  if (!fwReported) doc["fw_version"] = FW_VERSION;
  // Tiến trình OTA, chỉ có khi vừa xảy ra chuyện gì đó đáng báo.
  if (pendingFwStatus) {
    doc["fw_status"] = pendingFwStatus;
    if (pendingFwDetail[0]) doc["fw_status_detail"] = pendingFwDetail;
    if (pendingFwProgress >= 0) doc["fw_progress"] = pendingFwProgress;
  }

  if (configAck.pending) {
    JsonObject ack = doc.createNestedObject("bms_config_ack");
    ack["status"] = configAck.status;
    ack["config_id"] = configAck.configId;
    ack["config_version"] = configAck.configVersion;
    ack["config_hash"] = configAck.configHash;
    if (configAck.detail[0]) ack["detail"] = configAck.detail;
  }

  // Khớp StaticJsonDocument ở trên — serializeJson cắt bớt khi buffer nhỏ hơn,
  // và một payload JSON cụt sẽ bị ingest-telemetry trả 400 invalid_json.
  char payload[1536];
  size_t n = serializeJson(doc, payload);

  char topic[64];
  snprintf(topic, sizeof(topic), "solgrid/%s/telemetry", STATION_SLUG);

  // Ép sang overload (topic, uint8_t*, len): nếu truyền (char*, size_t) sẽ
  // trúng overload (topic, payload, retained) → retained=true, AWS IoT từ chối
  // vì policy không cấp iot:RetainPublish (AUTHORIZATION_FAILURE).
  bool ok = mqtt.publish(topic, (const uint8_t *)payload, (unsigned int)n);
  // Chỉ đánh dấu đã báo khi publish THÀNH CÔNG — nếu thất bại, bản tin sau vẫn
  // kèm lại AP thay vì mất luôn cho tới lần reconnect kế tiếp.
  if (ok) {
    apReported = true;
    fwReported = true;
    if (configAck.pending) clearBmsConfigAckIfUnchanged(configAck);
    // Trạng thái OTA cũng chỉ xoá khi đã gửi được, nếu không một lần publish
    // hỏng sẽ nuốt mất thông báo 'failed' và dashboard treo ở 'downloading'.
    pendingFwStatus = nullptr;
    pendingFwDetail[0] = '\0';
    pendingFwProgress = -1;
  }
  Serial.printf("pubTopic=[solgrid/%s/telemetry]\n", STATION_SLUG);
  Serial.printf("publish %s -> %s (%s)\n", topic, payload, ok ? "ok" : "fail");
}

/* =========================================================
 *  FREERTOS TASKS
 * ========================================================= */

static void BuckboostTask(void *parameter) {
  (void)parameter;

  TickType_t nextWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(BUCKBOOST_POLL_INTERVAL_MS);

  for (;;) {
    /* Serial1 has one runtime owner: this task. */
    pollBuckboost();
    vTaskDelayUntil(&nextWake, period);
  }
}

static void BMSTask(void *parameter) {
  (void)parameter;

  for (;;) {
    /* Serial2 has exactly one runtime owner: BMSTask. */
    pollBms();
    applyProtection();

    /* Cloud battery_config can wake this task early; otherwise poll BMS at 1 Hz. */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BMS_POLL_INTERVAL_MS));
  }
}

static void LocalServerTask(void *parameter) {
  (void)parameter;

  for (;;) {
    /* WebServer + DNSServer have one runtime owner: this task.  Therefore a
     * TLS reconnect or an OTA download can no longer freeze the local UI. */
    captiveDns.processNextRequest();
    localServer.handleClient();
    vTaskDelay(LOCAL_TASK_IDLE_TICKS);
  }
}

static void CloudTask(void *parameter) {
  (void)parameter;

  for (;;) {
    /* PubSubClient has exactly one owner: CloudTask. */
    maintainCloudConnections();

    if (mqtt.connected())
      mqtt.loop();

    g_mqttOnline = mqtt.connected();

    /* Safe here: onCommand() has already returned to mqtt.loop(). */
    flushFwStatus();
    flushBmsConfigAck();
    flushAwsJobStatus();
    clearOtaCompletionMarkerIfReported();
    flushAwsStartNextRequest();

    const unsigned long now = millis();
    if (mqtt.connected() && now - lastPublish >= PUBLISH_INTERVAL_MS) {
      lastPublish = now;
      publishTelemetry();
    }

    /* Keep the original OTA behavior/functionality, but it now blocks only
     * CloudTask.  STM32 polling, protection, and local HTTP continue running.
     * A later revision can move OTA into its own task if MQTT keepalive during
     * a long download becomes a requirement. */
    if (otaJob.pending) {
      // static: OtaJob ~1.2 KB (url[1024]) — không sao chép lên stack CloudTask.
      static OtaJob job;
      job = otaJob; // mqtt.loop() may queue another job during download
      otaJob.pending = false;
      runOtaUpdate(job);   // success reboots; failure returns here
      g_mqttOnline = mqtt.connected();
    }

    vTaskDelay(CLOUD_TASK_IDLE_TICKS);
  }
}

static void createTaskOrHalt(TaskFunction_t fn,
                             const char *name,
                             uint32_t stackBytes,
                             UBaseType_t priority,
                             TaskHandle_t *handle,
                             BaseType_t core) {
  const BaseType_t ok = xTaskCreatePinnedToCore(
      fn, name, stackBytes, nullptr, priority, handle, core);

  if (ok != pdPASS) {
    Serial.printf("FATAL: cannot create %s\n", name);
    for (;;) delay(1000);
  }
}

/* =========================================================
 *  SETUP
 * ========================================================= */

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("SolGrid %s %s / FreeRTOS architecture\n", FW_BOARD, FW_VERSION);
  Serial.println("OTA: AWS IoT Jobs + Supabase Storage HTTPS enabled");
  loadOtaCompletionMarker();

  if (CHARGE_RELAY_PIN != 255) pinMode(CHARGE_RELAY_PIN, OUTPUT);
  if (DISCHARGE_RELAY_PIN != 255) pinMode(DISCHARGE_RELAY_PIN, OUTPUT);

  loadBootCount();
  Serial.printf("boot #%u\n", bootCount);

  /* One-time peripheral initialization. Runtime ownership after scheduler:
   * Serial1 -> BuckboostTask, Serial2 -> BMSTask. */
  Serial1.begin(BUCKBOOST_UART_BAUD, SERIAL_8N1,
                BUCKBOOST_UART_RX_PIN, BUCKBOOST_UART_TX_PIN);
  Serial2.begin(BMS_UART_BAUD, SERIAL_8N1,
                BMS_UART_RX_PIN, BMS_UART_TX_PIN);
  Serial.printf("[BMS] UART2 initialized: %lu baud, TX=GPIO%d, RX=GPIO%d\n",
                (unsigned long)BMS_UART_BAUD, BMS_UART_TX_PIN, BMS_UART_RX_PIN);
  Serial.println("[BMS] Waiting for STM32F103 BMS...");

  loadBatteryConfig();
  applyProtection();  // starts fail-safe OFF until first valid BMS frame
  const bool hasWiFiConfig = loadWiFiCredentials();
  Serial.printf("WiFi uplink config: %s\n",
                hasWiFiConfig ? "loaded from NVS" : "not configured");

  /* Network services are configured once; runtime processing is split across
   * LocalServerTask and CloudTask. */
  startAccessPoint();
  startLocalServer();
  configureAWS();
  startWiFiUplink();

  createTaskOrHalt(
      BuckboostTask,
      "BuckboostTask",
      BUCKBOOST_TASK_STACK_BYTES,
      BUCKBOOST_TASK_PRIORITY,
      &g_buckboostTaskHandle,
      APP_DEVICE_CORE);

  createTaskOrHalt(
      BMSTask,
      "BMSTask",
      BMS_TASK_STACK_BYTES,
      BMS_TASK_PRIORITY,
      &g_bmsTaskHandle,
      APP_DEVICE_CORE);

  createTaskOrHalt(
      LocalServerTask,
      "LocalServerTask",
      LOCAL_TASK_STACK_BYTES,
      LOCAL_TASK_PRIORITY,
      &g_localTaskHandle,
      APP_NETWORK_CORE);

  createTaskOrHalt(
      CloudTask,
      "CloudTask",
      CLOUD_TASK_STACK_BYTES,
      CLOUD_TASK_PRIORITY,
      &g_cloudTaskHandle,
      APP_NETWORK_CORE);

  Serial.println("RTOS tasks started");

  /* setup() runs inside Arduino's loopTask.  All application work now lives in
   * explicit FreeRTOS tasks, so retire loopTask instead of busy-spinning it. */
  vTaskDelete(nullptr);
}

void loop() {
  /* Intentionally unused. setup() deletes Arduino loopTask after creating the
   * application tasks above. */
}
