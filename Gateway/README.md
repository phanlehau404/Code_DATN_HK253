
<a href="https://github.com/ACLAB-HCMUT"><img src="https://raw.githubusercontent.com/ACLAB-HCMUT/Common/main/Assets/ACLAB_IMG_1.png" alt="ACLAB logo" title="ACLAB" align="right" height="100" /></a>

[![PlatformIO Registry](https://badges.registry.platformio.org/packages/luos/library/luos_engine.svg)](https://registry.platformio.org/libraries/luos/luos_engine)

<br>

# Starter Kit for IoT Projects at ACLAB-HCMUT

## Libraries:
- ESPAsyncWebServer
- SPIFFs
- Adafruit_MQTT_Library

## Manual:
1. Enter Bootloader by Hold Boot -> Hold Reset -> Release Reset - > Release Boot.
2. Upload Filesystem Image Using Platformio.
3. Flash Firmware.
4. Exit Bootloader by Pressing Reset.
5. To build and upload filesystem image (SPIFF): Platformio Extension -> "Build Filesystem Image" -> "Upload Filesystem Image"

## Cài đặt Wi-Fi Internet

1. Sau khi ESP32-S3 khởi động, kết nối điện thoại hoặc máy tính vào mạng Wi-Fi
   local được khai báo bằng `AP_SSID` trong `include/secrets.h`.
2. Mở `http://192.168.4.1` (captive portal có thể tự mở trang này).
3. Trong mục **Cài đặt Wi-Fi Internet**, nhập SSID và mật khẩu của router
   2.4 GHz, sau đó chọn **Lưu & kết nối**.
4. ESP32 lưu cấu hình vào NVS flash và tự dùng lại sau mỗi lần khởi động.

Nút **Xóa** trên trang local sẽ xóa cấu hình uplink nhưng không tắt mạng Wi-Fi
local của ESP32. Thao tác **Erase Flash** trong PlatformIO cũng xóa cấu hình NVS.

## Cập nhật OTA qua AWS

Xem [docs/AWS_IOT_OTA.md](docs/AWS_IOT_OTA.md) để cấu hình AWS IoT Jobs, S3,
policy cho certificate và tạo job OTA.
