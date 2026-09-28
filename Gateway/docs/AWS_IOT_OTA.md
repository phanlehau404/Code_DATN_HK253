# OTA qua AWS IoT Jobs

Firmware trong `src/main.cpp` dùng AWS IoT Jobs làm control plane và Amazon S3
làm nơi chứa file `.bin`. Thiết bị nhận presigned URL từ job document, kiểm tra
SHA-256 trước khi kích hoạt phân vùng OTA, reboot, kết nối lại AWS rồi mới đánh
dấu job `SUCCEEDED`.

## 1. Điều kiện phía thiết bị

- Cấu hình Wi-Fi uplink tại `http://192.168.4.1`. Dòng
  `WiFi uplink config: not configured` nghĩa là thiết bị chưa thể kết nối AWS.
- `AWS_THING_NAME`, endpoint, certificate và private key trong
  `include/secrets.h` phải thuộc cùng một AWS account/region.
- Tăng `FW_VERSION` trong `src/main.cpp` trước mỗi bản phát hành.
- Build bằng environment `yolo_uno`. Board đang dùng partition table 8 MB có
  hai app slot, mỗi slot `0x330000` byte.

File để tải lên S3 là:

```text
.pio/build/yolo_uno/firmware.bin
```

Lấy SHA-256 và kích thước trên PowerShell:

```powershell
Get-FileHash .pio/build/yolo_uno/firmware.bin -Algorithm SHA256
(Get-Item .pio/build/yolo_uno/firmware.bin).Length
```

## 2. Quyền AWS IoT policy của certificate

Giữ các quyền telemetry/command hiện tại và thêm quyền Jobs sau, thay `REGION`,
`ACCOUNT_ID` và `THING_NAME` bằng giá trị thật:

```json
{
  "Effect": "Allow",
  "Action": "iot:Publish",
  "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/$aws/things/THING_NAME/jobs/*"
},
{
  "Effect": "Allow",
  "Action": "iot:Subscribe",
  "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topicfilter/$aws/things/THING_NAME/jobs/*"
},
{
  "Effect": "Allow",
  "Action": "iot:Receive",
  "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/$aws/things/THING_NAME/jobs/*"
},
{
  "Effect": "Allow",
  "Action": [
    "iotjobsdata:StartNextPendingJobExecution",
    "iotjobsdata:UpdateJobExecution"
  ],
  "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/$aws/things/THING_NAME"
}
```

Không thay toàn bộ policy bằng ba statement trên; hãy ghép chúng vào mảng
`Statement` đang có.

## 3. IAM role để AWS IoT Jobs tạo presigned URL

Tạo role mà service `iot.amazonaws.com` được phép assume. Role cần ít nhất
`s3:GetObject` cho prefix chứa firmware, ví dụ:

```json
{
  "Version": "2012-10-17",
  "Statement": [{
    "Effect": "Allow",
    "Action": "s3:GetObject",
    "Resource": "arn:aws:s3:::YOUR_BUCKET/firmware/*"
  }]
}
```

Trust policy tối thiểu:

```json
{
  "Version": "2012-10-17",
  "Statement": [{
    "Effect": "Allow",
    "Principal": { "Service": "iot.amazonaws.com" },
    "Action": "sts:AssumeRole"
  }]
}
```

Trong production nên thêm điều kiện `aws:SourceAccount` và `aws:SourceArn` để
tránh confused-deputy.

## 4. Job document

Tạo `job-document.json`. `size` phải là số byte chính xác; `sha256` là 64 ký tự
hex. AWS IoT Jobs sẽ thay trường `url` bằng presigned URL mỗi khi thiết bị gọi
`start-next`.

```json
{
  "operation": "ota",
  "board": "esp32s3-solgrid",
  "version": "v2.7.0",
  "url": "${aws:iot:s3-presigned-url-v2:https://s3.ap-southeast-1.amazonaws.com/YOUR_BUCKET/firmware/firmware-v2.7.0.bin}",
  "sha256": "PUT_64_HEX_SHA256_HERE",
  "size": 1234567
}
```

Tạo job (một dòng để chạy giống nhau trên PowerShell):

```powershell
aws iot create-job --region ap-southeast-1 --job-id solgrid-esp32-02-v2-7-0 --targets arn:aws:iot:ap-southeast-1:ACCOUNT_ID:thing/solgrid-esp32-02 --document file://job-document.json --presigned-url-config roleArn=arn:aws:iam::ACCOUNT_ID:role/IotJobsS3DownloadRole,expiresInSec=3600 --target-selection SNAPSHOT
```

## 5. Log mong đợi

Sau khi cấu hình Wi-Fi thành công:

```text
WiFi uplink connected: ...
AWS IoT connected
AWS Jobs: received solgrid-esp32-02-v2-7-0
fw_status downloading / v2.7.0
AWS Job solgrid-esp32-02-v2-7-0 -> IN_PROGRESS (downloading)
fw_status applying / v2.7.0
```

Thiết bị reboot, bản mới kết nối lại AWS, nhận lại job đang `IN_PROGRESS`, nhận
ra `FW_VERSION` đã trùng và báo `SUCCEEDED (already_running)`.

## 6. Lưu ý bảo mật

SHA-256 trong job document bảo vệ tính toàn vẹn vì document đi qua MQTT mutual
TLS. Với hệ thống production, nên bổ sung code-signing và xác minh chữ ký công
khai trên thiết bị để tách quyền phát hành firmware khỏi quyền vận hành AWS.
