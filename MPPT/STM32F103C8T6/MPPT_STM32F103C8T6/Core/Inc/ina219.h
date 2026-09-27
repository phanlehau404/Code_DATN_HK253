#ifndef INA219_H_
#define INA219_H_

#include "main.h" /* Tự động nhận diện thư viện HAL của vi điều khiển bạn đang dùng */

/* --- Định nghĩa địa chỉ các thanh ghi của INA219 --- */
#define INA219_REG_CONFIG                      0x00
#define INA219_REG_SHUNTVOLTAGE                0x01
#define INA219_REG_BUSVOLTAGE                  0x02
#define INA219_REG_POWER                       0x03
#define INA219_REG_CURRENT                     0x04
#define INA219_REG_CALIBRATION                 0x05

/* --- Cấu trúc dữ liệu của INA219 --- */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    float current_lsb;  /* Giá trị LSB của dòng điện (mA/bit) */
    float power_lsb;    /* Giá trị LSB của công suất (mW/bit) */
} INA219_HandleTypeDef;

/* --- Khai báo các hàm giao tiếp --- */
HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
float INA219_GetBusVoltage_V(INA219_HandleTypeDef *dev);
float INA219_GetShuntVoltage_mV(INA219_HandleTypeDef *dev);
float INA219_GetCurrent_mA(INA219_HandleTypeDef *dev);
float INA219_GetCurrent_A(INA219_HandleTypeDef *dev);
float INA219_GetPower_W(INA219_HandleTypeDef *dev);
int16_t INA219_GetCurrentRaw(INA219_HandleTypeDef *dev);
int16_t INA219_GetShuntRaw(INA219_HandleTypeDef *dev);
HAL_StatusTypeDef INA219_Reset(INA219_HandleTypeDef *dev);

#endif /* INA219_H_ */
