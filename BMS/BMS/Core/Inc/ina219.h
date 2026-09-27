#ifndef INA219_H
#define INA219_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* INA219 register map. */
#define INA219_REG_CONFIG        0x00u
#define INA219_REG_SHUNTVOLTAGE  0x01u
#define INA219_REG_BUSVOLTAGE    0x02u
#define INA219_REG_POWER         0x03u
#define INA219_REG_CURRENT       0x04u
#define INA219_REG_CALIBRATION   0x05u

/* Calibration copied from the original project. */
#define INA219_DEFAULT_ADDR_7BIT       0x40u
#define INA219_CALIBRATION_VALUE       0x1000u
#define INA219_CONFIG_VALUE            0x399Fu
#define INA219_SHUNT_OHM               0.100f
#define INA219_CURRENT_LSB_MA           0.1f
#define INA219_POWER_LSB_MW             2.0f

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address;       /* HAL-format address: 7-bit address << 1. */
    float current_lsb;      /* mA / bit */
    float power_lsb;        /* mW / bit */
} INA219_HandleTypeDef;

HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *dev,
                              I2C_HandleTypeDef *hi2c,
                              uint8_t address_7bit);
HAL_StatusTypeDef INA219_Reset(INA219_HandleTypeDef *dev);
HAL_StatusTypeDef INA219_ReapplyCalibration(INA219_HandleTypeDef *dev);
HAL_StatusTypeDef INA219_ReadCurrent_mA(INA219_HandleTypeDef *dev, float *current_mA);
HAL_StatusTypeDef INA219_ReadBusVoltage_V(INA219_HandleTypeDef *dev, float *bus_V);
HAL_StatusTypeDef INA219_ReadShuntVoltage_mV(INA219_HandleTypeDef *dev, float *shunt_mV);
HAL_StatusTypeDef INA219_ReadPower_W(INA219_HandleTypeDef *dev, float *power_W);

/* Legacy-compatible convenience APIs. */
int16_t INA219_GetCurrentRaw(INA219_HandleTypeDef *dev);
int16_t INA219_GetShuntRaw(INA219_HandleTypeDef *dev);
float INA219_GetBusVoltage_V(INA219_HandleTypeDef *dev);
float INA219_GetShuntVoltage_mV(INA219_HandleTypeDef *dev);
float INA219_GetCurrent_mA(INA219_HandleTypeDef *dev);
float INA219_GetCurrent_A(INA219_HandleTypeDef *dev);
float INA219_GetPower_W(INA219_HandleTypeDef *dev);

#endif /* INA219_H */
