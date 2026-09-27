#include "ina219.h"

/* --- Hàm hỗ trợ nội bộ: Đọc/Ghi 16-bit qua I2C (INA219 dùng Big-Endian) --- */
static HAL_StatusTypeDef INA219_WriteRegister(INA219_HandleTypeDef *dev, uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF; /* MSB */
    data[1] = value & 0xFF;        /* LSB */
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

static uint16_t INA219_ReadRegister(INA219_HandleTypeDef *dev, uint8_t reg)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    return ((uint16_t)data[0] << 8) | data[1];
}

int16_t INA219_GetCurrentRaw(INA219_HandleTypeDef *dev)
{
    INA219_WriteRegister(dev, INA219_REG_CALIBRATION, 0x1000);
    return (int16_t)INA219_ReadRegister(dev, INA219_REG_CURRENT);
}

int16_t INA219_GetShuntRaw(INA219_HandleTypeDef *dev)
{
    return (int16_t)INA219_ReadRegister(dev, INA219_REG_SHUNTVOLTAGE);
}

/* --- Khởi tạo cảm biến --- */
HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    dev->hi2c = hi2c;
    dev->address = address << 1; /* Dịch trái 1 bit cho hàm I2C của STM32 HAL (ví dụ: 0x40 thành 0x80) */

    /* Thiết lập thông số Calibration cho trở shunt 0.1 Ohm, Dòng Max = 3.2A */
    /* Current_LSB = 3.2A / 32768 = 0.1 mA/bit */
    /* Cal = trunc(0.04096 / (Current_LSB * Rshunt)) = 4096 = 0x1000 */
    dev->current_lsb = 0.1f;
    dev->power_lsb = 2.0f; /* Power LSB bằng 20 lần Current LSB (2 mW/bit) */

    /* Ghi giá trị Calibration */
    if (INA219_WriteRegister(dev, INA219_REG_CALIBRATION, 0x1000) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Cấu hình Config Register: Bus 32V, Gain /8, 12-bit ADC, Chế độ Continuous */
    return INA219_WriteRegister(dev, INA219_REG_CONFIG, 0x399F);
}

/* --- Reset cảm biến về mặc định nhà sản xuất --- */
HAL_StatusTypeDef INA219_Reset(INA219_HandleTypeDef *dev)
{
    return INA219_WriteRegister(dev, INA219_REG_CONFIG, 0x8000);
}

/* --- Đọc điện áp Bus (V) --- */
float INA219_GetBusVoltage_V(INA219_HandleTypeDef *dev)
{
    uint16_t raw_data = INA219_ReadRegister(dev, INA219_REG_BUSVOLTAGE);
    /* Dịch phải 3 bit (bỏ qua cờ OVF và CNVR), độ phân giải là 4mV */
    raw_data >>= 3;
    return (float)raw_data * 0.004f;
}

/* --- Đọc điện áp Shunt (mV) --- */
float INA219_GetShuntVoltage_mV(INA219_HandleTypeDef *dev)
{
    int16_t raw_data = (int16_t)INA219_ReadRegister(dev, INA219_REG_SHUNTVOLTAGE);
    /* Độ phân giải là 0.01mV */
    return (float)raw_data * 0.01f;
}

/* --- Đọc dòng điện (mA) --- */
float INA219_GetCurrent_mA(INA219_HandleTypeDef *dev)
{
    /* Đôi khi cần kích hoạt lại thanh ghi calibration trước khi đọc Current/Power
       để tránh lỗi do nhiễu làm mất calib */
    INA219_WriteRegister(dev, INA219_REG_CALIBRATION, 0x1000);

    int16_t raw_data = (int16_t)INA219_ReadRegister(dev, INA219_REG_CURRENT);
    return (float)raw_data * dev->current_lsb;
}

/* --- Đọc dòng điện (A) --- */
float INA219_GetCurrent_A(INA219_HandleTypeDef *dev)
{
    return INA219_GetCurrent_mA(dev) / 1000.0f;
}

/* --- Đọc công suất (W) --- */
float INA219_GetPower_W(INA219_HandleTypeDef *dev)
{
    INA219_WriteRegister(dev, INA219_REG_CALIBRATION, 0x1000);

    int16_t raw_data = (int16_t)INA219_ReadRegister(dev, INA219_REG_POWER);
    /* LSB của công suất tính bằng mW, nên cần nhân lsb và chia 1000 để ra W */
    return ((float)raw_data * dev->power_lsb) / 1000.0f;
}
