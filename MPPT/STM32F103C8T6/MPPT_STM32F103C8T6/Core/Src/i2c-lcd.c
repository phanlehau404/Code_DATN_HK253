#include "i2c-lcd.h"
#include <stdio.h> // Cần thiết cho hàm sprintf

extern I2C_HandleTypeDef hi2c1;

/* Dịch trái 1 bit cho địa chỉ I2C theo chuẩn của STM32 HAL */
#define SLAVE_ADDRESS_LCD (0x27 << 1) // Hoặc đổi thành (0x3F << 1) nếu mạch của bạn dùng chip khác

void lcd_send_cmd (char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  // en=1, rs=0
    data_t[1] = data_u | 0x08;  // en=0, rs=0
    data_t[2] = data_l | 0x0C;  // en=1, rs=0
    data_t[3] = data_l | 0x08;  // en=0, rs=0
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  // en=1, rs=1
    data_t[1] = data_u | 0x09;  // en=0, rs=1
    data_t[2] = data_l | 0x0D;  // en=1, rs=1
    data_t[3] = data_l | 0x09;  // en=0, rs=1
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_init (void) {
    lcd_send_cmd(0x33); /* set 4-bits interface */
    HAL_Delay(5);
    lcd_send_cmd(0x32);
    HAL_Delay(5);
    lcd_send_cmd(0x28); /* start to set LCD function */
    HAL_Delay(5);
    lcd_send_cmd(0x01); /* clear display */
    HAL_Delay(5);
    lcd_send_cmd(0x06); /* set entry mode */
    HAL_Delay(5);
    lcd_send_cmd(0x0c); /* set display to on */
    HAL_Delay(5);
    lcd_send_cmd(0x02); /* move cursor to home and set data address to 0 */
    HAL_Delay(5);
    lcd_send_cmd(0x80);
}

void lcd_send_string (char *str)
{
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_clear_display (void)
{
    lcd_send_cmd(0x01); // clear display
    HAL_Delay(2);       // Lệnh clear tốn thời gian hơn bình thường
}

void lcd_goto_XY (int row, int col)
{
    // Mảng lưu địa chỉ offset của 4 dòng trên LCD2004
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

    // Giới hạn giá trị row từ 1 đến 4 để tránh lỗi truy cập mảng
    if (row >= 1 && row <= 4)
    {
        // 0x80 là mã lệnh set DDRAM Address, cộng thêm offset của dòng và cột
        lcd_send_cmd(0x80 | (col + row_offsets[row - 1]));
    }
}

void lcd_send_number (int number)
{
    char buffer[16]; // Tăng kích thước mảng để tránh tràn bộ nhớ khi in số lớn
    sprintf(buffer, "%d", number);
    lcd_send_string(buffer);
}
