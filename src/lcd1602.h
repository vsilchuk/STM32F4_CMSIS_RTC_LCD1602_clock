#ifndef lcd1602_h
#define lcd1602_h

#include <stdint.h>

void softdelay(volatile unsigned long N);

void I2C1_init(void);
void I2C_send(uint8_t *pack, uint8_t pack_num);
void I2C_send_byte(uint8_t byte);

void PCF8574AT_send(uint8_t data, uint8_t led_flag, uint8_t rw_flag, uint8_t rs_flag, uint8_t cycles, uint32_t delay_ms);

void LCD1602_backlight_on(void);
void LCD1602_backlight_off(void);
void LCD1602_send_char(uint8_t character);
void LCD1602_init(void);
void LCD1602_set_cursor(uint8_t position);
void LCD1602_send_char_position(uint8_t character, uint8_t position);
void LCD1602_position_rst(void);
void LCD1602_display_clear(void);
void LCD1602_send_string(uint8_t *string, uint8_t position);
void LCD1602_send_integer(uint16_t number, uint8_t position, uint8_t sign_num);

#endif
