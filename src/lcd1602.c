#include "lcd1602.h"
#include <stdint.h>	
#include "stm32f4xx.h"

#define frequency	16000000UL		// 16 MHz high-speed internal (RC)

/*	I2C CODE DEFINES && VARIABLES && MACRO START	*/

#define I2C_MODE_TRANSMIT 0
#define I2C_ADDR(address, mode)	(((address) << 1) | (mode))	// [[addr[7:0]],[R/!W bit]]

#define PCF8574AT_ADDRESS 0x3F	// tested with Arduino 

#define LCD_READ 1	// R / !W
#define LCD_WRITE 0	// R / !W
#define LCD_INSTR 0	// RS
#define LCD_DATA 1	// RS
#define ENA 1	// latching
#define N_ENA 0	// latching	

/*	DISPLAY INSTRUCTIONS				*/

#define INTERFACE_8_BIT 0x30			// 0011 XXXX -> 001 [DATA 8/4] XXXX	
#define INTERFACE_4_BIT_2_ROWS_FONT_5X8 0x28	// 0010 10XX -> 001 [DATA 8/4] [Number of lines 1/2] [Font size] XX
#define INTERFACE_4_BIT	0x20
#define DISPLAY_ON_CURSOR_ON_BLINK_DISABLE 0x0E	// 0000 1110 -> 0000 1 [Display on/off] [Cursor] [Blink]	
#define DISPLAY_ON_CURSOR_ON_BLINK_ENABLE 0x0F	// 0000 1111 -> 0000 1 [Display on/off] [Cursor] [Blink]	
#define DISPLAY_CLEAR 0x01			// 0000 0001
#define DIRECTION_L_TO_R_SHIFT_DISABLE 0x06	// 0000 0110 -> 0000 01 [DDRAM address Increment/Decrement] [Shift]
#define DDRAM_ADDRESS_0X0 0x80
#define DISPLAY_ON_CURSOR_OFF_BLINK_DISABLE 0x0C
#define DISPLAY_OFF_CURSOR_OFF_BLINK_DISABLE 0x8

/*	I2C CODE DEFINES && VARIABLES && MACRO END	*/

void softdelay(volatile unsigned long N)
{
	volatile unsigned long inner;
	
	while (N--) 
	{
		for (inner = 100; inner > 0; inner--);
	}
}

// I2C initialization

void I2C1_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;    
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// enable GPIOB
	
	GPIOB->AFR[1] |= (4 << (4 * 0));	// PB8 - I2C1_SCL - for AF4
	GPIOB->AFR[1] |= (4 << (4 * 1));	// PB9 - I2C1_SDA - for AF4
	
	GPIOB->MODER &= ~(GPIO_MODER_MODER9 |GPIO_MODER_MODER8);	// AF open drain mode, external pull-ups
	GPIOB->MODER |= (GPIO_MODER_MODER9_1 |  GPIO_MODER_MODER8_1);
	GPIOB->OTYPER |=(GPIO_OTYPER_OT_9 |GPIO_OTYPER_OT_8);

	I2C1->CR2 &= ~I2C_CR2_FREQ;	
	I2C1->CR2 |= 16;		// 16 MHz - PB1
	
	I2C1->CCR &= ~I2C_CCR_CCR;
	I2C1->CCR |= 80;		// 100 kHz SCL frequency
	
	I2C1->CCR &= ~I2C_CCR_FS;
	
	I2C1->TRISE = 17;		// (1000ns / Tpclk1) + 1
	
	I2C1->CR1 |= I2C_CR1_PE;	// peripheral enable
}

void I2C_send(uint8_t *pack, uint8_t pack_num)
{
	// Sends N = pack_num bytes of the data to the Slave.
	// Check "Figure 164. Transfer sequence diagram for master transmitter" to see the events flow.
	
	I2C1->CR1 |= I2C_CR1_START;			// start bit 
	
	while (!(I2C1->SR1 & I2C_SR1_SB));		// wait for SB (event 5)
	(void) I2C1->SR1;				// clear SB flag - EV5: SB = 1, cleared by reading SR1 register followed by writing DR 
							// register with Address
							
	I2C1->DR = 0x7E;
	
	while (!(I2C1->SR1 & I2C_SR1_ADDR));		// wait for ADDR
	(void) I2C1->SR1;				// clear ADDR flag - EV6: ADDR = 1, cleared by reading SR1 register followed by 
	(void) I2C1->SR2;				// reading SR2
	
	I2C1->DR = *pack;
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// wait for data register empty
	
	pack++;
	
	for(uint8_t num = 0; num < pack_num-1; num++)
	{
		  I2C1->DR = *pack++;			// send and go to the next cell of the input array
		  while (!(I2C1->SR1 & I2C_SR1_TXE));
	}
	
	while (!(I2C1->SR1 & I2C_SR1_BTF));		// byte transfer finished
		
	I2C1->CR1 |= I2C_CR1_STOP;			// stop bit
}

void I2C_send_byte(uint8_t byte)
{
	I2C1->CR1 |= I2C_CR1_START;
    	
	while (!(I2C1->SR1 & I2C_SR1_SB));
	(void) I2C1->SR1;

	I2C1->DR = 0x7E;
           	
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void) I2C1->SR1;
	(void) I2C1->SR2;
    	
	I2C1->DR = byte;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	while (!(I2C1->SR1 & I2C_SR1_BTF));
	
	I2C1->CR1 |= I2C_CR1_STOP;
}

// implementation of latching inputting way
void PCF8574AT_send(uint8_t data, uint8_t led_flag, uint8_t rw_flag, uint8_t rs_flag, uint8_t cycles, uint32_t delay_ms)
{
	// latching timings (delays) were not taken in account
	
	// RS | R/!W | DB7 | DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0	- instruction format from datasheet
	// DB7 | DB6 | DB5 | DB4 | LED | E | R/!W | RS			- I2C (PCF8547AT instruction format)
	
	// cycles - 2 (instr -> latched_instr) or 4 (MSB_instr -> latched_MSB_instr -> LSB_instr -> latched_LSB_instr)
	// RS: 0 = instr, 1 = data
	// rw - R / !W
	
	uint8_t led, ena, rw, rs, data_msbits, data_lsbits;	// data's 4 most significant bits, and 4 least significant bits
	
	led = led_flag << 3;					// LED: 0 - backlight OFF, 1 - backlight ON
	ena = ENA << 2;
	rw = rw_flag << 1;					// R/!W: 0 - write, 1 - read
	rs = rs_flag;						// RS: 0 - instruction, 1 - data
	
	data_msbits = data & 0xF0;				// 1111 1111 & F0 = 1111 0000
	data_lsbits = (data << 4) & 0xF0;			// 1111 0001 << 4 = 0001 0000
	
	uint8_t package[4];
	
	for(uint8_t cells = 0; cells < 4; cells++)
	{
		package[cells] = 0;
	}
	
	package[0] = data_msbits | led | ena | rw | rs;		// ENA = 1
	package[1] = package[0] ^ 4; 				// ENA = 0 --> LATCHED!
	package[2] = data_lsbits | led | ena | rw | rs;		// ENA = 1
	package[3] = package[2] ^ 4;				// ENA = 0 --> LATCHED!
		
	I2C_send(package, cycles);				// send ready instruction
	
	softdelay(delay_ms);					// wait some time, required delay after instruction was sent
}

void LCD1602_backlight_on(void)
{
	PCF8574AT_send(0x0, 1, LCD_WRITE, LCD_INSTR, 2, 1);	// 1 ms delay, 2 cycles
}

void LCD1602_backlight_off(void)
{
	PCF8574AT_send(0x0, 0, LCD_WRITE, LCD_INSTR, 2, 1);	// 1 ms delay, 2 cycles
}

void LCD1602_send_char(uint8_t character)
{
	PCF8574AT_send(character, 1, LCD_WRITE, LCD_DATA, 4, 3);	// 4 cycles, 3 ms delay
}

void LCD1602_init()
{
	// RS | R/!W | DB7 | DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0 - instruction format from datasheet
	// DB7 | DB6 | DB5 | DB4 | LED | E | R/!W | RS	- I2C (PCF8547AT instruction format)
	
	// softdelay(5);	// some (40 ms, for the best way) delay before initialization start
	
	/*
	uint8_t instruction[] = 
	{
		INTERFACE_8_BIT, 
		INTERFACE_8_BIT, 
		INTERFACE_4_BIT, 
		INTERFACE_4_BIT_2_ROWS_FONT_5X8, 
		DISPLAY_OFF_CURSOR_OFF_BLINK_DISABLE,
		DISPLAY_CLEAR, 
		DIRECTION_L_TO_R_SHIFT_DISABLE, 
		DISPLAY_ON_CURSOR_OFF_BLINK_DISABLE
	};
	*/
	
	uint8_t instruction[] = {0x33, 0x32, 0x28, 0x0c, 0x01, 0x06, 0x80};	// working version from the forum 
	
	// 0x33,0x32,INTERFACE_4_BIT_2_ROWS_FONT_5X8,DISPLAY_ON_CURSOR_OFF_BLINK_DISABLE, DISPLAY_CLEAR, DIRECTION_L_TO_R_SHIFT_DISABLE, DDRAM_ADDRESS_0X0
	
	// uint8_t instr_count = sizeof(instruction) / sizeof(instruction[0]);		// number of initialization instructions
	uint8_t instr_count = 7;	// piece of hardcode
	
	uint8_t instr;
	
	for(instr = 0; instr < 3; instr++)
	{
		PCF8574AT_send(instruction[instr], 1, LCD_WRITE, LCD_INSTR, 2, 3);	// 3 ms delay, 2 cycles
	}
	for(instr = 3; instr < instr_count; instr++)
	{
		PCF8574AT_send(instruction[instr], 1, LCD_WRITE, LCD_INSTR, 4, 3);	// 3 ms delay, 4 cycles
	}
}

void LCD1602_set_cursor(uint8_t position)
{
	/*
		00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 0A | 0B | 0C | 0D | 0E | 0F
		40 | 41 | 42 | 43 | 44 | 45 | 46 | 47 | 48 | 49 | 4A | 4B | 4C | 4D | 4E | 4F
	*/
	
	uint8_t data;
	
	data = (1 << 7) | position;
	
	PCF8574AT_send(data, 1, LCD_WRITE, LCD_INSTR, 4, 3);	// 4 cycles, 3 ms delay
}

void LCD1602_send_char_position(uint8_t character, uint8_t position)
{
	LCD1602_set_cursor(position);
	LCD1602_send_char(character);
}

void LCD1602_position_rst(void)
{
	PCF8574AT_send(0x2, 1, LCD_WRITE, LCD_INSTR, 4, 1);	// 1 ms delay, 4 cycles
}

void LCD1602_display_clear(void)
{
	LCD1602_set_cursor(0x00);
	
	for(uint8_t position = 0x0; position <= 0x67; position++)
	{
		LCD1602_send_char(' ');
	}
	
	LCD1602_set_cursor(0x00);			// go to the start
}

void LCD1602_send_string(uint8_t *string, uint8_t position)
{
	LCD1602_set_cursor(position);
	
	while(*string)
	{
		LCD1602_send_char((uint8_t) *string);
		
		string++;
	}
}

void LCD1602_send_integer(uint16_t number, uint8_t position, uint8_t sign_num)
{
	if (sign_num > 0 && sign_num <= 5)
	{
		// uint16_t tens_of_thousands, thousands, hundreds, tens, units;
		uint8_t output[5];
		
		output[0] = number / 10000;
		output[1] = (number - (output[0] * 10000)) / 1000;
		output[2] = (number - (output[0] * 10000) - (output[1] * 1000)) / 100;	
		output[3] = (number - (output[0] * 10000) - (output[1] * 1000) - (output[2] * 100)) / 10;
		output[4] = (number - (output[0] * 10000) - (output[1] * 1000) - (output[2] * 100) - (output[3] * 10));
		
		LCD1602_set_cursor(position);
		
		for(uint8_t i = (5 - sign_num); i < 5; i++)
		{
			LCD1602_send_char(output[i] + 48);
		}
	}
	else
	{
		LCD1602_send_string("?????", position);
	}
}
