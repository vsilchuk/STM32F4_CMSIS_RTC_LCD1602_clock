#include <stdint.h>		
#include "stm32f4xx.h"
#include "lcd1602.h"

#define frequency	16000000UL	// 16 MHz high-speed internal (RC)

/*	GLOBAL VARIABLES START				*/

uint8_t LCD_show_ready = 0, debounce_ms_enable = 0, pressure_flag;
uint16_t pressure_10_ms_ticks = 0;
uint32_t field = 0;
uint32_t alarm_fields_num = 3, clock_fields_num = 7;
uint32_t alarm_cfg_mode = 0, clock_cfg_mode = 0, alarm_show_mode = 0, clock_show_mode = 1;
uint32_t btn_pressure_start = 0, btn_pressure_done = 0;
uint8_t time_get_done = 0, alarm_get_done = 0;
uint8_t clk_1hz = 0;
uint8_t alarm_enable = 0, is_alarm = 0;

typedef struct s_RTC_struct_full {
	uint8_t year_tens;		// 20[1]9
	uint8_t year_units;		// 201[9]
	uint8_t week_day;		// 001 for Monday, 111 for Sunday
	uint8_t month_tens;		// [1]2
	uint8_t month_units;		// 1[2]
	uint8_t date_tens;		// [2]5
	uint8_t date_units;		// 2[5]
	uint8_t hour_tens;		// [0]0
	uint8_t hour_units;		// 0[0]
	uint8_t minute_tens;		// [0]3
	uint8_t minute_units;		// 0[3]
	uint8_t second_tens;		// [1]5
	uint8_t second_units;		// 1[5]
} RTC_struct_full;

typedef struct s_RTC_struct_brief {
	uint8_t years;			// 2019
	uint8_t week_day;		// 001 for Monday, 111 for Sunday
	uint8_t months;			// 12
	uint8_t date;			// 25
	uint8_t hours;			// 0
	uint8_t minutes;		// 03
	uint8_t seconds;		// 15
} RTC_struct_brief;

volatile RTC_struct_full RTC_data_full_buff;	
volatile RTC_struct_brief RTC_data_brief_buff;

/*	GLOBAL VARIABLES END				*/

/************************************************** SYSTICK **************************************************/

void fill_struct_default(RTC_struct_brief volatile *br_data, RTC_struct_full volatile *f_data)
{	
	// default date: FRI 27.12.19 04:49:00
	
	f_data->year_tens = 0x1;
	f_data->year_units = 0x9;
	f_data->week_day = 0x5;
	f_data->month_tens = 0x1;
	f_data->month_units = 0x2;
	f_data->date_tens = 0x2;
	f_data->date_units = 0x7;	
	f_data->hour_tens = 0x0;
	f_data->hour_units = 0x4;
	f_data->minute_tens = 0x4;
	f_data->minute_units = 0x9;
	f_data->second_tens = 0x0;
	f_data->second_units = 0x0;
	
	br_data->years = 0x19;
	br_data->week_day = 0x5;	
	br_data->months = 0x12; 
	br_data->date = 0x27;
	br_data->hours = 0x04;
	br_data->minutes = 0x49;
	br_data->seconds = 0x00;
}

void init_systick(void)
{
	SysTick->CTRL = 0;						// disable SysTick
	SysTick->LOAD = frequency - 1;					// set the initial reload value
	SysTick->VAL = 0;						// reset the curent SysTick counter value
	
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	// switch off

	// use proc.clock (1 = processor clock, 0 = external clock (HCLK/8 = 2 MHz)), enable interrupts (1 = enable)
}

void SysTick_Handler(void)
{
	if(debounce_ms_enable)
	{
		debounce_ms_enable = 0;
		
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	// SysTick off
		
		EXTI->IMR |= (EXTI_IMR_IM13 | EXTI_IMR_IM14 | EXTI_IMR_IM15);	// enable disabled interrupts
	}
}

void systick_debounce_ms(uint32_t debounce_ms)
{
	// default value is 250 ms debounce
	uint32_t time_ms = (debounce_ms >= 1 && debounce_ms <= 1000) ? debounce_ms : 250;	
	
	SysTick->VAL = 0;					// clear current CNT value
	SysTick->LOAD = ((frequency / 1000) * time_ms) - 1;	// (time * 10^-3 * frequency) - 1 = (time * 10^-3 * 16 000 000) - 1 = (time * 16 000) - 1

	debounce_ms_enable = 1;
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	// enable SysTick
}

/**********************************************************************************************************/

/************************************************** TIM2 **************************************************/
void tim2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// TIM2 clock
	TIM2->PSC = 16000 - 1;						// 1000 ticks per second - 1ms - prescaler 
	TIM2->ARR = 10-1;						// T = 10 ms = 10 ticks -> {0, 1, 2, 3, 4, 5, 6, 7, 8, 9} - auto-reload-register
	TIM2->DIER |= TIM_DIER_UIE;					// TIM2 DMA/interrupt enable register - allow events by timer -  Update interrupt enable 		
	TIM2->CR1 &= ~TIM_CR1_DIR;					// upcounting, DIR = 0
		
	NVIC_EnableIRQ(TIM2_IRQn);	// enable interrupts by TIM2
	__enable_irq();	// global interrupts enable
}

// Every 10 milliseconds in the TIM2 interrupt handler we check the status of the button.

void btn_pressure_check(void)
{
	EXTI->IMR &= ~EXTI_IMR_IM13;	// disable EXTI13 interrupts to make safe and clear situation
	EXTI->IMR &= ~EXTI_IMR_IM14;	// disable EXTI14 interrupts
	EXTI->IMR &= ~EXTI_IMR_IM15;	// disable EXTI15 interrupts
	
	btn_pressure_start = 1;
	
	TIM2->CR1 |= TIM_CR1_CEN;		// switch on TIM2 - 10 miliseconds to the next TIM2 interrupt
}

/**********************************************************************************************************/

/************************************************** GPIO **************************************************/

void init_GPIO(void)
{
	// OSPEEDR - 2 MHz by default after reset

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// enable GPIOB
	
	GPIOB->MODER &= ~GPIO_MODER_MODE13;			// PB13 for input 
	GPIOB->MODER &= ~GPIO_MODER_MODE14;			// PB14 for input
	GPIOB->MODER &= ~GPIO_MODER_MODE15;			// PB15 for input
	
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD13_0;			// pull-up for PB13 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD13_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD14_0;			// pull-up for PB14 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD15_0;			// pull-up for PB15 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD15_1;
}

/**********************************************************************************************************/

/************************************************** EXTI **************************************************/

void Nucleo_LED_init(void)
{
	if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN))
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// enable GPIOA
	}
	
	GPIOA->MODER |= GPIO_MODER_MODE5_0;	// PA5 for output - Nucleo onboard LED
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;	// push-pull mode is by default
}

void btn_irq_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// SYSCFG (System configuration controller) clock through APB2 bus enable
	
	// interrupt source selection
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PB;	// PB13 -> EXTI13
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PB;	// PB14 -> EXTI14
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PB;	// PB15 -> EXTI15
	
	EXTI->IMR |= (EXTI_IMR_IM13 | EXTI_IMR_IM14 | EXTI_IMR_IM15);	// interrupt request mask - IM11 and IM12 are not masked now
	EXTI->FTSR |= (EXTI_FTSR_TR13 | EXTI_FTSR_TR14 | EXTI_FTSR_TR15);	// falling trigger
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0);	// highest priority
	
	__enable_irq();	// enable interrupts, PRIMASK reset
}

void btn_fill_date_fields(RTC_struct_brief volatile *br_data, uint32_t field_cnt, int32_t add_sub, uint32_t is_alarm_cfg, uint32_t is_clock_cfg)
{
	if (is_alarm_cfg) 
	{
		switch(field_cnt)
		{
			case 0:
				br_data->date = (br_data->date >= 1 && br_data->date <= 30) ? (br_data->date + add_sub) : 1;
				break;
			case 1: 
				br_data->hours = (br_data->hours <= 22) ? (br_data->hours + add_sub) : 0;
				break;
			case 2: 
				br_data->minutes = (br_data->minutes <= 58) ? (br_data->minutes + add_sub) : 0;
				break;
			default:
				br_data->date = br_data->date;
				br_data->hours = br_data->hours;
				br_data->minutes = br_data->minutes;
		}
	}
	else if (is_clock_cfg)
	{
		switch(field)
		{
			case 0:
				br_data->week_day = (br_data->week_day >= 1 && br_data->week_day <= 6) ? (br_data->week_day + add_sub) : 1;
				break;
			case 1: 
				br_data->hours = (br_data->hours <= 22) ? (br_data->hours + add_sub) : 0;
				break;
			case 2: 
				br_data->minutes = (br_data->minutes <= 58) ? (br_data->minutes + add_sub) : 0;
				break;
			case 3:
				br_data->seconds = (br_data->seconds <= 58) ? (br_data->seconds + add_sub) : 0;
				break;
			case 4:
				br_data->date = (br_data->date >= 1 && br_data->date <= 30) ? (br_data->date + add_sub) : 1;
				break;
			case 5: 
				br_data->months = (br_data->months >= 1 && br_data->months <= 11) ? (br_data->months + add_sub) : 1;
				break;
			case 6: 
				br_data->years = (br_data->years <= 98) ? (br_data->years + add_sub) : 0;
				break;

			default:
				br_data->week_day = br_data->week_day;
				br_data->hours = br_data->hours;
				br_data->minutes = br_data->minutes;
				br_data->seconds = br_data->seconds;
				br_data->date = br_data->date;
				br_data->months = br_data->months;
				br_data->years = br_data->years;
		}
	}
}

/****************************************************** RTC ****************************************************/

void RTC_auto_wakeup_enable(void) 
{
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// disable the wakeup timer
	RTC->CR &= ~RTC_CR_WUTE;
	
	// polling to make sure the access to wakeup auto-reload counter and to WUCKSEL[2:0] bits is allowed 	
	while (!(RTC->ISR & RTC_ISR_WUTWF));
	
	// program the wakeup auto-reload value and the wakeup clock selection
	RTC->WUTR &= ~RTC_WUTR_WUT;	// the WUTF flag is set every (WUT[15:0] + 1) = (0 + 1) = (1) ck_wut cycles
	
	RTC->CR &= ~RTC_CR_WUCKSEL_1;	// 10x: ck_spre (usually 1 Hz) clock is selected
	RTC->CR |= RTC_CR_WUCKSEL_2;
			
	// enable the RTC Wakeup interrupt - enable the EXTI Line 22
	if (!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN))
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	}
	
	EXTI->IMR |= EXTI_IMR_IM22;	// interrupt request mask - IM22 is not masked now
	EXTI->RTSR |= EXTI_RTSR_TR22;	// rising edge trigger enabled for EXTI line 17
	
	NVIC_EnableIRQ(RTC_WKUP_IRQn);	// enable the RTC_WKUP IRQ channel in the NVIC
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
	NVIC_SetPriority(RTC_WKUP_IRQn, 0);	// highest priority
	
	// 1: Wakeup timer interrupt enabled
	RTC->CR |= RTC_CR_WUTIE;	
	
	// enable the timer again
	RTC->CR |= RTC_CR_WUTE;	
	
	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
	
	__enable_irq();	// global interrupts enable
}

void RTC_WKUP_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PR22)	
	{
		if(RTC->ISR & RTC_ISR_WUTF)
		{
			RTC->ISR &= ~RTC_ISR_WUTF;	//  flag is cleared by software by writing 0
			
			// GPIOA->ODR ^= GPIO_ODR_OD5;
			
			if (clock_show_mode) 
			{
				clk_1hz = 1;
				LCD_show_ready = 1;
			}

			EXTI->PR |= EXTI_PR_PR22;	// clear pending flag 
		}
	}
}

void RTC_data_init(void)
{
	// uint32_t time_value, date_value;
	
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// initialization mode on (INITF == 1) - calendar counter is stopped, can update now
	RTC->ISR |= RTC_ISR_INIT;
	
	// INITF polling
	while (!(RTC->ISR & RTC_ISR_INITF));
	
	// prescalers - two separate write access - synch predivider 
	RTC->PRER |= 0xFF; // 255 
		
	// asynch predivider
	RTC->PRER |= (0x7F << 16);	// 127
	
	RTC->TR = 0x00000000;
	RTC->DR = 0x00000000;
	
	// 24h format == 0
	RTC->CR &= ~RTC_CR_FMT;
		
	// exit from the init mode
	RTC->ISR &= ~RTC_ISR_INIT;
	
	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
}

void RTC_data_update(RTC_struct_full volatile *f_data)
{
	uint32_t time_value, date_value;
	time_value = time_value ^ time_value;
	date_value = date_value ^ date_value;
	
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// initialization mode on (INITF == 1) - calendar counter is stopped, can update now
	RTC->ISR |= RTC_ISR_INIT;
	
	// INITF polling
	while (!(RTC->ISR & RTC_ISR_INITF));
	
	// prescalers - two separate write access - synch predivider 
	RTC->PRER |= 0xFF; // 255 
		
	// asynch predivider
	RTC->PRER |= (0x7F << 16);	// 127
	
	RTC->TR = 0x00000000;
	
	time_value |= ((f_data->hour_tens << RTC_TR_HT_Pos) | (f_data->hour_units << RTC_TR_HU_Pos) | (f_data->minute_tens << RTC_TR_MNT_Pos) | (f_data->minute_units << RTC_TR_MNU_Pos));
	time_value |= ((f_data->second_tens << RTC_TR_ST_Pos) | (f_data->second_units << RTC_TR_SU_Pos));
	
	RTC->TR = time_value;
	
	RTC->DR = 0x00000000;
	
	date_value |= ((f_data->year_tens << RTC_DR_YT_Pos) | (f_data->year_units << RTC_DR_YU_Pos) | (f_data->week_day << RTC_DR_WDU_Pos) | (f_data->month_tens << RTC_DR_MT_Pos) | (f_data->month_units << RTC_DR_MU_Pos) | (f_data->date_tens << RTC_DR_DT_Pos) | (f_data->date_units << RTC_DR_DU_Pos));
	
	RTC->DR = date_value;
	
	// 24h format == 0
	RTC->CR &= ~RTC_CR_FMT;
		
	// exit from the init mode
	RTC->ISR &= ~RTC_ISR_INIT;
	
	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
}

void RTC_alarm_init(void)
{
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;
	
	// wait for Alarm A write flag, to make sure the access to alarm reg is allowed
	while (!(RTC->ISR & RTC_ISR_ALRAWF));
	
	// date, hours, minutes, seconds mask - Alarm A set if they match
	RTC->ALRMAR &= ~RTC_ALRMAR_MSK4;	// 0: Alarm A set if the date/day match
	RTC->ALRMAR &= ~RTC_ALRMAR_MSK3;	// 0: Alarm A set if the hours match
	RTC->ALRMAR &= ~RTC_ALRMAR_MSK2;	// 0: Alarm A set if the minutes match
	RTC->ALRMAR |= RTC_ALRMAR_MSK1;		// 1: Seconds don’t care in Alarm A comparison
		
	RTC->ALRMAR &= ~RTC_ALRMAR_WDSEL;	// DU[3:0] field represents the date units
	RTC->ALRMAR &= ~RTC_ALRMAR_PM;		// 0: AM or 24-hour format
	
	// enable the RTC Alarm interrupt - enable the EXTI Line 17
	// SYSCFG (System configuration controller) clock through APB2 bus enable
	if (!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN))
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	}
	
	EXTI->IMR |= EXTI_IMR_IM17;	// interrupt request mask - IM17 is not masked now
	EXTI->RTSR |= EXTI_RTSR_TR17;	// rising edge trigger enabled for EXTI line 17
	
	NVIC_EnableIRQ(RTC_Alarm_IRQn);	// enable the RTC_Alarm IRQ channel in the NVIC
	NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);	// highest priority
	
	// 1: Alarm A interrupt enabled
	RTC->CR |= RTC_CR_ALRAIE;	
	
	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
	
	__enable_irq();	// global interrupts enable
}

void RTC_alarm_update(RTC_struct_full volatile *f_data) 
{
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;
	
	// wait for Alarm A write flag, to make sure the access to alarm reg is allowed
	while (!(RTC->ISR & RTC_ISR_ALRAWF));
	
	RTC->ALRMAR |= (f_data->date_tens << RTC_ALRMAR_DT_Pos);	// Bits 29:28 DT[1:0]: Date tens in BCD format
	RTC->ALRMAR |= (f_data->date_units << RTC_ALRMAR_DU_Pos);	// Bits 27:24 DU[3:0]: Date units or day in BCD format.
	RTC->ALRMAR |= (f_data->hour_tens << RTC_ALRMAR_HT_Pos);	// Bits 21:20 HT[1:0]: Hour tens in BCD forma
	RTC->ALRMAR |= (f_data->hour_units << RTC_ALRMAR_HU_Pos);	// Bits 19:16 HU[3:0]: Hour units in BCD format.
	RTC->ALRMAR |= (f_data->minute_tens << RTC_ALRMAR_MNT_Pos);	// Bits 14:12 MNT[2:0]: Minute tens in BCD format.
	RTC->ALRMAR |= (f_data->minute_units << RTC_ALRMAR_MNU_Pos);	// Bits 11:8 MNU[3:0]: Minute units in BCD format.
	
	// enable Alarm A
	RTC->CR |= RTC_CR_ALRAE;

	// enable the RTC Alarm interrupt - enable the EXTI Line 17
	// SYSCFG (System configuration controller) clock through APB2 bus enable
	if (!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN))
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	}
	
	EXTI->IMR |= EXTI_IMR_IM17;	// interrupt request mask - IM17 is not masked now
	EXTI->RTSR |= EXTI_RTSR_TR17;	// rising edge trigger enabled for EXTI line 17
	
	NVIC_EnableIRQ(RTC_Alarm_IRQn);	// enable the RTC_Alarm IRQ channel in the NVIC
	NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);	// highest priority
	
	// 1: Alarm A interrupt enabled
	RTC->CR |= RTC_CR_ALRAIE;	
	
	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
	
	__enable_irq();	// global interrupts enable
}

void RTC_alarm_disable(void)
{
	alarm_enable = 0;	// clear enable flag
	
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;
	
	// wait for Alarm A write flag, to make sure the access to alarm reg is allowed
	while (!(RTC->ISR & RTC_ISR_ALRAWF));
		
	RTC->ALRMAR &= (RTC_ALRMAR_DU ^ RTC_ALRMAR_DU);		// Bits 27:24 DU[3:0]: Date units or day in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_HT ^ RTC_ALRMAR_HT);		// Bits 21:20 HT[1:0]: Hour tens in BCD forma
	RTC->ALRMAR &= (RTC_ALRMAR_HU ^ RTC_ALRMAR_HU);		// Bits 19:16 HU[3:0]: Hour units in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_MNT ^ RTC_ALRMAR_MNT);	// Bits 14:12 MNT[2:0]: Minute tens in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_MNU ^ RTC_ALRMAR_MNU);	// Bits 11:8 MNU[3:0]: Minute units in BCD format.
	
	RTC->WPR = 0xFF;
}

void RTC_Alarm_IRQHandler(void)
{
	is_alarm = 1;	// display 
			
	// unlock write protection
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	// disable Alarm A
	RTC->CR &= ~RTC_CR_ALRAE;
	alarm_enable = 0;
			
	// wait for Alarm A write flag, to make sure the access to alarm reg is allowed
	while (!(RTC->ISR & RTC_ISR_ALRAWF));
		
	RTC->ALRMAR &= (RTC_ALRMAR_DU ^ RTC_ALRMAR_DU);		// Bits 27:24 DU[3:0]: Date units or day in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_HT ^ RTC_ALRMAR_HT);		// Bits 21:20 HT[1:0]: Hour tens in BCD forma
	RTC->ALRMAR &= (RTC_ALRMAR_HU ^ RTC_ALRMAR_HU);		// Bits 19:16 HU[3:0]: Hour units in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_MNT ^ RTC_ALRMAR_MNT);	// Bits 14:12 MNT[2:0]: Minute tens in BCD format.
	RTC->ALRMAR &= (RTC_ALRMAR_MNU ^ RTC_ALRMAR_MNU);	// Bits 11:8 MNU[3:0]: Minute units in BCD format.

	// lock write protection - writing a wrong key reactivates the write protection
	RTC->WPR = 0xFF;
			
	RTC->ISR &= ~RTC_ISR_ALRAF;	//  flag is cleared by software by writing 0
	EXTI->PR |= EXTI_PR_PR17;	// clear pending flag 
}

void fill_RTC_struct_full(RTC_struct_brief volatile *br_data, RTC_struct_full volatile *f_data, uint32_t is_clock_cfg_mode)
{
	// get brief data format --> fill full data format
	// is_clock_cfg_mode == 0 --> alarm_cfg mode --> need only date, hour and minute tens/units
	
	f_data->year_tens = (is_clock_cfg_mode) ? (br_data->years / 10) : (f_data->year_tens);
	f_data->year_units = (is_clock_cfg_mode) ? (br_data->years - (f_data->year_tens * 10)) : (f_data->year_units);
	
	f_data->week_day = (is_clock_cfg_mode) ? br_data->week_day : (f_data->week_day);
		
	f_data->month_tens = (is_clock_cfg_mode) ? (br_data->months / 10) : (f_data->month_tens);
	f_data->month_units = (is_clock_cfg_mode) ? (br_data->months - (f_data->month_tens * 10)) : (f_data->month_units);
		
	f_data->date_tens = br_data->date / 10;
	f_data->date_units = (br_data->date - (f_data->date_tens * 10));
	
	f_data->hour_tens = br_data->hours / 10;
	f_data->hour_units = (br_data->hours - (f_data->hour_tens * 10));
	
	f_data->minute_tens = br_data->minutes / 10;
	f_data->minute_units = (br_data->minutes - (f_data->minute_tens * 10));
	
	f_data->second_tens = (is_clock_cfg_mode) ? (br_data->seconds / 10) : (f_data->second_tens);
	f_data->second_units = (is_clock_cfg_mode) ? (br_data->seconds - (f_data->second_tens * 10)) : (f_data->second_units);
}

/***************************************************************************************************************/

void EXTI15_10_IRQHandler(void)
{
	EXTI->IMR &= ~EXTI_IMR_IM13;	// disable EXTI13 interrupts
	EXTI->IMR &= ~EXTI_IMR_IM14;	// disable EXTI14 interrupts
	EXTI->IMR &= ~EXTI_IMR_IM15;	// disable EXTI15 interrupts
	
	systick_debounce_ms(500);	// start debouncing - you can't input some button values more often, than 1 time in 500 ms
	
	if(EXTI->PR & EXTI_PR_PR13)	// PB13 - VALUE++		
	{
		btn_fill_date_fields(&RTC_data_brief_buff, field, 1, alarm_cfg_mode, clock_cfg_mode);
		
		EXTI->PR |= EXTI_PR_PR13;	// clear pending flag by writing 1 (clear event flag after work)
	}
	else if(EXTI->PR & EXTI_PR_PR14)	// PB14 - VALUE--
	{
		btn_fill_date_fields(&RTC_data_brief_buff, field, (-1), alarm_cfg_mode, clock_cfg_mode);
		
		EXTI->PR |= EXTI_PR_PR14;	// clear pending flag by writing 1 
	}
	else if(EXTI->PR & EXTI_PR_PR15)	// PB15 - ALARM CONFIGURATON
	{
		// set pressure flag, buffer:
		pressure_flag = (!(GPIOB->IDR & GPIO_IDR_ID15));	// PB15 == 0 --> pressure_flag = 1
		
		btn_pressure_check();		// save PB15 IDR value --> start TIM2 10ms counting
	
		EXTI->PR |= EXTI_PR_PR15;	// clear pending flag by writing 1
	}
	
	LCD_show_ready = 1;
}

void TIM2_IRQHandler(void)
{	
	TIM2->SR &= ~TIM_SR_UIF;	// clear pending bit in the interrupt handler
	
	// periodic checking 
	if (btn_pressure_start)
	{
		if ((!(GPIOB->IDR & GPIO_IDR_ID15)) == pressure_flag)
		{
			pressure_10_ms_ticks = pressure_10_ms_ticks + 1;		
		}
		else 
		{
			btn_pressure_start = 0;
			
			TIM2->CR1 &= ~TIM_CR1_CEN;	// switch off TIM2
				
			if (pressure_10_ms_ticks > 700)
			{
				RTC_alarm_disable();
			}
			else if (pressure_10_ms_ticks > 500 && pressure_10_ms_ticks <= 700)	
			{
				// 700 x 10ms = 7000ms = 7s --> calendar-clock configuration mode ON
						
				clock_cfg_mode = 1;
				alarm_cfg_mode = 0;
					
				field = 0;
			} 
			else if (pressure_10_ms_ticks >= 200 && pressure_10_ms_ticks <= 500)	
			{
				// 200 x 10ms = 2000ms = 2s --> alarm configuration mode ON (2s <= time <= 7s)
					
				alarm_cfg_mode = 1;
				clock_cfg_mode = 0;
					
				field = 0;
			} 
			else 
			{
				// <2s press
				if (clock_cfg_mode || alarm_cfg_mode)
				{
					field = field + 1;
				}
				else if (is_alarm)
				{
					is_alarm = 0;	// short buttons pressing 
				}
				else
				{
					clock_show_mode = (clock_show_mode) ? 0 : 1;
					alarm_show_mode = (!(clock_show_mode));
				}
			}
				
			pressure_10_ms_ticks = 0;	// reset cnt

			if (alarm_cfg_mode)
			{
				if (field >= alarm_fields_num) 
				{
					fill_RTC_struct_full(&RTC_data_brief_buff, &RTC_data_full_buff, 0);	// save final values
									
					RTC_alarm_update(&RTC_data_full_buff);	// update
						
					// configuration done
					alarm_cfg_mode = 0;
					field = 0;
							
					alarm_enable = 1;
					alarm_show_mode = 1;	// ready to display final alarm config data
					clock_show_mode = 0;
				}
			}
			else if (clock_cfg_mode)
			{
				if (field >= clock_fields_num)
				{
					// save final values
					fill_RTC_struct_full(&RTC_data_brief_buff, &RTC_data_full_buff, 1);	// save final values

					RTC_data_update(&RTC_data_full_buff);	// update
						
					// configuration done
					clock_cfg_mode = 0;
					field = 0;
									
					clock_show_mode = 1;	// ready to show final clock data
					alarm_show_mode = 0;
				}
			}
				
			EXTI->IMR |= (EXTI_IMR_IM13 | EXTI_IMR_IM14 | EXTI_IMR_IM15);	// enable disabled interrupts	
		}
	}
	
	LCD_show_ready = 1;
}

void RTC_get_time(RTC_struct_brief volatile *br_data) 
{
	// we need to clear less bits: (RTC->DR & RTC_DR_DT)
	// and to shift right the part, which we want to --> to normal decimal
	
	while (!(RTC->ISR & RTC_ISR_RSF));	//  Calendar shadow registers synchronized
	
	uint32_t TR_buf = 0, DR_buf = 0;
	
	TR_buf = (RTC->TR);
	
	br_data->hours = ((((TR_buf & RTC_TR_HT) >> RTC_TR_HT_Pos) * 10) + ((TR_buf & RTC_TR_HU) >> RTC_TR_HU_Pos));
	br_data->minutes = ((((TR_buf & RTC_TR_MNT) >> RTC_TR_MNT_Pos) * 10) + ((TR_buf & RTC_TR_MNU) >> RTC_TR_MNU_Pos));
	br_data->seconds = ((((TR_buf & RTC_TR_ST) >> RTC_TR_ST_Pos) * 10) + ((TR_buf & RTC_TR_SU) >> RTC_TR_SU_Pos));

	DR_buf = (RTC->DR);
	
	br_data->years = ((((DR_buf & RTC_DR_YT) >> RTC_DR_YT_Pos) * 10) + ((DR_buf & RTC_DR_YU) >> RTC_DR_YU_Pos));
	br_data->months = ((((DR_buf & RTC_DR_MT) >> RTC_DR_MT_Pos) * 10) + ((DR_buf & RTC_DR_MU) >> RTC_DR_MU_Pos));
	br_data->date = ((((DR_buf & RTC_DR_DT) >> RTC_DR_DT_Pos) * 10) + ((DR_buf & RTC_DR_DU) >> RTC_DR_DU_Pos));
	br_data->week_day = ((DR_buf & RTC_DR_WDU) >> RTC_DR_WDU_Pos);
	
	time_get_done = 1;
}

void RTC_get_alarm(RTC_struct_brief volatile *br_data) 
{	
	br_data->date = ((((RTC->ALRMAR & RTC_ALRMAR_DT) >> RTC_ALRMAR_DT_Pos) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_DU) >> RTC_ALRMAR_DU_Pos));
	br_data->hours = ((((RTC->ALRMAR & RTC_ALRMAR_HT) >> RTC_ALRMAR_HT_Pos) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_HU) >> RTC_ALRMAR_HU_Pos));
	br_data->minutes = ((((RTC->ALRMAR & RTC_ALRMAR_MNT) >> RTC_ALRMAR_MNT_Pos) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_MNU) >> RTC_ALRMAR_MNU_Pos));
	
	alarm_get_done = 1;
}

void LCD_clock_display(RTC_struct_brief volatile *br_data, uint32_t alarm_show, uint32_t alarm_cfg, uint32_t clk_show, uint32_t clk_cfg)
{
	// 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 0A | 0B | 0C | 0D | 0E | 0F
	// 40 | 41 | 42 | 43 | 44 | 45 | 46 | 47 | 48 | 49 | 4A | 4B | 4C | 4D | 4E | 4F
	
	uint8_t *week_days[] = {"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
	
	LCD1602_display_clear();
	
	if (alarm_show || alarm_cfg)
	{
		LCD1602_send_string("Alarm|Date: ", 0x00);
		LCD1602_send_string("|Time: ", 0x45);
		LCD1602_send_string(":", 0x4D);
	
		LCD1602_send_integer(br_data->date, 0x0C, 2);
		LCD1602_send_integer(br_data->hours, 0x4B, 2);
		LCD1602_send_integer(br_data->minutes, 0x4E, 2);
		
		if (alarm_cfg)
		{
			LCD1602_send_string("=CFG", 0x40);
		}
	}
	else if (clk_show || clk_cfg)
	{	
		LCD1602_send_string(week_days[br_data->week_day-1], 0x02);
		LCD1602_send_integer(br_data->hours, 0x06, 2);
		LCD1602_send_string(":", 0x08);
		LCD1602_send_integer(br_data->minutes, 0x09, 2);
		LCD1602_send_string(":", 0x0B);
		LCD1602_send_integer(br_data->seconds, 0x0C, 2);
		
		LCD1602_send_integer(br_data->date, 0x46, 2);
		LCD1602_send_string(".", 0x48);
		LCD1602_send_integer(br_data->months, 0x49, 2);
		LCD1602_send_string(".", 0x4B);
		LCD1602_send_integer(br_data->years, 0x4C, 2);		
		
		if (clk_cfg)
		{
			LCD1602_send_string("=CFG", 0x41);
		}
	}
}

void RTC_init(void) 
{
	// check the INITS status flag in RTC_ISR register to verify if the calendar is already initialized
	if (RTC->ISR & RTC_ISR_INITS)
	{
		return;
	}
	
	// PWR clock on
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	// enable WRITE - allow access to backup registers (BDCR)
	PWR->CR |= PWR_CR_DBP;	
	
	// pass only at the first time
	if (!(RCC->BDCR & RCC_BDCR_RTCEN))
	{
		// software reset - 1: Resets the entire Backup domain
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;
	}
	
	// enable LSE - Low-speed external oscillator 
	RCC->BDCR |= RCC_BDCR_LSEON;	
	
	// wait for being ready by polling
	while (!(RCC->BDCR & RCC_BDCR_LSERDY));
	
	// LSE as clk source - [01] - LSE oscillator clock used as the RTC clock
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;	
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_1;
	
	// RTC clock on
	RCC->BDCR |= RCC_BDCR_RTCEN;
	
	RTC_auto_wakeup_enable();
	RTC_alarm_init();
	
	// RTC_data_update(f_data);
	RTC_data_init();
}

void LED_indication(uint8_t enable)
{
	if (enable)
	{
		GPIOA->ODR |= GPIO_ODR_OD5;
	}
	else 
	{
		GPIOA->ODR &= ~GPIO_ODR_OD5;
	}
}

/*********************************************************************************************************/

/************************************************** main loop **************************************************/

int main(void)
{	
	init_GPIO();
	btn_irq_init();
	init_systick();
	tim2_init();
	
	Nucleo_LED_init();
	
	I2C1_init();
	LCD1602_init();	
	
	fill_struct_default(&RTC_data_brief_buff, &RTC_data_full_buff);	// initial structs filling 
	
	RTC_init();
	RTC_data_update(&RTC_data_full_buff);
	
	while(1)
	{		
		if(LCD_show_ready)
		{
			if (is_alarm)
			{
				// alarm situation handling 
				// ends by any button short pressing 
	
				LCD1602_display_clear();
				LCD1602_send_string("Wake up, Neo...", 0x00);
				LED_indication(is_alarm);
			}
			else if (clock_cfg_mode)
			{
				LCD_clock_display(&RTC_data_brief_buff, alarm_show_mode, alarm_cfg_mode, clock_show_mode, clock_cfg_mode);
			}
			else if (alarm_cfg_mode)
			{
				LCD_clock_display(&RTC_data_brief_buff, alarm_show_mode, alarm_cfg_mode, clock_show_mode, clock_cfg_mode);
			}
			else if (clock_show_mode)
			{
				if (clk_1hz)
				{
					// refresh every 1 second 
					RTC_get_time(&RTC_data_brief_buff);
			
					if (time_get_done)
					{
						LCD_clock_display(&RTC_data_brief_buff, alarm_show_mode, alarm_cfg_mode, clock_show_mode, clock_cfg_mode);
						
						time_get_done = 0; 
					}
					else 
					{
						LCD1602_display_clear();
						LCD1602_send_string("Time getting err", 0x00);
					}
					
					clk_1hz = 0;
				}
			}
			else if (alarm_show_mode)
			{
				RTC_get_alarm(&RTC_data_brief_buff);	// alarm time configuration-initiaization
				
				if (alarm_get_done)
				{
					LCD_clock_display(&RTC_data_brief_buff, alarm_show_mode, alarm_cfg_mode, clock_show_mode, clock_cfg_mode);
					
					if (alarm_enable)
					{
						LCD1602_send_string("=ON", 0x41);
					}
					else 
					{
						LCD1602_send_string("=OFF", 0x41);
					}
						
					alarm_get_done = 0; 
				}
				else 
				{
					LCD1602_display_clear();
					LCD1602_send_string("Alrm getting err", 0x00);
				}	
			}
			
			LCD_show_ready = 0;
		}
		else
		{
			__NOP;
		}
	}
}
