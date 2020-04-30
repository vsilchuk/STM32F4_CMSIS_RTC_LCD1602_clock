## Current time displaying device with additional features based on the STM32F401RE MCU and LCD1602 display with the PCF8574 I/O expander [CMSIS, NUCLEO-F401RE].

###### NTUU KPI, The Faculty of Electronics, The Department of Design of Electronic Digital Equipment (DEDEC/KEOA).

A current time displaying device with calendar and alarm functions based on the STM32F401RE microcontroller and LCD1602 display with the PCF8574 I/O expander.

See the [project report][1] (in ukrainian) for the details of this project and its detailed description. See also the [circuit diagram][2] and the [bill of materials][3] (also in ukrainian) to understand how to assemble this circuit and what details to use.

This project also uses a `lcd1602` library, written by me to be used for the work with the LCD1602 display and the PCF8574 I/O expander.

## Device structure:

+ The main component of this device is the STM32F401RE MCU, which is the main part of the NUCLEO-F401RE — STM32 Nucleo-64 development board.
+ The control and data input unit is represented by three tactile switches, which can be used for the device operation configuration — to configure the current time value, to activate the alarm and to configure it and to switch the device work modes.
+ The display unit implements a visual indication of the operation of the device, and is represented by one LED and by LCD1602 display (based on the HD44780 controller) and the PCF8574 I/O extender.
	
## Buttons and device configuration:

This device consists of three tactile switches for the device configuration: [SB2, SB3, SB4][2]. These buttons are connected to the PB13, PB14 and PB15 MCU pins and internally connected to the MCU VCC (+3V3) by the internal R11, R13 and R14 resistors.

There is also software buttons debounce implemented here.

+ **SB2** tactile switch increments the current date/time value in the date/time configuration mode.
+ **SB3** tactile switch increments the current date/time value in the date/time configuration mode.
+ **SB4** tactile switch can be used for the device work mode selection. It was implemented also an important possibility of determining the duration of its pressing in the way of using the TIM2 timer and interrupts. This duration determines the choice of the current device operation mode:
  + SB4 pressing duration time > 7s — alarm off; 
  + 7s > SB4 pressing duration time > 5s — current date/time values configuration mode activation; 
  + 5s > SB4 pressing duration time > 2s — alarm date/time values configuration mode activation; 
  + SB4 pressing duration time < 2s — switch to the next date/time field in the date/time values configuration mode / disable the triggered alarm / switch between displaying the current date/time values and displaying the set alarm date/time;

## Clock configuration:

The startup configuration file for the microcontroller according to the required microcontroller clock timing settings was generated using the [STM32F4xx_Clock_Configuration_V1.1.0][4] tool provided on the STMicroelectronics website:

![Clock configuration](https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/img/clock_configuration.png "Clock configuration")

Therefore, for the correct operation of the given code, make sure that:

+ the system frequency (**SYSCLK**) is **16MHz**;
+ the **APB** and **AHB** buses are clocked at **16MHz**;
+ the **SysTick** system timer is clocked at **16MHz**;

## Using the RTC:

You can read a document ["Using the STM32F4 hardware real-time clock (RTC)"][5] (written by me, in ukrainian), which describes the configuration steps for the STM32F401 hardware RTC module with the appropriate code examples in CMSIS.

[This document][6] will also be very useful for reading.

## A few photos of the operating modes of the device:

+ Alarm setting:

	![Alarm configuration](https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/img/alarm_cfg.png "Alarm configuration")
	
+ Alarm off:

	![Alarm off](https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/img/alarm_off.png "Alarm off")

+ Current date/time:

	![Current date/time](https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/img/time_date.png "Current date/time")

+ Setting the current date and time:

	![Date/time setting](https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/img/time_date_cfg.png "Date/time setting")
	
## Links:

+ [RM0368 Reference manual — STM32F401xB/C and STM32F401xD/E advanced Arm®-based 32-bit MCUs][7].
+ [AN3371 Application note — Using the hardware real-time clock (RTC) in STM32 F0, F2, F3, F4 and L1 series of MCUs][6].
+ [UM1724 User manual — STM32 Nucleo-64 boards (MB1136)][8].

[1]: https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/doc/UA_Report.pdf
[2]: https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/doc/UA_Circuit_diagram.pdf 
[3]: https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/blob/master/doc/UA_Bill_of_materials.pdf  
[4]: https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stsw-stm32091.html
[5]: https://github.com/vsilchuk/STM32F4_CMSIS_RTC_LCD1602_clock/doc/UA_Using_the_STM32F4_hardware_real-time_clock.pdf
[6]: https://www.st.com/resource/en/application_note/dm00025071-using-the-hardware-realtime-clock-rtc-in-stm32-f0-f2-f3-f4-and-l1-series-of-mcus-stmicroelectronics.pdf
[7]: https://www.st.com/resource/en/reference_manual/dm00096844-stm32f401xb-c-and-stm32f401xd-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
[8]: https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf
