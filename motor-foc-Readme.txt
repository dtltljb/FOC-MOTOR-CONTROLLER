# 概述
基于STM32有感直流无刷电机驱动控制程序，集成CANopen通讯电机控制子集协议 和 MONDBUS 协议。


#硬件资源配置表如下:

Configuration	STM32F302R8-FOC
STM32CubeMX 	4.27.0
Date	05/25/2019
MCU	STM32F302R8Tx

PERIPHERALS	MODES	FUNCTIONS	PINS
ADC1	IN4 Single-ended	ADC1_IN4	PA3
ADC1	IN6 Single-ended	ADC1_IN6	PC0
ADC1	IN7 Single-ended	ADC1_IN7	PC1
ADC1	IN8 Single-ended	ADC1_IN8	PC2
ADC1	IN9 Single-ended	ADC1_IN9	PC3
ADC1	IN10 Single-ended	ADC1_IN10	PA6
ADC1	IN11 Single-ended	ADC1_IN11	PB0
ADC1	IN12 Single-ended	ADC1_IN12	PB1
ADC1	Single-ended	ADC1_TempSens_Input	VP_ADC1_TempSens_Input
CAN	Master	CAN_RX	PB8
CAN	Master	CAN_TX	PB9
DAC	OUT1 Configuration	DAC_OUT1	PA4
RCC	Crystal/Ceramic Resonator	RCC_OSC_IN	PF0-OSC_IN
RCC	Crystal/Ceramic Resonator	RCC_OSC_OUT	PF1-OSC_OUT
SYS	SysTick	SYS_VS_Systick	VP_SYS_VS_Systick
TIM1	Trigger Mode	TIM1_VS_ControllerModeTrigger	VP_TIM1_VS_ControllerModeTrigger
TIM1	ITR1	TIM1_VS_ClockSourceITR	VP_TIM1_VS_ClockSourceITR
TIM1	PWM Generation CH1 CH1N	TIM1_CH1	PA8
TIM1	PWM Generation CH1 CH1N	TIM1_CH1N	PB13
TIM1	PWM Generation CH2 CH2N	TIM1_CH2	PA9
TIM1	PWM Generation CH2 CH2N	TIM1_CH2N	PB14
TIM1	PWM Generation CH3 CH3N	TIM1_CH3	PA10
TIM1	PWM Generation CH3 CH3N	TIM1_CH3N	PB15
TIM1	PWM Generation No Output	TIM1_VS_no_output4	VP_TIM1_VS_no_output4
TIM1	Activate-Break-Input-2	TIM1_BKIN2	PA11
TIM2	Internal Clock	TIM2_VS_ClockSourceINT	VP_TIM2_VS_ClockSourceINT
TIM2	XOR ON / Hall Sensor Mode	TIM2_CH1	PA0
TIM2	XOR ON / Hall Sensor Mode	TIM2_CH2	PA1
TIM2	XOR ON / Hall Sensor Mode	TIM2_CH3	PA2
USART1	Asynchronous	USART1_RX	PB7
USART1	Asynchronous	USART1_TX	PB6
USART3	Asynchronous	USART3_RX	PB11
USART3	Asynchronous	USART3_TX	PB10



Pin Nb	PINs	FUNCTIONs	LABELs
5	PF0-OSC_IN	RCC_OSC_IN	
6	PF1-OSC_OUT	RCC_OSC_OUT	
8	PC0	ADC1_IN6	M1_CURR_AMPL_U
9	PC1	ADC1_IN7	M1_CURR_AMPL_V
10	PC2	ADC1_IN8	M1_CURR_AMPL_W
11	PC3	ADC1_IN9	M1_BUS_VOLTAGE
14	PA0	TIM2_CH1	M1_HALL_H1
15	PA1	TIM2_CH2	M1_HALL_H2
16	PA2	TIM2_CH3	M1_HALL_H3
17	PA3	ADC1_IN4	TEMP_IN
20	PA4	DAC_OUT1	DBG_DAC_CH1
22	PA6	ADC1_IN10	BEMP_U
24	PC4	GPIO_EXTI4	BEEP_EVT
26	PB0	ADC1_IN11	BEMP_V
27	PB1	ADC1_IN12	BEMP_W
29	PB10	USART3_TX	DBG_TX3
30	PB11	USART3_RX	DBG_RX2
34	PB13	TIM1_CH1N	M1_PWM_UL
35	PB14	TIM1_CH2N	M1_PWM_VL
36	PB15	TIM1_CH3N	M1_PWM_WL
41	PA8		TIM1_CH1	M1_PWM_UH
42	PA9		TIM1_CH2	M1_PWM_VH
43	PA10	TIM1_CH3	M1_PWM_WH
44	PA11	TIM1_BKIN2	M1_OCP
58	PB6	USART1_TX	R485_TX1
59	PB7	USART1_RX	R485_RX1
61	PB8	CAN_RX	
62	PB9	CAN_TX	



SOFTWARE PROJECT

Project Settings : 
Project Name : STM32F302R8-FOC
Project Folder : D:\motor\FOC5.0STM32F302R8\STM32F302R8-FOC
Toolchain / IDE : MDK-ARM V5
Firmware Package Name and Version : STM32Cube FW_F3 V1.10.0

Code Generation Settings : 
STM32Cube Firmware Library Package : Copy all used libraries into the project folder
Generate peripheral initialization as a pair of '.c/.h' files per peripheral : No
Backup previously generated files when re-generating : No
Delete previously generated files when not re-generated : Yes
Set all free pins as analog (to optimize the power consumption) : No


Toolchains Settings : 
Compiler Optimizations : 





