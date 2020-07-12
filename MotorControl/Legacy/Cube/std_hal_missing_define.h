#ifndef STD_HAL_MISSING_DEFINE
#define STD_HAL_MISSING_DEFINE
/* Defined of STDLIB used by WB and FW */
#if 0
#define TIM_BreakPolarity_Low TIM_BREAKPOLARITY_LOW

#define ADC_SampleTime_3Cycles                    ((uint8_t)0x00)
#define ADC_SampleTime_15Cycles                   ((uint8_t)0x01)
#define ADC_SampleTime_28Cycles                   ((uint8_t)0x02)
#define ADC_SampleTime_56Cycles                   ((uint8_t)0x03)
#define ADC_SampleTime_84Cycles                   ((uint8_t)0x04)
#define ADC_SampleTime_112Cycles                  ((uint8_t)0x05)
#define ADC_SampleTime_144Cycles                  ((uint8_t)0x06)
#define ADC_SampleTime_480Cycles                  ((uint8_t)0x07)
#define ADC_Prescaler_Div2                         ((uint32_t)0x00000000)
#define ADC_Prescaler_Div4                         ((uint32_t)0x00010000)
#define ADC_Prescaler_Div6                         ((uint32_t)0x00020000)
#define ADC_Prescaler_Div8                         ((uint32_t)0x00030000)


#define ADC_Channel_0                               ((uint8_t)0x00)
#define ADC_Channel_1                               ((uint8_t)0x01)    /*!<  ADC Channel 1 */
#define ADC_Channel_2                               ((uint8_t)0x02)    /*!<  ADC Channel 2 */
#define ADC_Channel_3                               ((uint8_t)0x03)    /*!<  ADC Channel 3 */
#define ADC_Channel_4                               ((uint8_t)0x04)    /*!<  ADC Channel 4 */
#define ADC_Channel_5                               ((uint8_t)0x05)    /*!<  ADC Channel 5 */
#define ADC_Channel_6                               ((uint8_t)0x06)    /*!<  ADC Channel 6 */
#define ADC_Channel_7                               ((uint8_t)0x07)    /*!<  ADC Channel 7 */
#define ADC_Channel_8                               ((uint8_t)0x08)    /*!<  ADC Channel 8 */
#define ADC_Channel_9                               ((uint8_t)0x09)    /*!<  ADC Channel 9 */
#define ADC_Channel_10                              ((uint8_t)0x0A)    /*!<  ADC Channel 10 */
#define ADC_Channel_11                              ((uint8_t)0x0B)    /*!<  ADC Channel 11 */
#define ADC_Channel_12                              ((uint8_t)0x0C)    /*!<  ADC Channel 12 */
#define ADC_Channel_13                              ((uint8_t)0x0D)    /*!<  ADC Channel 13 */
#define ADC_Channel_14                              ((uint8_t)0x0E)    /*!<  ADC Channel 14 */
#define ADC_Channel_15                              ((uint8_t)0x0F)    /*!<  ADC Channel 15 */
#define ADC_Channel_16                              ((uint8_t)0x10)    /*!<  ADC Channel 16 */
#define ADC_Channel_17                              ((uint8_t)0x11)    /*!<  ADC Channel 17 */
#define ADC_Channel_18                              ((uint8_t)0x12)    /*!<  ADC Channel 18 */

#define ADC_SampleTime_1Cycles5                    ((uint8_t)0x00)   /*!<  ADC sampling time 1.5 cycle */
#define ADC_SampleTime_2Cycles5                    ((uint8_t)0x01)   /*!<  ADC sampling time 2.5 cycles */
#define ADC_SampleTime_4Cycles5                    ((uint8_t)0x02)   /*!<  ADC sampling time 4.5 cycles */
#define ADC_SampleTime_7Cycles5                    ((uint8_t)0x03)   /*!<  ADC sampling time 7.5 cycles */
#define ADC_SampleTime_19Cycles5                   ((uint8_t)0x04)   /*!<  ADC sampling time 19.5 cycles */
#define ADC_SampleTime_61Cycles5                   ((uint8_t)0x05)   /*!<  ADC sampling time 61.5 cycles */
#define ADC_SampleTime_181Cycles5                  ((uint8_t)0x06)   /*!<  ADC sampling time 181.5 cycles */
#define ADC_SampleTime_601Cycles5                  ((uint8_t)0x07)   /*!<  ADC sampling time 601.5 cycles */

/** 
  * @brief  AF 0 selection
  */ 
#define GPIO_AF_0            ((uint8_t)0x00) /* JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT,  
                                                MCO, NJTRST, TRACED, TRACECK */
/** 
  * @brief  AF 1 selection
  */ 
#define GPIO_AF_1            ((uint8_t)0x01) /*  OUT, TIM2, TIM15, TIM16, TIM17 */

/** 
  * @brief  AF 2 selection
  */ 
#define GPIO_AF_2            ((uint8_t)0x02) /* COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16 */

/** 
  * @brief  AF 3 selection
  */ 
#define GPIO_AF_3            ((uint8_t)0x03) /* COMP7_OUT, TIM8, TIM15, Touch, HRTIM1 */

/** 
  * @brief  AF 4 selection
  */ 
#define GPIO_AF_4            ((uint8_t)0x04) /* I2C1, I2C2, TIM1, TIM8, TIM16, TIM17 */

/** 
  * @brief  AF 5 selection
  */ 
#define GPIO_AF_5            ((uint8_t)0x05) /* IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5 */

/** 
  * @brief  AF 6 selection
  */ 
#define GPIO_AF_6            ((uint8_t)0x06) /*  IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8 */

/** 
  * @brief  AF 7 selection
  */ 
#define GPIO_AF_7            ((uint8_t)0x07) /* AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, 
                                                USART1, USART2, USART3 */

/** 
  * @brief  AF 8 selection
  */ 
#define GPIO_AF_8            ((uint8_t)0x08) /* COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, 
                                                COMP5_OUT, COMP6_OUT */

/** 
  * @brief  AF 9 selection
  */ 
#define GPIO_AF_9            ((uint8_t)0x09) /* AOP4_OUT, CAN, TIM1, TIM8, TIM15 */

/** 
  * @brief  AF 10 selection
  */ 
#define GPIO_AF_10            ((uint8_t)0x0A) /* AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17 */

/** 
  * @brief  AF 11 selection
  */ 
#define GPIO_AF_11            ((uint8_t)0x0B) /* TIM1, TIM8 */

/** 
   * @brief  AF 12 selection
   */ 
#define GPIO_AF_12            ((uint8_t)0x0C) /* TIM1, HRTIM1 */

/** 
   * @brief  AF 13 selection
   */ 
#define GPIO_AF_13            ((uint8_t)0x0D) /* HRTIM1, AOP2_OUT */

/** 
  * @brief  AF 14 selection
  */ 
#define GPIO_AF_14            ((uint8_t)0x0E) /* USBDM, USBDP */

/** 
  * @brief  AF 15 selection
  */ 
#define GPIO_AF_15            ((uint8_t)0x0F) /* OUT */


#define TIM_IT_Break TIM_IT_BREAK

#define TIM_OCIdleState_Reset   LL_TIM_OCIDLESTATE_LOW
#define TIM_OCNIdleState_Reset  LL_TIM_OCIDLESTATE_LOW
#define TIM_OCNIdleState_Set    TIM_CR2_OIS1N
#define TIM_Break1Polarity_Low	LL_TIM_BREAK_POLARITY_LOW
#define TIM_Break1Polarity_High	LL_TIM_BREAK_POLARITY_HIGH
#define TIM_Break2Polarity_Low  LL_TIM_BREAK2_POLARITY_LOW
#define TIM_Break2Polarity_High LL_TIM_BREAK2_POLARITY_HIGH

#define ADC_Clock_AsynClkMode                  ((uint32_t)0x00000000)   /*!< ADC Asynchronous clock mode */
#define ADC_Clock_SynClkModeDiv1               ((uint32_t)0x00010000)   /*!< Synchronous clock mode divided by 1 */
#define ADC_Clock_SynClkModeDiv2               ((uint32_t)0x00020000)   /*!<  Synchronous clock mode divided by 2 */
#define ADC_Clock_SynClkModeDiv4               ((uint32_t)0x00030000)   /*!<  Synchronous clock mode divided by 4 */
#define IS_ADC_CLOCKMODE(CLOCK) (((CLOCK) == ADC_Clock_AsynClkMode) ||\
				((CLOCK) == ADC_Clock_SynClkModeDiv1) ||\
				((CLOCK) == ADC_Clock_SynClkModeDiv2)||\
				((CLOCK) == ADC_Clock_SynClkModeDiv4))
						  
#define RCC_AHBPeriph_ADC34               RCC_AHBENR_ADC3EN
#define RCC_AHBPeriph_ADC12               RCC_AHBENR_ADC12EN
#define RCC_AHBPeriph_GPIOA               RCC_AHBENR_GPIOAEN
#define RCC_AHBPeriph_GPIOB               RCC_AHBENR_GPIOBEN
#define RCC_AHBPeriph_GPIOC               RCC_AHBENR_GPIOCEN
#define RCC_AHBPeriph_GPIOD               RCC_AHBENR_GPIODEN
#define RCC_AHBPeriph_GPIOE               RCC_AHBENR_GPIOEEN
#define RCC_AHBPeriph_GPIOF               RCC_AHBENR_GPIOFEN
#define RCC_AHBPeriph_TS                  RCC_AHBENR_TSEN
#define RCC_AHBPeriph_CRC                 RCC_AHBENR_CRCEN
#define RCC_AHBPeriph_FLITF               RCC_AHBENR_FLITFEN
#define RCC_AHBPeriph_SRAM                RCC_AHBENR_SRAMEN
#define RCC_AHBPeriph_DMA2                RCC_AHBENR_DMA2EN
#define RCC_AHBPeriph_DMA1                RCC_AHBENR_DMA1EN

#define OPAMP_Selection_OPAMP1                    ((uint32_t)0x00000000) /*!< OPAMP1 Selection */
#define OPAMP_Selection_OPAMP2                    ((uint32_t)0x00000004) /*!< OPAMP2 Selection */
#define OPAMP_Selection_OPAMP3                    ((uint32_t)0x00000008) /*!< OPAMP3 Selection */
#define OPAMP_Selection_OPAMP4                    ((uint32_t)0x0000000C) /*!< OPAMP4 Selection */

#define OPAMP_InvertingInput_IO1          ((uint32_t)0x00000000) /*!< IO1 (PC5 for OPAMP1 and OPAMP2, PB10 for OPAMP3 and OPAMP4)
                                                                     connected to OPAMPx inverting input */
#define OPAMP_InvertingInput_IO2          OPAMP_CSR_VMSEL_0      /*!< IO2 (PA3 for OPAMP1, PA5 for OPAMP2, PB2 for OPAMP3, PD8 for OPAMP4)
                                                                      connected to OPAMPx inverting input */
#define OPAMP_InvertingInput_PGA          OPAMP_CSR_VMSEL_1      /*!< Resistor feedback output connected to OPAMPx inverting input (PGA mode) */
#define OPAMP_InvertingInput_Vout         OPAMP_CSR_VMSEL        /*!< Vout connected to OPAMPx inverting input (follower mode) */

#define OPAMP_NonInvertingInput_IO1          ((uint32_t)0x00000000) /*!< IO1 (PA7 for OPAMP1, PD14 for OPAMP2, PB13 for OPAMP3, PD11 for OPAMP4)
                                                                        connected to OPAMPx non inverting input */
#define OPAMP_NonInvertingInput_IO2          OPAMP_CSR_VPSEL_0      /*!< IO2 (PA5 for OPAMP1, PB14 for OPAMP2, PA5 for OPAMP3, PB11 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
#define OPAMP_NonInvertingInput_IO3          OPAMP_CSR_VPSEL_1      /*!< IO3 (PA3 for OPAMP1, PB0 for OPAMP2, PA1 for OPAMP3, PA4 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
#define OPAMP_NonInvertingInput_IO4          OPAMP_CSR_VPSEL        /*!< IO4 (PA1 for OPAMP1, PA7 for OPAMP2, PB0 for OPAMP3, PB13 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
#define OPAMP_OPAMP_PGAGain_2                ((uint32_t)0x00000000)
#define OPAMP_OPAMP_PGAGain_4                OPAMP_CSR_PGGAIN_0
#define OPAMP_OPAMP_PGAGain_8                OPAMP_CSR_PGGAIN_1
#define OPAMP_OPAMP_PGAGain_16               ((uint32_t)0x0000C000)

#define OPAMP_PGAConnect_No                ((uint32_t)0x00000000)
#define OPAMP_PGAConnect_IO1               OPAMP_CSR_PGGAIN_3
#define OPAMP_PGAConnect_IO2               ((uint32_t)0x00030000)

#define COMP_Selection_COMP1                    ((uint32_t)0x00000000) /*!< COMP1 Selection */
#define COMP_Selection_COMP2                    ((uint32_t)0x00000004) /*!< COMP2 Selection */
#define COMP_Selection_COMP3                    ((uint32_t)0x00000008) /*!< COMP3 Selection */
#define COMP_Selection_COMP4                    ((uint32_t)0x0000000C) /*!< COMP4 Selection */
#define COMP_Selection_COMP5                    ((uint32_t)0x00000010) /*!< COMP5 Selection */
#define COMP_Selection_COMP6                    ((uint32_t)0x00000014) /*!< COMP6 Selection */
#define COMP_Selection_COMP7                    ((uint32_t)0x00000018) /*!< COMP7 Selection */

#define COMP_InvertingInput_1_4VREFINT          ((uint32_t)0x00000000) /*!< 1/4 VREFINT connected to comparator inverting input */
#define COMP_InvertingInput_1_2VREFINT          COMP_CSR_COMPxINSEL_0  /*!< 1/2 VREFINT connected to comparator inverting input */
#define COMP_InvertingInput_3_4VREFINT          COMP_CSR_COMPxINSEL_1  /*!< 3/4 VREFINT connected to comparator inverting input */
#define COMP_InvertingInput_VREFINT             ((uint32_t)0x00000030) /*!< VREFINT connected to comparator inverting input */
#define COMP_InvertingInput_DAC1OUT1            COMP_CSR_COMPxINSEL_2  /*!< DAC1_OUT1 (PA4) connected to comparator inverting input */
#define COMP_InvertingInput_DAC1OUT2            ((uint32_t)0x00000050) /*!< DAC1_OUT2 (PA5) connected to comparator inverting input */

#define COMP_InvertingInput_IO1                 ((uint32_t)0x00000060) /*!< I/O1 (PA0 for COMP1, PA2 for COMP2, PD15 for COMP3, 
                                                                            PE8 for COMP4, PD13 for COMP5, PD10 for COMP6,
                                                                            PC0 for COMP7) connected to comparator inverting input */

#define COMP_InvertingInput_IO2                 COMP_CSR_COMPxINSEL    /*!< I/O2 (PB12 for COMP3, PB2 for COMP4, PB10 for COMP5,
                                                                            PB15 for COMP6) connected to comparator inverting input */

#define COMP_InvertingInput_DAC2OUT1            COMP_CSR_COMPxINSEL_3  /*!< DAC2_OUT1 (PA6) connected to comparator inverting input */

#define COMP_NonInvertingInput_IO1                 ((uint32_t)0x00000000) /*!< I/O1 (PA1 for COMP1, PA7 for COMP2, PB14 for COMP3, 
                                                                               PB0 for COMP4, PD12 for COMP5, PD11 for COMP6,
                                                                               PA0 for COMP7) connected to comparator non inverting input */

#define COMP_NonInvertingInput_IO2                 COMP_CSR_COMPxNONINSEL /*!< I/O2 (PA3 for COMP2, PD14 for COMP3, PE7 for COMP4, PB13 for COMP5,
                                                                               PB11 for COMP6, PC1 for COMP7) connected to comparator non inverting input */
#define COMP_Output_TIM1BKIN              COMP_CSR_COMPxOUTSEL_0   /*!< COMP output connected to TIM1 Break Input (BKIN) */
#define COMP_Output_TIM1BKIN2             ((uint32_t)0x00000800)   /*!< COMP output connected to TIM1 Break Input 2 (BKIN2) */
#define COMP_Output_TIM8BKIN              ((uint32_t)0x00000C00)   /*!< COMP output connected to TIM8 Break Input (BKIN) */
#define COMP_Output_TIM8BKIN2             ((uint32_t)0x00001000)   /*!< COMP output connected to TIM8 Break Input 2 (BKIN2) */
#define COMP_Output_TIM1BKIN2_TIM8BKIN2   ((uint32_t)0x00001400)   /*!< COMP output connected to TIM1 Break Input 2 and TIM8 Break Input 2 */

#define COMP_OutputPol_NonInverted          ((uint32_t)0x00000000)  /*!< COMP output on GPIO isn't inverted */
#define COMP_OutputPol_Inverted             COMP_CSR_COMPxPOL       /*!< COMP output on GPIO is inverted */

#define COMP_Mode_HighSpeed                     0x00000000            /*!< High Speed */
#define COMP_Mode_MediumSpeed                   COMP_CSR_COMPxMODE_0  /*!< Medium Speed */
#define COMP_Mode_LowPower                      COMP_CSR_COMPxMODE_1  /*!< Low power mode */
#define COMP_Mode_UltraLowPower                 COMP_CSR_COMPxMODE    /*!< Ultra-low power mode */

#ifndef COMP_CSR_COMPxNONINSEL
#define COMP_CSR_COMPxNONINSEL             LL_COMP_INPUT_PLUS_IO2 /*!< Comparator input plus connected to IO2: Same as IO1 */
#endif /* COMP_CSR_COMPxNONINSEL */

#else

#define OPAMP_Selection_OPAMP1       OPAMP1
#define OPAMP_Selection_OPAMP2       OPAMP2
#define OPAMP_Selection_OPAMP3       OPAMP3
#define OPAMP_Selection_OPAMP4       OPAMP4

#define COMP_Selection_COMP1         COMP1
#define COMP_Selection_COMP2         COMP2
#define COMP_Selection_COMP3         COMP3
#define COMP_Selection_COMP4         COMP4
#define COMP_Selection_COMP5         COMP5
#define COMP_Selection_COMP6         COMP6
#define COMP_Selection_COMP7         COMP7

#endif
#endif /* STD_HAL_MISSING_DEFINE */

