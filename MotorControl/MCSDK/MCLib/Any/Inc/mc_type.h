/**
  ******************************************************************************
  * @file    mc_type.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control SDK global types definitions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MCSDK
  * @{
  */
  
/** @addtogroup MC_Type Motor Control types
  * @{
  */

/** 
  * @define MISRA_C_2004_BUILD
  * @brief Uncomment #define MISRA_C_2004_BUILD to build the library including 
  *        "stm32fxxx_MisraCompliance.h" instead of "stm32fxxx.h".
  *
  *        This will build the library in 'strict ISO/ANSI C' and in
  *        compliance with MISRA C 2004 rules (check project options)
  */
/*#define MISRA_C_2004_BUILD*/

#include <mc_stm_types.h>

/**
* @brief macro to use bit banding capability
*/
#define BB_REG_BIT_SET(regAddr,bit_number) *(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000 )<<5) + (bit_number <<2)) = (uint32_t)(0x1u)
#define BB_REG_BIT_CLR(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) = (uint32_t)(0x0u))
#define BB_REG_BIT_READ(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) )


/**
* @brief Not initialized pointer
*/
#define MC_NULL    (uint16_t)(0x0000u)

/**
* @{
*/
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
#define M2      (uint8_t)(0x1)  /*!< Motor 2.*/
#define M_NONE  (uint8_t)(0xFF) /*!< None motor.*/
/**
  * @}
  */

/** @defgroup Fault_generation_error_codes Fault generation error codes definition
* @{
*/

#define  MC_NO_ERROR  (uint16_t)(0x0000u)      /*!<No error.*/
#define  MC_NO_FAULTS  (uint16_t)(0x0000u)     /*!<No error.*/
#define  MC_FOC_DURATION  (uint16_t)(0x0001u)  /*!<Error: FOC rate to high.*/
#define  MC_OVER_VOLT  (uint16_t)(0x0002u)     /*!<Error: Software over voltage.*/
#define  MC_UNDER_VOLT  (uint16_t)(0x0004u)    /*!<Error: Software under voltage.*/
#define  MC_OVER_TEMP  (uint16_t)(0x0008u)     /*!<Error: Software over temperature.*/
#define  MC_START_UP  (uint16_t)(0x0010u)      /*!<Error: Startup failed.*/
#define  MC_SPEED_FDBK  (uint16_t)(0x0020u)    /*!<Error: Speed feedback.*/
#define  MC_BREAK_IN  (uint16_t)(0x0040u)      /*!<Error: Emergency input (Over current).*/
#define  MC_SW_ERROR  (uint16_t)(0x0080u)       /*!<Software Error.*/

/**
  * @}
  */
  
/** @brief Dual motor Frequency comparison definition
* @{
*/
#define SAME_FREQ   0u
#define HIGHER_FREQ 1u
#define LOWER_FREQ  2u

#define HIGHEST_FREQ 1u
#define LOWEST_FREQ  2u
/**
  * @}
  */  
/** @defgroup MCType_exported_types Exported types
* @{
*/

/** 
  * @brief  Two components stator current type definition 
  */
typedef struct 
{
  int16_t qI_Component1;
  int16_t qI_Component2;
} Curr_Components;

/** 
  * @brief  Two components stator voltage type definition 
  */
typedef struct 
{
  int16_t qV_Component1;
  int16_t qV_Component2;
} Volt_Components;

/** 
  * @brief  ADConv_t type definition, it is used by PWMC_ADC_SetSamplingTime method of PWMnCurrFdbk class for user defined A/D regular conversions 
  */
typedef struct
{ 
  uint8_t Channel;   /*!< Integer channel number, from 0 to 15 */
  uint8_t SamplTime; /*!< Sampling time selection, ADC_SampleTime_nCycles5 
                          in case of STM32F10x, n= 1, 3, ...; */
} ADConv_t;

/** 
  * @brief  SensorType_t type definition, it's used in BusVoltageSensor and TemperatureSensor component parameters structures
  *		    to specify whether the sensor is real or emulated by SW
  */	
typedef enum
{
REAL_SENSOR, VIRTUAL_SENSOR
} SensorType_t;

/** 
  * @brief  DOutputState_t type definition, it's used by DOUT_SetOutputState method of DigitalOutput class to specify the 
  *			required output state
  */
typedef enum
{
 INACTIVE, ACTIVE
} DOutputState_t;


/** 
  * @brief  STC_Modality_t type definition, it's used by STC_SetControlMode and STC_GetControlMode methods in
  *         SpeednTorqCtrl class to specify the control modality type
  */
typedef enum
{
	STC_TORQUE_MODE, /*!<Torque mode.*/
	STC_SPEED_MODE   /*!<Speed mode.*/
} STC_Modality_t;


/** 
  * @brief IMFF_PMSM class, structure type definition for feed-forward constants
  *        tuning 
  */
typedef struct
{
  int32_t wConst_1D;
  int32_t wConst_1Q;
  int32_t wConst_2;  
} IMFF_TuningStruct_t, FF_TuningStruct_t;

/** 
  * @brief  Current references source type, internal or external to FOCDriveClass 
  */
typedef enum
{
  INTERNAL, EXTERNAL
} CurrRefSource_t ;

/** 
  * @brief  FOC variables structure
  */
typedef struct
{
  Curr_Components Iab;         /*!< Stator current on stator reference frame abc */
  Curr_Components Ialphabeta;  /*!< Stator current on stator reference frame 
                                    alfa-beta*/
  Curr_Components IqdHF;       /*!< Stator current on stator reference frame 
                                    alfa-beta*/                                     
  Curr_Components Iqd;         /*!< Stator current on rotor reference frame qd */ 
  Curr_Components Iqdref;      /*!< Stator current on rotor reference frame qd */ 
  int16_t UserIdref;           /*!< User value for the Idref stator current */ 
  Volt_Components Vqd;         /*!< Phase voltage on rotor reference frame qd */ 
  Volt_Components Valphabeta;  /*!< Phase voltage on stator reference frame 
                                   alpha-beta*/ 
  int16_t hTeref;              /*!< Reference torque */ 
  int16_t hElAngle;            /*!< Electrical angle used for reference frame 
                                    transformation  */
  uint16_t hCodeError;         /*!< error message */
  CurrRefSource_t bDriveInput; /*!< It specifies whether the current reference 
                                    source must be INTERNAL or EXTERNAL*/
} FOCVars_t, *pFOCVars_t;

/** 
  * @brief  Low side or enabling signal definition  
  */
#define LS_DISABLED   0
#define LS_PWM_TIMER  1
#define ES_GPIO       2

typedef enum
{
  LS_DISABLED_VAL = LS_DISABLED,    /*!< Low side signals and enabling signals always off.
                                         It is equivalent to DISABLED. */
  LS_PWM_TIMER_VAL = LS_PWM_TIMER,  /*!< Low side PWM signals are generated by timer. It is
                                         equivalent to ENABLED. */
  ES_GPIO_VAL = ES_GPIO             /*!< Enabling signals are managed by GPIOs (L6230 mode).*/
} LowSideOutputsFunction_t, *pLowSideOutputsFunction_t;

/** 
  * @brief  MPInfo structure (used only for serial communication)
  */
typedef struct
{
  uint8_t* data;
  uint8_t len;
} MPInfo_t, *pMPInfo_t;

/**
  * @}
  */

/** @defgroup MCType_UserInterfaceRelated_exported_definitions UserInterface related exported definitions
* @{
*/

#define OPT_NONE    0x00 /*!<No UI option selected.*/
#define OPT_LCD     0x01 /*!<Bit field indicating that the UI uses LCD manager.*/
#define OPT_COM     0x02 /*!<Bit field indicating that the UI uses serial communication.*/
#define OPT_DAC     0x04 /*!<Bit field indicating that the UI uses real DAC.*/
#define OPT_DACT    0x08 /*!<Bit field indicating that the UI uses RC Timer DAC.*/
#define OPT_DACS    0x10 /*!<Bit field indicating that the UI uses SPI communication.*/
#define OPT_DACF3   0x40 /*!<Bit field indicating that the UI uses DAC for STM32F3.*/
#define OPT_DACF072 0x80 /*!<Bit field indicating that the UI uses DAC for STM32F072.*/

/**
  * @}
  */
  
#define MAIN_SCFG_POS (28)
#define AUX_SCFG_POS (24)

#define MAIN_SCFG_VALUE(x) (((x)>>MAIN_SCFG_POS)&0x0F)
#define AUX_SCFG_VALUE(x)  (((x)>>AUX_SCFG_POS)&0x0F)

/** @defgroup MCType_UserInterfaceRelated_exported_definitions UserInterface related exported definitions
* @{
*/

#define PFC_SWE             0x0001u /*!<PFC Software error.*/
#define PFC_HW_PROT         0x0002u /*!<PFC hardware protection.*/
#define PFC_SW_OVER_VOLT    0x0004u /*!<PFC software over voltage.*/
#define PFC_SW_OVER_CURRENT 0x0008u /*!<PFC software over current.*/
#define PFC_SW_MAINS_FREQ   0x0010u /*!<PFC mains frequency error.*/
#define PFC_SW_MAIN_VOLT    0x0020u /*!<PFC mains voltage error.*/

/**
  * @}
  */

/** @defgroup MCType_DACUsedAsReferenceForProtectionRelated_exported_definitions DAC channel used as reference for protection exported definitions
* @{
*/

#define AO_DISABLED 0x00u /*!<Analog output disabled.*/
#define AO_DEBUG    0x01u /*!<Analog output debug.*/
#define VREF_OCPM1  0x02u /*!<Voltage reference for over current protection of motor 1.*/
#define VREF_OCPM2  0x03u /*!<Voltage reference for over current protection of motor 2.*/
#define VREF_OCPM12 0x04u /*!<Voltage reference for over current protection of both motors.*/
#define VREF_OVPM12 0x05u /*!<Voltage reference for over voltage protection of both motors.*/

/**
  * @}
  */

/** @defgroup MCType_Utils_exported_definitions Utillity fuinctions definitions
* @{
*/

#define RPM2MEC01HZ(rpm) (int16_t)((int32_t)(rpm)/6)

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
