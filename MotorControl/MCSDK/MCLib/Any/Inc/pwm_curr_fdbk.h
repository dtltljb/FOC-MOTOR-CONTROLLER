/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PWM & Current Feedback component of the Motor Control SDK.
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
#ifndef __PWMNCURRFDBK_H
#define __PWMNCURRFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u
#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct PWMC_Handle PWMC_Handle_t;

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to one of the callback pointers
  *        pFctSwitchOffPwm
  *        pFctSwitchOnPwm
  *        pFctCurrReadingCalib
  *        pFctTurnOnLowSides
  *        pFctRLDetectionModeEnable
  *        pFctRLDetectionModeDisable
  *
  *
  */
typedef void (*PWMC_Generic_Cb_t)( PWMC_Handle_t *pHandle);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctIrqHandler
  *
  */
typedef void* (*PWMC_IrqHandler_Cb_t)( PWMC_Handle_t *pHandle, unsigned char flag);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctGetPhaseCurrents
  *
  */
typedef void (*PWMC_GetPhaseCurr_Cb_t)(PWMC_Handle_t *pHandle, Curr_Components* pStator_Currents);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctSetSamplingTime
  *
  */
typedef void (*PWMC_SetSampTime_Cb_t)(PWMC_Handle_t *pHandle, ADConv_t ADConv_struct);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to one of the callback pointers
  *
  */
typedef void (*PWMC_SetOcpRefVolt_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDACVref);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to one of callback pointers
  *        pFctSetADCSampPointSect1
  *        pFctSetADCSampPointSect2
  *        pFctSetADCSampPointSect3
  *        pFctSetADCSampPointSect4
  *        pFctSetADCSampPointSect5
  *        pFctSetADCSampPointSect6
  *
  */
typedef uint16_t (*PWMC_SetSampPointSectX_Cb_t)( PWMC_Handle_t *pHandle);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctRegularConvExec
  *
  */
typedef uint16_t (*PWMC_RegConvExec_Cb_t)(PWMC_Handle_t *pHandle, uint8_t bChannel);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctIsOverCurrentOccurred
  *
  */
typedef uint16_t (*PWMC_OverCurr_Cb_t)(PWMC_Handle_t *pHandle);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctRLDetectionModeSetDuty
  *
  */
typedef uint16_t (*PWMC_RLDetectSetDuty_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDuty);

/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_IrqHandler_Cb_t pFctIrqHandler;
  PWMC_GetPhaseCurr_Cb_t pFctGetPhaseCurrents;
  PWMC_Generic_Cb_t pFctSwitchOffPwm;
  PWMC_Generic_Cb_t pFctSwitchOnPwm;
  PWMC_Generic_Cb_t pFctCurrReadingCalib;
  PWMC_Generic_Cb_t pFctTurnOnLowSides;
  PWMC_SetSampTime_Cb_t pFctSetSamplingTime;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect1;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect2;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect3;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect4;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect5;
  PWMC_SetSampPointSectX_Cb_t pFctSetADCSampPointSect6;
  PWMC_RegConvExec_Cb_t pFctRegularConvExec;
  PWMC_OverCurr_Cb_t pFctIsOverCurrentOccurred;
  PWMC_SetOcpRefVolt_Cb_t pFctOCPSetReferenceVoltage;
  PWMC_Generic_Cb_t pFctRLDetectionModeEnable;
  PWMC_Generic_Cb_t pFctRLDetectionModeDisable;
  PWMC_RLDetectSetDuty_Cb_t pFctRLDetectionModeSetDuty;
  /** @} */
  uint16_t  hT_Sqrt3;   /*!< Contains a constant utilized by pwm algorithm */
  uint16_t  hSector;    /*!< Contains the space vector sector number */
  uint16_t  hCntPhA;    /*!< Contains duty cycle for phase A */
  uint16_t  hCntPhB;    /*!< Contains duty cycle for phase B */
  uint16_t  hCntPhC;    /*!< Contains duty cycle for phase C */
  uint16_t  SWerror;    /*!< Contains status about SW error */
  bool bTurnOnLowSidesAction; /*!< true if TurnOnLowSides action is active,
                                   false otherwise. */
  uint16_t  hOffCalibrWaitTimeCounter; /*!< Counter to wait fixed time before
                                            motor current measurement offset
                                            calibration. */
  uint8_t   bMotor;     /*!< Motor reference number */
  bool      RLDetectionMode; /*!< true if enabled, false if disabled. */
  int16_t   hIa; /* Last Ia measurement. */
  int16_t   hIb; /* Last Ib measurement. */
  int16_t   hIc; /* Last Ic measurement. */
  uint16_t  DTTest;    /* Reserved */
  uint16_t  DTCompCnt; /* Reserved */

  /* former  PWMnCurrFdbkParams_t */
  uint16_t hPWMperiod;            /*!< It contains the PWM period expressed in
                                       timer clock cycles unit:
                                       hPWMPeriod = Timer Fclk / Fpwm    */
  uint16_t hOffCalibrWaitTicks;   /*!< Wait time duration before current reading
                                       calibration expressed in number of calls
                                       of PWMC_CurrentReadingCalibr with action
                                       CRC_EXEC */
  uint16_t hDTCompCnt;            /*!< It contains half of Dead time expressed
                                       in timer clock cycles unit:
                                       hDTCompCnt = (DT(s) * Timer Fclk)/2 */
  uint16_t  Ton;                  /*!< Reserved */
  uint16_t  Toff;                 /*!> Reserved */

};

/**
  * @brief  Current reading calibration definition
  */
typedef enum CRCAction
{
  CRC_START, /*!< Initialize the current reading calibration.*/
  CRC_EXEC   /*!< Execute the current reading calibration.*/
} CRCAction_t;


/* It is used to get the motor phase current in Curr_Components format
   as read by AD converter */
void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle,
                           Curr_Components* pStator_Currents);

/*  It converts input voltage components Valfa, beta into duty cycles
    and feed it to the inverter */
uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle,
                              Volt_Components Valfa_beta);

/* It switch off the PWM generation, setting to inactive the outputs */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle);

/* It switch on the PWM generation */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle);

/* It calibrates ADC current conversions by reading the offset voltage
   present on ADC pins when no motor current is flowing. It's suggested
   to call this function before each motor start-up */
bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle,
                               CRCAction_t action);

/* It turns on low sides. This function is intended to be used for
   charging boot capacitors of driving section. It has to be called each
   motor start-up when using high voltage drivers */
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle);

/* Execute a regular conversion using ADC1.  The function is not
   re-entrant (can't executed twice at the same time). It returns 0xFFFF
   in case of conversion error. */
uint16_t PWMC_ExecRegularConv(PWMC_Handle_t *pHandle,
                              uint8_t bChannel);

/* It sets the specified sampling time for the specified ADC channel
   on ADC1. It must be called once for each channel utilized by user */
void PWMC_ADC_SetSamplingTime(PWMC_Handle_t *pHandle,
                              ADConv_t ADConv_struct);

/* It is used to check if an overcurrent occurred since last call. */
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle);

/* It is used to set the overcurrent threshold through the DAC reference
   voltage. */
void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle,
                                 uint16_t hDACVref);

/* It is used to retrieve the satus of TurnOnLowSides action. */
bool PWMC_GetTurnOnLowSidesAction(PWMC_Handle_t *pHandle);

/* It is used to set the RL Detection mode. */
void PWMC_RLDetectionModeEnable(PWMC_Handle_t *pHandle);

/* It is used to disable the RL Detection mode and set the standard PWM. */
void PWMC_RLDetectionModeDisable(PWMC_Handle_t *pHandle);

/* It is used to set the PWM dutycycle in the RL Detection mode. */
uint16_t PWMC_RLDetectionModeSetDuty(PWMC_Handle_t *pHandle,
                                     uint16_t hDuty);

/* Sets the Callback which PWMC shall invoke to get phases current. */
void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
                                            PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to switch off PWM
 *        generation. */
void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to switch on PWM
 *        generation. */
void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                      PWMC_Handle_t* pHandle );

/*  Sets the Callback which PWMC shall invoke to execute a calibration
 *        of the current sensing system. */
void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
                                              PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to turn on low sides.
 * @param pCallBack pointer on the callback */
void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to set a new sampling
 * time on a ADC channel. */
void PWMC_RegisterSetSamplingTimeCallBack( PWMC_SetSampTime_Cb_t pCallBack,
                                           PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 1. */
void PWMC_RegisterSampPointSect1CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/*  Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 2. */
void PWMC_RegisterSampPointSect2CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 3 */
void PWMC_RegisterSampPointSect3CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/*  Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 4. */
void PWMC_RegisterSampPointSect4CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 5. */
void PWMC_RegisterSampPointSect5CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 6.;*/
void PWMC_RegisterSampPointSect6CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to execute a regular ADC
 * conversion */
void PWMC_RegisterRegularConvExecCallBack( PWMC_RegConvExec_Cb_t pCallBack,
                                           PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback */
void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
                                                 PWMC_Handle_t* pHandle );
/* Sets the Callback which PWMC shall invoke to set the reference
 * voltage for the overcurrent protection */
void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
                                            PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to set the R/L detection
 * mode */
void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
                                                 PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to disable the R/L detection
 * mode */
void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
                                                  PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to set the duty cycle
 * for the R/L detection mode */
void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
                                                  PWMC_Handle_t* pHandle );

/* Sets the Callback which PWMC shall invoke to call PWMC instance IRQ handler */
void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t* pHandle );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
