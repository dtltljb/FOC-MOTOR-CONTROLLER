/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
  *
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

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM.
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * compoarators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
  *
  * The choice of the implementation mostly depend on these two factors and is performed by the
  * Motor Control Workbench tool.
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It is used to get the motor phase current in Curr_Components format
  *         as read by AD converter.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  pStator_Currents Pointer to the struct that will receive motor current
  *         of phase A and B in Curr_Components format.
  * @retval none.
*/
void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle,Curr_Components* pStator_Currents)
{
  pHandle->pFctGetPhaseCurrents(pHandle, pStator_Currents);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif  
/**
  * @brief  It converts input voltage components Valfa and Vbeta into duty cycles
  *         and feed it to the inverter
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  Valfa_beta: Voltage Components in alfa beta reference frame
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR' 
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, Volt_Components Valfa_beta)
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
  PWMC_SetSampPointSectX_Cb_t pSetADCSamplingPoint;
    
  wUAlpha = Valfa_beta.qV_Component1 * (int32_t)pHandle->hT_Sqrt3;
  wUBeta = -(Valfa_beta.qV_Component2 * (int32_t)(pHandle->hPWMperiod))*2;
 
  wX = wUBeta;
  wY = (wUBeta + wUAlpha)/2;
  wZ = (wUBeta - wUAlpha)/2;
  
  /* Sector calculation from wX, wY, wZ */
  if (wY<0)
  {
    if (wZ<0)
    {
      pHandle->hSector = SECTOR_5;
      wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wY - wZ)/(int32_t)262144);
      wTimePhB = wTimePhA + wZ/131072;
      wTimePhC = wTimePhA - wY/131072;
      pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect5;
    }
    else /* wZ >= 0 */
      if (wX<=0)
      {
        pHandle->hSector = SECTOR_4;
        wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wX - wZ)/(int32_t)262144);
        wTimePhB = wTimePhA + wZ/131072;
        wTimePhC = wTimePhB - wX/131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect4;
      }
      else /* wX > 0 */
      {
        pHandle->hSector = SECTOR_3;
        wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wY - wX)/(int32_t)262144);
        wTimePhC = wTimePhA - wY/131072;
        wTimePhB = wTimePhC + wX/131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect3;
      }
  }
  else /* wY > 0 */
  {
    if (wZ>=0)
    {
      pHandle->hSector = SECTOR_2;
      wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wY - wZ)/(int32_t)262144);
      wTimePhB = wTimePhA + wZ/131072;
      wTimePhC = wTimePhA - wY/131072;             
      pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect2;
    }
    else /* wZ < 0 */
      if (wX<=0)
      {  
        pHandle->hSector = SECTOR_6;
        wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wY - wX)/(int32_t)262144);
        wTimePhC = wTimePhA - wY/131072;
        wTimePhB = wTimePhC + wX/131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect6;
      }
      else /* wX > 0 */
      {
        pHandle->hSector = SECTOR_1;
        wTimePhA = (int32_t)(pHandle->hPWMperiod)/4 + ((wX - wZ)/(int32_t)262144);
        wTimePhB = wTimePhA + wZ/131072;
        wTimePhC = wTimePhB - wX/131072;
        pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect1;
      }
  }
  
  pHandle->hCntPhA = (uint16_t)wTimePhA;
  pHandle->hCntPhB = (uint16_t)wTimePhB;
  pHandle->hCntPhC = (uint16_t)wTimePhC;
  
  if (pHandle->DTTest == 1u)
  {
  /* Dead time compensation */
  if (pHandle->hIa > 0)
  {
    pHandle->hCntPhA += pHandle->DTCompCnt;
  }
  else
  {
    pHandle->hCntPhA -= pHandle->DTCompCnt;
  }
  
  if (pHandle->hIb > 0)
  {
    pHandle->hCntPhB += pHandle->DTCompCnt;
  }
  else
  {
    pHandle->hCntPhB -= pHandle->DTCompCnt;
  }
  
  if (pHandle->hIc > 0)
  {
    pHandle->hCntPhC += pHandle->DTCompCnt;
  }
  else
  {
    pHandle->hCntPhC -= pHandle->DTCompCnt;
  }
  }
  
  return(pSetADCSamplingPoint(pHandle));
}

/**
  * @brief  It switch off the PWM generation, setting to inactive the outputs
  * @param  pHandle Handler of the target instance of the PWMC component
  * @retval none
  */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
  pHandle->pFctSwitchOffPwm(pHandle);
}

/**
  * @brief  It switch on the PWM generation
  * @param  pHandle Handler of the target instance of the PWMC component
  * @retval None
  */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
  pHandle->pFctSwitchOnPwm(pHandle);
}

/**
  * @brief  It calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing. It's suggested
  *         to call this function before each motor start-up
  * @param  pHandle Handler of the target instance of the PWMC component
  * @param  action: it can be CRC_START to initialize the offset calibration or
  *         CRC_EXEC to execute the offset calibration.
  * @retval bool It returns true if the current calibration has been completed
  *         otherwise if is ongoing it returns false.
  */
bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle, CRCAction_t action)
{
  bool retVal = false;
  if (action == CRC_START)
  {
    PWMC_SwitchOffPWM(pHandle);
    pHandle->hOffCalibrWaitTimeCounter = pHandle->hOffCalibrWaitTicks;
    if (pHandle->hOffCalibrWaitTicks == 0u)
    {
      pHandle->pFctCurrReadingCalib(pHandle);
      retVal = true;
    }
  }
  else if (action == CRC_EXEC)
  {
    if (pHandle->hOffCalibrWaitTimeCounter > 0u)
    {
      pHandle->hOffCalibrWaitTimeCounter--;
      if (pHandle->hOffCalibrWaitTimeCounter == 0u)
      {
        pHandle->pFctCurrReadingCalib(pHandle);
        retVal = true;
      }
    }
    else
    {
      retVal = true;
    }
  }
  else
  {
  }
  return retVal;
}

/**
  * @brief  It switch on low sides. This function is intended to be used for
  *         charging boot capacitors of driving section. It has to be called each
  *         motor start-up when using high voltage drivers
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval None
  */
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle)
{
  pHandle->pFctTurnOnLowSides(pHandle);
}

/**
  * @brief  Execute a regular conversion. User must guarantee by
  *         design (i.e. properly selecting tasks priorities) that this function
  *         can only be interrupted by TIMx_UP_ISR and ADC1_2_ISR.
  *         The function is not re-entrant (can't executed twice at the same time)
  *         It returns 0xFFFF in case of conversion error.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  bChannel ADC channel used for the regular conversion
  * @retval It returns converted value or oxFFFF for conversion error
  */
uint16_t PWMC_ExecRegularConv(PWMC_Handle_t *pHandle, uint8_t bChannel)
{ 
  uint16_t hConvValue;
  hConvValue = pHandle->pFctRegularConvExec(pHandle, bChannel);
  return(hConvValue);
}

/**
  * @brief  It sets the specified sampling time for the specified ADC channel
  *         on ADC1. It must be called once for each channel utilized by user
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  ADConv_struct struct containing ADC channel and sampling time
  * @retval none
  */
void PWMC_ADC_SetSamplingTime(PWMC_Handle_t *pHandle, ADConv_t ADConv_struct)
{
  pHandle->pFctSetSamplingTime(pHandle, ADConv_struct);
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle)
{
  return pHandle->pFctIsOverCurrentOccurred(pHandle);
}

/**
  * @brief  It is used to set the overcurrent threshold through the DAC reference
  *         voltage.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
  *         Ex. 0 = 0V 65536 = VDD_DAC.
  * @retval none
  */
void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle,uint16_t hDACVref)
{
  if (pHandle->pFctOCPSetReferenceVoltage)
  {
    pHandle->pFctOCPSetReferenceVoltage(pHandle, hDACVref);
  }
}

/**
  * @brief  It is used to retrieve the satus of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
bool PWMC_GetTurnOnLowSidesAction(PWMC_Handle_t *pHandle)
{
  return pHandle->bTurnOnLowSidesAction;
}

/**
  * @brief  It is used to set the RL Detection mode.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none
  */
void PWMC_RLDetectionModeEnable(PWMC_Handle_t *pHandle)
{
  if (pHandle->pFctRLDetectionModeEnable)
  {
    pHandle->pFctRLDetectionModeEnable(pHandle);
  }
}

/**
  * @brief  It is used to disable the RL Detection mode and set the standard PWM.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none
  */
void PWMC_RLDetectionModeDisable(PWMC_Handle_t *pHandle)
{
  if (pHandle->pFctRLDetectionModeDisable)
  {
    pHandle->pFctRLDetectionModeDisable(pHandle);
  }
}

/**
  * @brief  It is used to set the PWM dutycycle in the RL Detection mode.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  hDuty to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t PWMC_RLDetectionModeSetDuty(PWMC_Handle_t *pHandle, uint16_t hDuty)
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if (pHandle->pFctRLDetectionModeSetDuty)
  {
    hRetVal = pHandle->pFctRLDetectionModeSetDuty(pHandle, hDuty);
  }
  return hRetVal;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to get phases current.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
                                            PWMC_Handle_t* pHandle )
{
  pHandle->pFctGetPhaseCurrents = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to switch off PWM
 *        generation.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t* pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to switch on PWM
 *        generation.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                      PWMC_Handle_t* pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
                                              PWMC_Handle_t* pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to turn on low sides.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to set a new sampling
 * time on a ADC channel.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSetSamplingTimeCallBack( PWMC_SetSampTime_Cb_t pCallBack,
                                           PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetSamplingTime = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 1.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect1CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect1 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 2.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect2CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect2 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 3.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect3CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect3 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 4.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect4CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect4 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 5.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect5CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect5 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to ADC sampling point for
 * sector 6.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterSampPointSect6CallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
                                          PWMC_Handle_t* pHandle )
{
  pHandle->pFctSetADCSampPointSect6 = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to execute a regular ADC
 * conversion
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRegularConvExecCallBack( PWMC_RegConvExec_Cb_t pCallBack,
                                           PWMC_Handle_t* pHandle )
{
  pHandle->pFctRegularConvExec = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
                                                 PWMC_Handle_t* pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to set the reference
 * voltage for the overcurrent protection
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
                                            PWMC_Handle_t* pHandle )
{
  pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to set the R/L detection
 * mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
                                                 PWMC_Handle_t* pHandle )
{
  pHandle->pFctRLDetectionModeEnable = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to disable the R/L detection
 * mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
                                                  PWMC_Handle_t* pHandle )
{
  pHandle->pFctRLDetectionModeDisable = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to set the duty cycle
 * for the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
                                                  PWMC_Handle_t* pHandle )
{
  pHandle->pFctRLDetectionModeSetDuty = pCallBack;
}

/**
 * @brief Sets the Callback which PWMC shall invoke to call PWMC instance IRQ handler
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t* pHandle )
{
  pHandle->pFctIrqHandler = pCallBack;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
