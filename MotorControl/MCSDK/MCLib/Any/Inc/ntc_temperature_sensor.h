/**
  ******************************************************************************
  * @file    ntc_temperature_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Temperature Sensor component of the Motor Control SDK.
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
#ifndef __TEMPERATURESENSOR_H
#define __TEMPERATURESENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup TemperatureSensor
  * @{
  */


/**
  * @brief NTC_Handle_t structure used for temperature monitoring
  *
  */
typedef struct
{
  SensorType_t  bSensorType;   /**< Type of instanced temperature.
                                    This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  uint16_t hAvTemp_d;          /**< It contains latest available average Vbus.
                                    This parameter is expressed in u16Celsius */

  uint16_t hExpectedTemp_d;    /**< Default set when no sensor available (ie virtual sensor) */

  uint16_t hExpectedTemp_C;    /**< Default value when no sensor available (ie virtual sensor).
                                    This parameter is expressed in Celsius */

  uint16_t hFaultState;        /**< Contains latest Fault code.
                                    This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

  uint8_t bTsensADChannel;     /**< ADC channel used for conversion of temperature sensor output.
                                    This parameter must be equal to ADC_CHANNEL_xx x= 0, ...,15 */

  GPIO_TypeDef* hTsensPort;    /**< GPIO port used by bVbusADChannel.
                                    This parameter must be equal to GPIOx x= A, B, ... */

  uint16_t hTsensPin;          /**< GPIO pin used by bVbusChannel.
                                    This parameter must be equal to GPIO_Pin_x x= 0, 1, ...*/

  uint8_t bTsensSamplingTime;  /**< Sampling time used for bVbusChannel AD conversion.
                                    This parameter must be equal to ADC_SampleTime_xCycles5 x= 1, 7, ...*/

  uint16_t hLowPassFilterBW;   /**< used to configure the first order software filter bandwidth.
                                    hLowPassFilterBW = NTC_CalcBusReading
                                    call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t hOverTempThreshold; /**< Represents the over voltage protection intervention threshold.
                                    This parameter is expressed in u16Celsius through formula:
                                    hOverTempThreshold = 
                                    (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C] - T0[°C]))* 65536 / MCU supply voltage */
  uint16_t hOverTempDeactThreshold; /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         hOverTempDeactThreshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 / MCU supply voltage*/
  int16_t hSensitivity;        /**< NTC sensitivity
                                    This parameter is equal to MCU supply voltage [V] / dV/dT [V/°C] */
  uint32_t wV0;                /**< V0 voltage constant value used to convert the temperature into Volts.    
                                    This parameter is equal V0*65536/MCU supply
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
  uint16_t hT0;                /**< T0 temperature constant value used to convert the temperature into Volts
                                    Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */

  PWMC_Handle_t * pPWMnCurrentSensor;    /**< CPWMC object to be used for regular conversions */

} NTC_Handle_t;

/* Initialize temperature sensing parameters */
void NTC_Init(NTC_Handle_t *pHandle, PWMC_Handle_t *pPWMnCurrentSensor);

/* Clear static average temperature value */
void NTC_Clear(NTC_Handle_t *pHandle);

/* Temperature sensing computation */
uint16_t NTC_CalcAvTemp(NTC_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in u16Celsius */
uint16_t NTC_GetAvTemp_d(NTC_Handle_t *pHandle);

/* Get averaged temperature measurement expressed in Celsius degrees */
int16_t NTC_GetAvTemp_C(NTC_Handle_t *pHandle);


/* Get the temperature measurement fault status */
uint16_t NTC_CheckTemp(NTC_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __TEMPERATURESENSOR_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
