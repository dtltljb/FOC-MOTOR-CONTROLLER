/**
  ******************************************************************************
  * @file    ntc_temperature_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control SDK.
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
#include "ntc_temperature_sensor.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup TemperatureSensor NTC Temperature Sensor
  * @brief Allows to read the temperature of the heat sink
  *
  * This component implements both a virtual and a real temperature sensor,
  * depending on the sensor availability. 
  * 
  * Access to the MCU peripherals needed to acquire the temperature (GPIO and ADC
  * used for regular conversion) is managed by the PWM component used in the Motor
  * Control subsystem. As a consequence, this NTC temperature sensor implementation
  * is hardware-independent.
  * 
  * If a real temperature sensor is available (Sensor Type = #REAL_SENSOR),
  * this component can handle NTC sensors or, more generally, analog temperature sensors
  * which output is related to the temperature by the following formula:
  *
  * @f[
  *               V_{out} = V_0 + \frac{dV}{dT} \cdot ( T - T_0)
  * @f]
  * 
  * In case a real temperature sensor is not available (Sensor Type = #VIRTUAL_SENSOR),
  * This component will always returns a constant, programmable, temperature.
  * 
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Updated internal fault status
  */
uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle)
{
    uint16_t hFault;

    if(pHandle->hAvTemp_d > pHandle->hOverTempThreshold)
    {
        hFault = MC_OVER_TEMP;
    }
    else if (pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold)
    {
        hFault = MC_NO_ERROR;
    }
    else
    {
        hFault = pHandle->hFaultState;
    }
    return hFault;
}

/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes temperature sensing conversions
 *
 *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
 *
 *  @p pPWMnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
void NTC_Init(NTC_Handle_t *pHandle, PWMC_Handle_t * pPWMnCurrentSensor)
{

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        pHandle->pPWMnCurrentSensor = pPWMnCurrentSensor;

        NTC_Clear(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
        pHandle->hFaultState = MC_NO_ERROR;
        pHandle->hAvTemp_d = pHandle->hExpectedTemp_d;
    }

}

/**
 * @brief Initializes internal average temperature computed value
 *
 *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
 */
void NTC_Clear(NTC_Handle_t *pHandle)
{
    pHandle->hAvTemp_d = 0u;
}

/**
  * @brief Performs the temperature sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Error reported in case of an over temperature detection
  */
uint16_t NTC_CalcAvTemp(NTC_Handle_t *pHandle)
{
    uint32_t wtemp;
    uint16_t hAux;

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        hAux = PWMC_ExecRegularConv((PWMC_Handle_t *)pHandle->pPWMnCurrentSensor, pHandle->bTsensADChannel);

        if(hAux != 0xFFFFu)
        {
            wtemp =  (uint32_t)(pHandle->hLowPassFilterBW)-1u;
            wtemp *= (uint32_t) (pHandle->hAvTemp_d);
            wtemp += hAux;
            wtemp /= (uint32_t)(pHandle->hLowPassFilterBW);

            pHandle->hAvTemp_d = (uint16_t) wtemp;
        }

        pHandle->hFaultState = NTC_SetFaultState(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
        pHandle->hFaultState = MC_NO_ERROR;
    }

    return(pHandle->hFaultState);
}

/**
  * @brief  Returns latest averaged temperature measured expressed in u16Celsius
  *
  * @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @r AverageTemperature : Current averaged temperature measured (in u16Celsius)
  */
uint16_t NTC_GetAvTemp_d(NTC_Handle_t *pHandle)
{
    return(pHandle->hAvTemp_d);
}

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  *
  * @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @r AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
int16_t NTC_GetAvTemp_C(NTC_Handle_t *pHandle)
{
    int32_t wTemp;

    if (pHandle->bSensorType == REAL_SENSOR)
    {
        wTemp = (int32_t)(pHandle->hAvTemp_d);
        wTemp -= (int32_t)(pHandle->wV0);
        wTemp *= pHandle->hSensitivity;
        wTemp = wTemp/65536 + (int32_t)(pHandle->hT0);
    }
    else
    {
        wTemp = pHandle->hExpectedTemp_C;
    }
    return((int16_t)wTemp);
}

/**
  * @brief  Returns Temperature measurement fault status
  *
  * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
  * MC_NO_ERROR if it is inside authorized range.
  *
  * @p pHandle: Pointer on Handle structure of TemperatureSensor component.
  *
  *  @r Fault status : read internal fault state
  */
uint16_t NTC_CheckTemp(NTC_Handle_t *pHandle)
{
    return(pHandle->hFaultState);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
