/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Resistor Divider Bus Voltage Sensor component of the Motor
  *          Control SDK:
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
#include "r_divider_bus_voltage_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion (ADC channel, conversion time,
  *         GPIO port and pin). It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
void RVBS_Init(RDivider_Handle_t *pHandle, PWMC_Handle_t* PWMnCurrentSensor)
{
  ADConv_t ADConv_struct;

  pHandle->PWMnCurrentSensor = PWMnCurrentSensor;

  /* Configure AD chaneel sampling time */
  ADConv_struct.Channel = pHandle->VbusADChannel;
  ADConv_struct.SamplTime = pHandle->VbusSamplingTime;
  PWMC_ADC_SetSamplingTime(PWMnCurrentSensor, ADConv_struct);
  RVBS_Clear(pHandle);
}


/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
void RVBS_Clear(RDivider_Handle_t *pHandle)
{
  uint16_t aux;
  uint16_t index;

  aux = (pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold)/2u;
  for (index = 0u; index < pHandle->LowPassFilterBW; index++)
  {
    pHandle->aBuffer[index] = aux;
  }
  pHandle->_Super.LatestConv = aux;
  pHandle->_Super.AvBusVoltage_d = aux;
  pHandle->index = 0;
}

static uint16_t RVBS_ConvertVbusFiltrered(RDivider_Handle_t *pHandle)
{
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max,min;
  uint32_t tot = 0u;

  for (vindex = 0; vindex < pHandle->LowPassFilterBW; )
  {
    hAux = PWMC_ExecRegularConv(pHandle->PWMnCurrentSensor,
                              pHandle->VbusADChannel);

    if(hAux != 0xFFFFu)
    {
      if (vindex == 0)
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if (hAux < min)
        {
          min = hAux;
        }
        if (hAux > max)
        {
          max = hAux;
        }
      }
      vindex++;

      tot += hAux;
    }
  }

  tot -= max;
  tot -= min;
  return (uint16_t)(tot / (pHandle->LowPassFilterBW-2u));
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CalcAvVbusFilt(RDivider_Handle_t *pHandle)
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RVBS_ConvertVbusFiltrered(pHandle);

  if (hAux != 0xFFFF)
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for (i = 0; i < pHandle->LowPassFilterBW; i++)
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
    pHandle->_Super.LatestConv = hAux;

    if (pHandle->index < pHandle->LowPassFilterBW-1)
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);

  return(pHandle->_Super.FaultState);
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CalcAvVbus(RDivider_Handle_t *pHandle)
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = PWMC_ExecRegularConv(pHandle->PWMnCurrentSensor,
                              pHandle->VbusADChannel);

  if (hAux != 0xFFFF)
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for (i = 0; i < pHandle->LowPassFilterBW; i++)
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
    pHandle->_Super.LatestConv = hAux;

    if (pHandle->index < pHandle->LowPassFilterBW-1)
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);

  return(pHandle->_Super.FaultState);
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CheckFaultState(RDivider_Handle_t *pHandle)
{
  uint16_t fault;

  if(pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold)
    {
      fault = MC_OVER_VOLT;
    }
    else if (pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold)
    {
      fault = MC_UNDER_VOLT;
    }
    else
    {
      fault = MC_NO_ERROR;
    }
  return fault;
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

