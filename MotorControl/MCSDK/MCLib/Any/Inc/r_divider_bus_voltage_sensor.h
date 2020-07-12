/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Resistor Divider Bus Voltage Sensor component of the Motor Control SDK.
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
#ifndef __RDIVIDER_BUSVOLTAGESENSOR_H
#define __RDIVIDER_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "bus_voltage_sensor.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @addtogroup RDividerBusVoltageSensor
  * @{
  */

#define BUS_BUFF_MAX                10   /* Maximum buffer size to compute average value.*/

/**
  * @brief  Rdivider class parameters definition
  */
typedef struct
{
  BusVoltageSensor_Handle_t _Super;

  uint8_t        VbusADChannel;         /*!< ADC channel used for conversion of
                                             bus voltage. It must be equal to
                                             ADC_CHANNEL_xx x= 0, ..., 15*/
  GPIO_TypeDef*  VbusPort;              /*!< GPIO port used by bVbusADChannel.
                                             It must be equal to GPIOx x= A, B, ...*/
  uint16_t       VbusPin;               /*!< GPIO pin used by bVbusChannel. It must
                                             be equal to GPIO_Pin_x x= 0, 1, ...*/
  uint8_t        VbusSamplingTime;      /*!< Sampling time used for bVbusChannel AD
                                             conversion. It must be equal to
                                             ADC_SampleTime_xCycles5 x= 1, 7, ...*/
  uint16_t       LowPassFilterBW;       /*!< Use this number to configure the Vbus
                                             first order software filter bandwidth.
                                             hLowPassFilterBW = VBS_CalcBusReading
                                             call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t       OverVoltageThreshold;  /*!< It represents the over voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hOverVoltageThreshold (digital value) =
                                             Over Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  uint16_t       UnderVoltageThreshold; /*!< It represents the under voltage protection
                                             intervention threshold. To be expressed
                                             in digital value through formula:
                                             hUnderVoltageThreshold (digital value)=
                                             Under Voltage Threshold (V) * 65536
                                             / hConversionFactor */
  PWMC_Handle_t* PWMnCurrentSensor;     /*!< PWMC_Handle_t to be used for regular
                                             conversions*/
  uint16_t       aBuffer[BUS_BUFF_MAX]; /*!< Buffer used to compute average value.*/
  uint8_t        elem;                  /*!< Number of stored elements in the average buffer.*/
  uint8_t        index;                 /*!< Index of last stored element in the average buffer.*/

}RDivider_Handle_t;

/* Exported functions ------------------------------------------------------- */
void RVBS_Init(RDivider_Handle_t *pHandle, PWMC_Handle_t* PWMnCurrentSensor);
void RVBS_Clear(RDivider_Handle_t *pHandle);
uint16_t RVBS_CalcAvVbusFilt(RDivider_Handle_t *pHandle);
uint16_t RVBS_CalcAvVbus(RDivider_Handle_t *pHandle);
uint16_t RVBS_CheckFaultState(RDivider_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __RDividerBusVoltageSensor_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

