/**
  ******************************************************************************
  * @file    USARTParams.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains USART constant parameters definition
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_PARAMS_H
#define __USART_PARAMS_H
#include "parameters_conversion.h"

/* For backward compatibility */
#if (!defined(USART_SPEED))
#define USART_SPEED 115200
#endif

#define MC_PROTOCOL_REG_UNDEFINED 1
#if (SERIAL_COM_CHANNEL2 == MC_PROTOCOL_REG_UNDEFINED)
  #define SERIAL_COM_CH_NBRS 1
#else
  #define SERIAL_COM_CH_NBRS 2
#endif
#undef MC_PROTOCOL_REG_UNDEFINED

#endif /* __USART_PARAMS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
