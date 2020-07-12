/**
  ******************************************************************************
  * @file    Control stage parameters.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *	         library v3.0
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
#ifndef __DEFINITIONS_H
#define __DEFINITIONS_H

#define	DEGREES_120 0u
#define	DEGREES_60 1u

#define TURN_ON  1
#define TURN_OFF 0
#define INRUSH_ACTIVE   1
#define INRUSH_INACTIVE 0

#define TURN_OFF_PWM      0
#define TURN_ON_R_BRAKE   1  
#define TURN_ON_LOW_SIDES 2

//#define STM32VALUE            1
//#define STM32PERFORMANCEMD    2
//#define STM32PERFORMANCEHD    3
//#define STM32PERFORMANCELD    4				
#define VFQFPN36  1
#define VFQFPN48  2
#define LQFP48    3
#define LQFP64    4
#define LQFP100   5
#define LQFP144   6
#define WLCSP64   7
#define LFBGA100  8
#define	LFBGA144  9
#define BGA100    10
#define BGA64     11
#define TFBGA64   12

#define HALL_TIM2 2
#define HALL_TIM3 3
#define HALL_TIM4 4
#define HALL_TIM5 5

#define ENC_TIM2 2
#define ENC_TIM3 3
#define ENC_TIM4 4
#define ENC_TIM5 5


#endif /*__DEFINITIONS_H*/
/**************** (c) 2007  STMicroelectronics ********************************/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
