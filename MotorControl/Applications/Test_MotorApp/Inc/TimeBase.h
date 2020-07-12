/**
  ******************************************************************************
  * @file    Timebase.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file defines booting function of MC library, single motor      
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
#ifndef __TIMEBASE_H
#define __TIMEBASE_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup Timebase
  * @{
  */

/** @defgroup Timebase_exported_functions Timebase exported functions
  * @{
  */
    
/**
  * @brief  Use this function to know whether the user time base is elapsed
  * has elapsed 
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TB_UserTimebaseHasElapsed(void);

/**
  * @brief  It set a counter intended to be used for counting the user time base 
  *         time
  * @param  SysTickCount number of System ticks to be counted
  * @retval void
  */
void TB_SetUserTimebaseTime(uint16_t SysTickCount);

/**
  * @brief  It is the task scheduler.
  * @param  none
  * @retval none
  */
void TB_Scheduler(void);

void TB_Set_DebounceDelay_500us(uint8_t hDelay);
bool TB_DebounceDelay_IsElapsed(void);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCBOOTSINGLEMOTOR_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
