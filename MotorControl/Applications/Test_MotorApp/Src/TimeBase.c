/**
  ******************************************************************************
  * @file    Timebase.c 
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file implements the time base utilized for running MC tasks
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

/* Includes ------------------------------------------------------------------*/
#include "Timebase.h"
#if defined(PFC_ENABLED)
  #include "pid_regulator.h"
#endif
#include "mc_tuning.h"
#include "mc_interface.h"
#if defined(PFC_ENABLED)
  #include "PFCApplication.h"
#endif
#include "mc_tasks.h"
#include "UITask.h"
#include "parameters_conversion.h"

extern uint8_t bMCBootCompleted;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t  bUDTBTaskCounter;
static volatile uint16_t  hKey_debounce_500us = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Use this function to know whether the user time base is elapsed
  * has elapsed 
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TB_UserTimebaseHasElapsed(void)
{
  bool retVal = false;
  if (bUDTBTaskCounter == 0u)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the user time base 
  *         time
  * @param  SysTickCount number of System ticks to be counted
  * @retval void
  */
void TB_SetUserTimebaseTime(uint16_t SysTickCount)
{
  bUDTBTaskCounter = SysTickCount;
}

/**
  * @brief  It is the task scheduler.
  * @param  none
  * @retval none
  */
void TB_Scheduler(void)
{
  if ( 1 == bMCBootCompleted ) {
    MC_Scheduler(); // Moved here to let SafetyTask to overcome MF action */

    TSK_SafetyTask();		/*	check temp\voltage\current,and PWMC_ExecRegularConv()	*/
  
#ifdef PFC_ENABLED
    {
      PFC_Scheduler();
    }
#endif
  
    UI_Scheduler();			/*	update	uart communiction flag */

    if(bUDTBTaskCounter > 0u)
    {
      bUDTBTaskCounter--;
    }

    if (hKey_debounce_500us != 0)
    {
      hKey_debounce_500us --;
    }
  }
}

void TB_Set_DebounceDelay_500us(uint8_t hDelay)
{
  hKey_debounce_500us = hDelay;
}  

bool TB_DebounceDelay_IsElapsed(void)
{
 if (hKey_debounce_500us == 0)
 {
   return (true);
 }
 else 
 {
   return (false);
 }
} 
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
