/**
  ******************************************************************************
  * @file    motorcontrol.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem initialization functions.
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
#include "main.h"
#include "mc_tuning.h"
#include "mc_interface.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "UITask.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCAPI
  * @{
  */

#define FIRMWARE_VERS "ST MC SDK\0Ver.5.0.1"
const char s_fwVer[32] = FIRMWARE_VERS;

MCI_Handle_t* pMCI[MC_NUM];
MCT_Handle_t* pMCT[MC_NUM];
uint32_t wConfig[MC_NUM] = {UI_CONFIG_M1,UI_CONFIG_M2};

/**
 * @brief Initializes and configures the Motor Control Subsystem
 *
 *  This function initializes and configures all the structures and components needed
 * for the Motor Control subsystem required by the Application. It expects that
 * all the peripherals needed for Motor Control purposes are already configured but
 * that their interrupts are not enabled yet. 
 *
 * CubeMX calls this function after all peripherals initializations and 
 * before the NVIC is configured
 */
void MX_MotorControl_Init() {
  /* Reconfigure the SysTick interrupt to fire every 2 ms. */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);
  
  /* Initialize the Motor Control Subsystem */
  MCboot(pMCI,pMCT);
  mc_lock_pins();
  
  /* Initialize the MC User Interface */
  UI_TaskInit(UI_INIT_CFG,wConfig,MC_NUM,pMCI,pMCT,s_fwVer);
}

/* Use this function with caution */
void MC_HandleStartStopButton(void)
{
    if (MCI_GetSTMState(pMCI[0]) == IDLE)
    {
      /* Ramp parameters should be tuned for the actual motor */
     // MCI_ExecSpeedRamp(pMCI[0], 2000, 2000);
      MCI_StartMotor(pMCI[0]);
    }
    else
    {
      MCI_StopMotor(pMCI[0]);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
