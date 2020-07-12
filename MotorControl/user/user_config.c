/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "mc_type.h"
#include "std_hal_missing_define.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"

//  #define MAX_TWAIT 0                 /* Dummy value for single drive */
//  #define FREQ_RATIO 1                /* Dummy value for single drive */
//  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
#include "pid_regulator.h"
#include "speed_torq_ctrl.h"
#include "revup_ctrl.h"
#include "ntc_temperature_sensor.h"
#include "digital_output.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "user_interface.h"
#include "lcd_manager_ui.h"
#include "lcd_vintage_ui.h"
#include "dac_common_ui.h"
#include "dac_ui.h"
#include "motor_control_protocol.h"
#include "usart_frame_communication_protocol.h"

#include "uart1_frame_communication_protocol.h"

#include "UIIRQHandlerClass.h"

#include "pwm_curr_fdbk.h"
#include "r3_1_f30x_pwm_curr_fdbk.h"

#include "hall_speed_pos_fdbk.h"
/* Temporary hack to be able to compile */
#include "MCIRQHandlerClass.h"
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"

//#define OFFCALIBRWAIT_MS     0
//#define OFFCALIBRWAIT_MS2    0     

#include "pqd_motor_power_measurement.h"
#include "user_config.h"

U1FCP_Handle_t pUSART1 =
{
    ._Super.RxTimeout = 0, 

    .USARTx              = USART1,                
    .UIIRQn              = UI_IRQ_USART,         
};


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

