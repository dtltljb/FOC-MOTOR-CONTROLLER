/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PMSM_MOTOR_PARAMETERS_H
#define __PMSM_MOTOR_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/
#define		MOTOR_BJ_HST		1			/* = 1 use BJ_HST_MOTOR , =0 USE SHENZHENG_MOTOR*/
/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#if	MOTOR_BJ_HST == 1
	#define POLE_PAIR_NUM          4 /* Number of motor pole pairs *///惠斯通26W 小电机
	#define RS                     2.90 /* Stator resistance , ohm*/
	#define LS                     0.000270 /* Stator inductance, H For I-PMSM it is equal to Lq */

	#define MOTOR_VOLTAGE_CONSTANT  5.0 /*!< Volts RMS ph-ph /kRPM */

//	#define HALL_PHASE_SHIFT_N        	345						//N rmp	280		惠斯通26w小电机
//	#define HALL_PHASE_SHIFT_0        	305 					//0 rmp
//	#define HALL_PHASE_SHIFT_P        	220 					//P rmp 320 	
	#define HALL_PHASE_SHIFT_N        	325						//N rmp	280		惠斯通26w小电机
	#define HALL_PHASE_SHIFT_0        	305 					//0 rmp
	#define HALL_PHASE_SHIFT_P        	290 					//P rmp 320 	
#else

//	#define POLE_PAIR_NUM          4 /* Number of motor pole pairs */
//	#define RS                     0.170 /* Stator resistance , ohm*/
//	#define LS                     0.000250 /* Stator inductance, H	For I-PMSM it is equal to Lq */

//	#define MOTOR_VOLTAGE_CONSTANT  1.5 /*!< Volts RMS ph-ph /kRPM 深圳电机 */

//	#define HALL_PHASE_SHIFT_N        	216						//N rmp		深圳50w电机
//	#define HALL_PHASE_SHIFT_0        	246 					//0 rmp
//	#define HALL_PHASE_SHIFT_P        	266 					//P rmp

	#define POLE_PAIR_NUM          4 /* Number of motor pole pairs */
	#define RS                     2.50 /* Stator resistance , ohm*/
	#define LS                     0.00080 /* Stator inductance, H	For I-PMSM it is equal to Lq */

	#define MOTOR_VOLTAGE_CONSTANT  1.5 /*!< Volts RMS ph-ph /kRPM 深圳电机 */

	#define HALL_PHASE_SHIFT_N        	216						//N rmp		深圳50w电机
	#define HALL_PHASE_SHIFT_0        	246 					//0 rmp
	#define HALL_PHASE_SHIFT_P        	266 					//P rmp

#endif

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         27405
#define MOTOR_MAX_SPEED_RPM     5000 /*!< Maximum rated speed  */

#define ID_DEMAG                -27405 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */



//#define HALL_PHASE_SHIFT        305 
																						/*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define ENCODER_PPR             400  /*!< Number of pulses per
                                            revolution */

#endif /*__PMSM_MOTOR_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
