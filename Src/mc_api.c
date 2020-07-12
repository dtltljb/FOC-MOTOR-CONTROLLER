/**
  ******************************************************************************
  * @file    mc_api.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the high level interface of the Motor Control SDK.
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
  
#include "mc_interface.h"
#include "mc_api.h"
#include "mc_extended_api.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCIAPI Motor Control API
  *
  * @brief High level Programming Interface of the Motor Control SDK
  *
  *  This interface allows for performing basic operations on the motor(s) driven by an
  * Motor Control SDK based application. With it, motors can be started and stopped, speed or
  * torque ramps can be programmed and executed and information on the state of the motors can
  * be retrieved, among others.
  *
  *  This interface consists in functions that target a specific motor, indicated in their name. 
  * These functions aims at being the main interface used by an Application to control motors.
  *
  *  The current Motor Control API can cope with up to 2 motors.  
  * @{
  */

extern MCI_Handle_t * pMCI[MC_NUM];

/**
  * @brief  Initiates the start-up procedure for Motor 1
  * 
  * If the state machine of Motor 1 is in #IDLE state, the command is immediately
  * executed. Otherwise the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *
  * One of the following commands must be executed before calling MC_StartMotor1():
  *
  * - MC_ProgramSpeedRampMotor1()
  * - MC_ProgramTorqueRampMotor1()
  * - MC_SetCurrentReferenceMotor1()
  *
  * Failing to do so results in an unpredictable behaviour.
  *
  * @note The MC_StartMotor1() command only triggers the start-up procedure: 
  * It moves Motor 1's state machine from the #IDLE to the #IDLE_START state and then 
  * returns. It is not blocking the application until the motor is indeed running. 
  * To know if it is running, the application can query Motor 1's state machine and 
  * check if it has reached the #RUN state. See MCI_GetSTMStateMotor1() for more details.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
bool MC_StartMotor1(void)
{
	return MCI_StartMotor( pMCI[M1] );
}

/**
  * @brief  Initiates the stop procedure for Motor 1.
  *
  *  If the state machine is in #RUN or #START states the command is immediately
  * executed. Otherwise, the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *  
  * @note The MCI_StopMotor1() command only triggers the stop motor procedure 
  * moving Motor 1's state machine to #ANY_STOP and then returns. It is not 
  * blocking the application until the motor is indeed stopped. To know if it has
  * stopped, the application can query Motor 1's state machine ans check if the 
  * #IDLE state has been reached back.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
void MC_StopMotor1(void)
{
	MCI_StopMotor( pMCI[M1] );
}

/**
  * @brief Programs a speed ramp for Motor 1 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p hFinalSpeed 
  * target speed in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramSpeedRampMotor1() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter 
  * differs from that of the current speed, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *                     Expressed in tenths of HZ.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp( pMCI[M1], hFinalSpeed, hDurationms );
}

/**
  * @brief Programs a torque ramp for Motor 1 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p hFinalTorque 
  * target torque reference in the given @p hDurationms time. 
  *
  *  Invoking the MC_ProgramTorqueRampMotor1() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * ramp is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the 
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using 
  * sensorless motor control techniques. If the sign of the hFinalTorque parameter 
  * differs from that of the current torque, the ramp will not complete and a Speed 
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to 
  * reach 0 rpm.    
  * 
  * @param  hFinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in digit.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp( pMCI[M1], hFinalTorque, hDurationms );
}

/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components. 
  *
  *  Invoking the MC_SetCurrentReferenceMotor1() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #START_RUN or #RUN states. Otherwise, the 
  * command is buffered and will be executed when the state machine reaches any of 
  * the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is 
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed 
  *         in the Curr_Components format.
  */
void MC_SetCurrentReferenceMotor1( Curr_Components Iqdref )
{
	MCI_SetCurrentReferences( pMCI[M1], Iqdref );
}

/**
  * @brief  Returns the status of the last buffered command for Motor 1.
  * The status can be one of the following values:
  * - #MCI_BUFFER_EMPTY: no buffered command is currently programmed.
  * - #MCI_COMMAND_NOT_ALREADY_EXECUTED: A command has been buffered but the conditions for its
  *   execution have not occurred yet. The command is still in the buffer, pending execution.
  * - #MCI_COMMAND_EXECUTED_SUCCESFULLY: the last buffered command has been executed successfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  * - #MCI_COMMAND_EXECUTED_UNSUCCESFULLY: the buffered command has been executed unsuccessfully. 
  *   In this case calling this function reset the command state to #BC_BUFFER_EMPTY.
  */
MCI_CommandState_t  MC_GetCommandStateMotor1( void)
{
	return MCI_IsCommandAcknowledged( pMCI[M1] );
}

/** 
 * @brief Stops the execution of the on-going speed ramp for Motor 1, if any.
 *
 *  If a speed ramp is currently being executed, it is immediately stopped, the rotation
 * speed of Motor 1 is maintained to its current value and true is returned. If no speed 
 * ramp is on-going, nothing is done and false is returned.
 */
bool MC_StopSpeedRampMotor1(void)
{
	return MCI_StopSpeedRamp( pMCI[M1] );
}

/**
 * @brief Returns true if the last ramp submited for Motor 1 has completed, false otherwise 
 */
bool MC_HasRampCompletedMotor1(void)
{
	return MCI_RampCompleted( pMCI[M1] );
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 1, expressed in dHz (tenth of Hertz) 
 */
int16_t MC_GetMecSpeedReferenceMotor1(void)
{
	return MCI_GetMecSpeedRef01Hz( pMCI[M1] );
}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 1, expressed in dHz (tenth of Hertz) 
 */
int16_t MC_GetMecSpeedAverageMotor1(void)
{
	return MCI_GetAvrgMecSpeed01Hz( pMCI[M1] );
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 1 if this ramp was a speed ramp, 0 otherwise.
 */
int16_t MC_GetLastRampFinalSpeedMotor1(void)
{
	return MCI_GetLastRampFinalSpeed( pMCI[M1] );
}

/**
 * @brief Returns the Control Mode used for Motor 1 (either Speed or Torque)
 */
STC_Modality_t MC_GetControlModeMotor1(void)
{
	return MCI_GetControlMode( pMCI[M1] );
}

/**
 * @brief Returns the rotation direction imposed by the last command on Motor 1 
 *
 * The last command is either MC_ProgramSpeedRampMotor1(), MC_ProgramTorqueRampMotor1() or
 * MC_SetCurrentReferenceMotor1(). 
 *
 * The function returns -1 if the sign of the final speed, the final torque or the Iq current 
 * reference component of the last command is negative. Otherwise, 1 is returned. 
 *   
 * @note if no such command has ever been submitted, 1 is returned as well.  
 */
int16_t MC_GetImposedDirectionMotor1(void)
{
	return MCI_GetImposedMotorDirection( pMCI[M1] );
}

/** 
 * @brief Returns true if the speed sensor used for Motor 1 is reliable, false otherwise 
 */
bool MC_GetSpeedSensorReliabilityMotor1(void)
{
	return MCI_GetSpdSensorReliability( pMCI[M1] );
}

/** 
 * @brief returns the amplitude of the phase current injected in Motor 1
 *
 * The returned amplitude (0-to-peak) is expressed in s16A unit. To convert it to amperes, use the following formula:
 *
 * @f[
 * I_{Amps} = \frac{ I_{s16A} \times V_{dd}}{ 65536 \times R_{shunt} \times A_{op} }
 * @f]
 *
 */
int16_t MC_GetPhaseCurrentAmplitudeMotor1(void)
{
	return MCI_GetPhaseCurrentAmplitude( pMCI[M1] );
}

/**
 * @brief returns the amplitude of the phase voltage applied to Motor 1 
 *
 * The returned amplitude (0-to-peak) is expressed in s16V unit. To convert it to volts, use the following formula:
 *
 * @f[
 * U_{Volts} = \frac{ U_{s16V} \times V_{bus}}{ \sqrt{3} \times 32768  }
 * @f]
 *
 */
int16_t MC_GetPhaseVoltageAmplitudeMotor1(void)
{
	return MCI_GetPhaseVoltageAmplitude( pMCI[M1] );
}

/**
 * @brief returns Ia and Ib current values for Motor 1 in Curr_Components format
 */
Curr_Components MC_GetIabMotor1(void)
{
	return MCI_GetIab( pMCI[M1] );
}

/**
 * @brief returns Ialpha and Ibeta current values for Motor 1 in Curr_Components format
 */
Curr_Components MC_GetIalphabetaMotor1(void)
{
	return MCI_GetIalphabeta( pMCI[M1] );
}

/**
 * @brief returns Iq and Id current values for Motor 1 in Curr_Components format 
 */
Curr_Components MC_GetIqdMotor1(void)
{
	return MCI_GetIqd( pMCI[M1] );
}

/**
 * @brief returns Iq and Id reference current values for Motor 1 in Curr_Components format
 */
Curr_Components MC_GetIqdrefMotor1(void)
{
	return MCI_GetIqdref( pMCI[M1] );
}

/**
 * @brief returns Vq and Vd voltage values for Motor 1 in Volt_Components format
 */
Volt_Components MC_GetVqdMotor1(void)
{
	return MCI_GetVqd( pMCI[M1] );
}

/**
 * @brief returns Valpha and Vbeta voltage values for Motor 1 in Volt_Components format 
 */
Volt_Components MC_GetValphabetaMotor1(void)
{
	return MCI_GetValphabeta( pMCI[M1] );
}

/**
 * @brief returns the electrical angle of the rotor of Motor 1, in DDP format 
 */
int16_t MC_GetElAngledppMotor1(void)
{
	return MCI_GetElAngledpp( pMCI[M1] );
}

/**
 * @brief returns the electrical torque reference for Motor 1 
 */
int16_t MC_GetTerefMotor1(void)
{
	return MCI_GetTeref( pMCI[M1] );
}

/**
 * @brief Sets Id reference value for Motor 2 
 *
 * When the current reference drive is internal (the FOCVars_t.bDriveInput field is set to #INTERNAL), 
 * Idref is normally managed by the FOC_CalcCurrRef() function. Neverthless, this function allows to 
 * force a change in Idref value. 
 * 
 * Calling this function has no effect when either flux weakening region is entered or MTPA is enabled.
 */
void MC_SetIdrefMotor1( int16_t hNewIdref )
{
	MCI_SetIdref( pMCI[M1], hNewIdref );
}

/**
 * @brief re-initializes Iq and Id references to their default values for Motor 1
 *
 * The default values for the Iq and Id references are comming from the Speed 
 * or the Torque controller depending on the control mode. 
 *
 * @see   SpeednTorqCtrl for more details.
 */
void MC_Clear_IqdrefMotor1(void)
{
	MCI_Clear_Iqdref( pMCI[M1] );
}

/**
 * @brief Acknowledge a Motor Control fault that occured on Motor 1
 *
 *  This function informs Motor 1's state machine that the Application has taken
 * the error condition that occured into account. If no error condition exists when
 * the function is called, nothing is done and false is returned. Otherwise, true is
 * returned. 
 */
bool MC_AcknowledgeFaultMotor1( void )
{
	return MCI_FaultAcknowledged( pMCI[M1] );
}

/**
 * @brief Returns a bitfiled showing "new" faults that occured on Motor 1
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
 *
 * See \link Fault_generation_error_codes Motor Control Faults\endlink for a list of 
 * of all possible faults codes.
 */
uint16_t MC_GetOccurredFaultsMotor1(void)
{
	return MCI_GetOccurredFaults( pMCI[M1] );
}

/**
 * @brief returns a bitfield showing all current faults on Motor 1 
 *
 * This function returns a 16 bit fields containing the Motor Control faults 
 * that are currently active.
 *
 * See \link Fault_generation_error_codes Motor Control Faults\endlink for a list of 
 * of all possible faults codes.
 */
uint16_t MC_GetCurrentFaultsMotor1(void)
{
	return MCI_GetCurrentFaults( pMCI[M1] );
}

/**
 * @brief returns the current state of Motor 1 state machine
 */
State_t  MCI_GetSTMStateMotor1(void)
{
	return MCI_GetSTMState( pMCI[M1] );
}

/**
* @brief  This function requests a user-defined regular conversion. All user
*         defined conversion requests must be performed inside routines with the
*         same priority level. If previous regular conversion request is pending
*         this function has no effect, for this reason is better to call the
*         MC_RegularConvState and check if the state is UDRC_STATE_IDLE before
*         to call MC_RequestRegularConv.
* @param  bChannel ADC channel used for the regular conversion.
* @param  bSamplTime Sampling time selection, ADC_SampleTime_nCycles defined in
*         stm32fxxx_adc.h see ADC_sampling_times.
*/
void MC_ProgramRegularConversion(uint8_t bChannel, uint8_t bSampleTime)
{
  MC_RequestRegularConv( bChannel, bSampleTime );
}

/**
* @brief  Get the last user-defined regular conversion.
* @retval uint16_t It returns converted value or oxFFFF for conversion error.
*         This function returns a valid result if the state returned by
*         MC_RegularConvState is UDRC_STATE_EOC.
*/
uint16_t MC_GetRegularConversionValue(void)
{
  return MC_GetRegularConv();
}

/**
* @brief  Use this function to know the status of the last requested regular
*         conversion.
* @retval UDRC_State_t The state of the last user-defined regular conversion.
*         It can be one of the following values:
*         UDRC_STATE_IDLE no regular conversion request pending.
*         UDRC_STATE_REQUESTED regular conversion has been requested and not
*         completed.
*         UDRC_STATE_EOC regular conversion has been completed but not readed
*         from the user.
*/
UDRC_State_t MC_GetRegularConversionState(void)
{
  return MC_RegularConvState();
}

