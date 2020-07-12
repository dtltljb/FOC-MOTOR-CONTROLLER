/**
  ******************************************************************************
  * @file    mc_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the MC Interface component of the Motor Control SDK:
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
#include "mc_math.h"
#include "speed_torq_ctrl.h"

#include "mc_interface.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCInterface Motor Control Interface
  * @brief MC Interface component of the Motor Control SDK
  *
  * @todo Document the MC Interface "module".
  *
  * @{
  */

/* Private macros ------------------------------------------------------------*/
/**
  * @brief This macro converts the exported enum from the state machine to the corresponding bit field.
  */
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* Functions -----------------------------------------------*/

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the
  *         state machine object, the speed and torque controller, and the FOC
  *         drive object to be used by MC Interface.
  * @param  pHandle pointer on the component instance to initialize.
  * @param  pSTM the state machine object used by the MCI.
  * @param  pSTC the speed and torque controller used by the MCI.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  * @retval none.
  */
void MCI_Init( MCI_Handle_t * pHandle, STM_Handle_t *pSTM, SpeednTorqCtrl_Handle_t *pSTC, pFOCVars_t pFOCVars )
{
  pHandle->pSTM = pSTM;
  pHandle->pSTC = pSTC;
  pHandle->pFOCVars = pFOCVars;

  /* Buffer related initialization */
  pHandle->lastCommand = MCI_NOCOMMANDSYET;
  pHandle->hFinalSpeed = 0;
  pHandle->hFinalTorque = 0;
  pHandle->hDurationms = 0;
  pHandle->CommandState = MCI_BUFFER_EMPTY;
}

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  * @retval none.
  */
void MCI_ExecSpeedRamp( MCI_Handle_t * pHandle,  int16_t hFinalSpeed, uint16_t hDurationms )
{
  pHandle->lastCommand = MCI_EXECSPEEDRAMP;
  pHandle->hFinalSpeed = hFinalSpeed;
  pHandle->hDurationms = hDurationms;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_SPEED_MODE;
}

/**
  * @brief  This is a buffered command to set a motor torque ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  hFinalTorque is the value of motor torque reference at the end of
  *         the ramp. This value represents actually the Iq current expressed in
  *         digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  * @retval none.
  */
void MCI_ExecTorqueRamp( MCI_Handle_t * pHandle,  int16_t hFinalTorque, uint16_t hDurationms )
{
  pHandle->lastCommand = MCI_EXECTORQUERAMP;
  pHandle->hFinalTorque = hFinalTorque;
  pHandle->hDurationms = hDurationms;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
  * @brief  This is a buffered command to set directly the motor current
  *         references Iq and Id. This commands don't become active as soon as
  *         it is called but it will be executed when the pSTM state is
  *         START_RUN or RUN. User can check the status of the command calling
  *         the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to work on.
  * @param  Iqdref current references on qd reference frame in Curr_Components
  *         format.
  * @retval none.
  */
void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, Curr_Components Iqdref )
{
  pHandle->lastCommand = MCI_SETCURRENTREFERENCES;
  pHandle->Iqdref.qI_Component1 = Iqdref.qI_Component1;
  pHandle->Iqdref.qI_Component2 = Iqdref.qI_Component2;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
  * @brief  This is a user command used to begin the start-up procedure.
  *         If the state machine is in IDLE state the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  *         Before calling MCI_StartMotor it is mandatory to execute one of
  *         these commands:\n
  *         MCI_ExecSpeedRamp\n
  *         MCI_ExecTorqueRamp\n
  *         MCI_SetCurrentReferences\n
  *         Otherwise the behaviour in run state will be unpredictable.\n
  *         <B>Note:</B> The MCI_StartMotor command is used just to begin the
  *         start-up procedure moving the state machine from IDLE state to
  *         IDLE_START. The command MCI_StartMotor is not blocking the execution
  *         of project until the motor is really running; to do this, the user
  *         have to check the state machine and verify that the RUN state (or
  *         any other state) has been reached.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCI_StartMotor( MCI_Handle_t * pHandle )
{
  bool RetVal = STM_NextState( pHandle->pSTM, IDLE_START );

  if (RetVal == true)
  {
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  }

  return RetVal;
}

/**
  * @brief  This is a user command used to begin the stop motor procedure.
  *         If the state machine is in RUN or START states the command is
  *         executed instantaneously otherwise the command is discarded. User
  *         must take care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_StopMotor command is used just to begin the
  *         stop motor procedure moving the state machine to ANY_STOP.
  *         The command MCI_StopMotor is not blocking the execution of project
  *         until the motor is really stopped; to do this, the user have to
  *         check the state machine and verify that the IDLE state has been
  *         reached again.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCI_StopMotor( MCI_Handle_t * pHandle )
{
  return STM_NextState( pHandle->pSTM, ANY_STOP );
}

/**
  * @brief  This is a user command used to indicate that the user has seen the
  *         error condition. If is possible, the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle )
{
  return STM_FaultAcknowledged( pHandle->pSTM );
}

/**
  * @brief  This is a user command used to begin the encoder alignment procedure.
  *         If the state machine is in IDLE state the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_EncoderAlign command is used just to begin the
  *         encoder alignment procedure moving the state machine from IDLE state
  *         to IDLE_ALIGNMENT. The command MCI_EncoderAlign is not blocking the
  *         execution of project until the encoder is really calibrated; to do
  *         this, the user have to check the state machine and verify that the
  *         IDLE state has been reached again.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
bool MCI_EncoderAlign( MCI_Handle_t * pHandle )
{
  return STM_NextState( pHandle->pSTM,IDLE_ALIGNMENT );
}

/**
  * @brief  This is usually a method managed by task. It must be called
  *         periodically in order to check the status of the related pSTM object
  *         and eventually to execute the buffered command if the condition
  *         occurs.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval none.
  */
void MCI_ExecBufferedCommands( MCI_Handle_t * pHandle )
{
  if ( pHandle != MC_NULL )
  {
    if (pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
      bool commandHasBeenExecuted = false;
      switch ( pHandle->lastCommand )
      {
      case MCI_EXECSPEEDRAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode( pHandle->pSTC, STC_SPEED_MODE );
          commandHasBeenExecuted = STC_ExecRamp( pHandle->pSTC, pHandle->hFinalSpeed, pHandle->hDurationms );
        }
        break;
      case MCI_EXECTORQUERAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode( pHandle->pSTC, STC_TORQUE_MODE );
          commandHasBeenExecuted = STC_ExecRamp( pHandle->pSTC, pHandle->hFinalTorque, pHandle->hDurationms );
        }
        break;
      case MCI_SETCURRENTREFERENCES:
        {
          pHandle->pFOCVars->bDriveInput = EXTERNAL;
          pHandle->pFOCVars->Iqdref = pHandle->Iqdref;
          commandHasBeenExecuted = true;
        }
        break;
      default:
        break;
      }

      if ( commandHasBeenExecuted )
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
      }
      else
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
      }
    }
  }
}

/**
  * @brief  It returns information about the state of the last buffered command.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval CommandState_t  It can be one of the following codes:
  *         - MCI_BUFFER_EMPTY if no buffered command has been called.
  *         - MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command
  *         condition hasn't already occurred.
  *         - MCI_COMMAND_EXECUTED_SUCCESFULLY if the buffered command has
  *         been executed successfully. In this case calling this function reset
  *         the command state to BC_BUFFER_EMPTY.
  *         - MCI_COMMAND_EXECUTED_UNSUCCESFULLY if the buffered command has
  *         been executed unsuccessfully. In this case calling this function
  *         reset the command state to BC_BUFFER_EMPTY.
  */
MCI_CommandState_t  MCI_IsCommandAcknowledged( MCI_Handle_t * pHandle )
{
  MCI_CommandState_t retVal = pHandle->CommandState;

  if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) |
      (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY))
  {
    pHandle->CommandState = MCI_BUFFER_EMPTY;
  }
  return retVal;
}

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval State_t It returns the current state of the related pSTM object.
  */
State_t  MCI_GetSTMState( MCI_Handle_t * pHandle )
{
  return STM_GetState( pHandle->pSTM );
}

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint16_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle )
{
  return (uint16_t)(STM_GetFaultState( pHandle->pSTM ));
}

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
uint16_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle )
{
  return (uint16_t)(STM_GetFaultState( pHandle->pSTM ) >> 16);
}

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t MCI_GetControlMode( MCI_Handle_t * pHandle )
{
  return pHandle->LastModalitySetByUser;
}

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.qI_Component1 of the last command.
  */
int16_t MCI_GetImposedMotorDirection( MCI_Handle_t * pHandle )
{
  int16_t retVal = 1;

  switch ( pHandle->lastCommand )
  {
  case MCI_EXECSPEEDRAMP:
    if ( pHandle->hFinalSpeed < 0 )
    {
      retVal = -1;
    }
    break;
  case MCI_EXECTORQUERAMP:
    if ( pHandle->hFinalTorque < 0 )
    {
      retVal = -1;
    }
    break;
  case MCI_SETCURRENTREFERENCES:
    if ( pHandle->Iqdref.qI_Component1 < 0 )
    {
      retVal = -1;
    }
    break;
  default:
    break;
  }
  return retVal;
}

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in tenths of HZ.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t last ramp final speed sent by the user expressed in tehts
  *         of HZ.
  */
int16_t MCI_GetLastRampFinalSpeed( MCI_Handle_t * pHandle )
{
  int16_t hRetVal = 0;

  /* Examine the last buffered commands */
  if (pHandle->lastCommand == MCI_EXECSPEEDRAMP)
  {
    hRetVal = pHandle->hFinalSpeed;
  }
  return hRetVal;
}

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool MCI_RampCompleted( MCI_Handle_t * pHandle )
{
  bool retVal = false;

  if ( (STM_GetState( pHandle->pSTM )) == RUN )
  {
    retVal = STC_RampCompleted( pHandle->pSTC );
  }

  return retVal;
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is executed, false otherwise.
  */
bool MCI_StopSpeedRamp( MCI_Handle_t * pHandle )
{
  return STC_StopSpeedRamp( pHandle->pSTC );
}

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
bool MCI_GetSpdSensorReliability( MCI_Handle_t * pHandle )
{
  SpeednPosFdbk_Handle_t * SpeedSensor = STC_GetSpeedSensor( pHandle->pSTC );

  return ( SPD_Check( SpeedSensor ) );
}

/**
  * @brief  It returns the last computed average mechanical speed, expressed in
  *         01Hz (tenth of Hertz) and related to the sensor actually used by FOC
  *         algorithm
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t rotor average mechanical speed (01Hz)
  */
int16_t MCI_GetAvrgMecSpeed01Hz( MCI_Handle_t * pHandle )
{
  SpeednPosFdbk_Handle_t * SpeedSensor = STC_GetSpeedSensor( pHandle->pSTC );

  return ( SPD_GetAvrgMecSpeed01Hz( SpeedSensor ) );
}

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ.
  */
int16_t MCI_GetMecSpeedRef01Hz( MCI_Handle_t * pHandle )
{
  return ( STC_GetMecSpeedRef01Hz( pHandle->pSTC ) );
}


/**
  * @brief  It returns stator current Iab in Curr_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Iab
  */
Curr_Components MCI_GetIab( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iab );
}

/**
  * @brief  It returns stator current Ialphabeta in Curr_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Ialphabeta
  */
Curr_Components MCI_GetIalphabeta( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Ialphabeta );
}

/**
  * @brief  It returns stator current Iqd in Curr_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Iqd
  */
Curr_Components MCI_GetIqd( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iqd );
}

/**
  * @brief  It returns stator current IqdHF in Curr_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current IqdHF if HFI is selected as main
  *         sensor. Otherwise it returns { 0, 0}.
  */
Curr_Components MCI_GetIqdHF( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->IqdHF );
}

/**
  * @brief  It returns stator current Iqdref in Curr_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Iqdref
  */
Curr_Components MCI_GetIqdref( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iqdref );
}

/**
  * @brief  It returns stator current Vqd in Volt_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Vqd
  */
Volt_Components MCI_GetVqd( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Vqd );
}

/**
  * @brief  It returns stator current Valphabeta in Volt_Components format
  * @param  pHandle Pointer on the component instance to work on.
  * @retval Curr_Components Stator current Valphabeta
  */
Volt_Components MCI_GetValphabeta( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Valphabeta );
}

/**
  * @brief  It returns the rotor electrical angle actually used for reference
  *         frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Rotor electrical angle in dpp format
  */
int16_t MCI_GetElAngledpp( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->hElAngle );
}

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Teref
  */
int16_t MCI_GetTeref( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->hTeref );
}

/**
  * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
  *         To convert s16A into Ampere following formula must be used:
  *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Motor phase current (0-to-peak) in s16A
  */
int16_t MCI_GetPhaseCurrentAmplitude( MCI_Handle_t * pHandle )
{
  Curr_Components Local_Curr;
  int32_t wAux1, wAux2;

  Local_Curr = pHandle->pFOCVars->Ialphabeta;
  wAux1 = (int32_t)(Local_Curr.qI_Component1) * Local_Curr.qI_Component1;
  wAux2 = (int32_t)(Local_Curr.qI_Component2) * Local_Curr.qI_Component2;

  wAux1 += wAux2;
  wAux1 = MCM_Sqrt( wAux1 );

  if ( wAux1 > INT16_MAX )
  {
    wAux1 = (int32_t) INT16_MAX;
  }

  return ( (int16_t)wAux1 );
}

/**
  * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in
  *         s16V. To convert s16V into Volts following formula must be used:
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t Motor phase voltage (0-to-peak) in s16V
  */
int16_t MCI_GetPhaseVoltageAmplitude( MCI_Handle_t * pHandle )
{
  Volt_Components Local_Voltage;
  int32_t wAux1,wAux2;

  Local_Voltage = pHandle->pFOCVars->Valphabeta;
  wAux1 = (int32_t)(Local_Voltage.qV_Component1) * Local_Voltage.qV_Component1;
  wAux2 = (int32_t)(Local_Voltage.qV_Component2) * Local_Voltage.qV_Component2;

  wAux1 += wAux2;
  wAux1 = MCM_Sqrt( wAux1 );

  if ( wAux1 > INT16_MAX )
  {
    wAux1 = (int32_t) INT16_MAX;
  }

  return ( (int16_t) wAux1 );
}

/**
  * @brief  When bDriveInput is set to INTERNAL, Idref should is normally managed
  *         by FOC_CalcCurrRef. Neverthless, this method allows forcing changing
  *         Idref value. Method call has no effect when either flux weakening
  *         region is entered or MTPA is enabled
  * @param  pHandle Pointer on the component instance to work on.
  * @param  int16_t New target Id value
  * @retval none
  */
void MCI_SetIdref( MCI_Handle_t * pHandle, int16_t hNewIdref )
{
  pHandle->pFOCVars->Iqdref.qI_Component2 = hNewIdref;
  pHandle->pFOCVars->UserIdref = hNewIdref;
}

/**
  * @brief  It re-initializes Iqdref variables with their default values.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval none
  */
void MCI_Clear_Iqdref( MCI_Handle_t * pHandle )
{
  pHandle->pFOCVars->Iqdref = STC_GetDefaultIqdref( pHandle->pSTC );
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
