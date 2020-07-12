/**
  ******************************************************************************
  * @file    user_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the user interface component of the Motor Control SDK.
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
/* Pre-compiler coherency check */
#include "MC_Type.h"
#include "parameters_conversion.h"

#include "MC_config.h"
#include "user_interface.h"
#include "uart1_user_interface.h"
#include "bus_voltage_sensor.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @defgroup UILib UI Library
 *
 * @brief Library of components dedicated to interfacing the Motor Control Applicaiton with the outside   
 *
 * @todo Complete the documenation
 * @{
 */

/** @defgroup MCUI Motor Control User Interface
  * @brief User Interface Components of the Motor Control SDK
  *
  * These components aim at connecting the Application with the outside. There are three categories
  * of UI Componentes:
  *
  * - Some provide a Graphical User Interface and can be used with the Application is equipped with 
  *   an LCD display in order to control the motor(s) driven by the application. 
  * - Others connect the application with the Motor Conrol Monitor tool via a UART link. The Motor
  *   Control Monitor can control the motor(s) driven by the application and also read and write  
  *   internal variables of the Motor Control subsystem. 
  * - Finally, some UI components allow for using the DAC(s) peripherals in 
  *   order to output internal variables of the Motor Control subsystem for debug purposes.
  *
  * @{
  */

/**
  * @brief  Initialize the user interface component. 
  *
  * Perform the link between the UI, MC interface and MC tuning components.

  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bMCNum  Number of MC instance present in the list.
  * @param  pMCI Pointer on the list of MC interface component to inked with UI.
  * @param  pMCT Pointer on the list of MC tuning component to inked with UI.
  * @param  pUICfg Pointer on the user interface configuration list. 
  *         Each element of the list must be a bit field containing one (or more) of
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  *  @retval none.
  */
void U1UI_Init(U1UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t ** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg)
{

  pHandle->bDriveNum = bMCNum;
  pHandle->pMCI = pMCI;
  pHandle->pMCT = pMCT;
  pHandle->bSelectedDrive = 0u;
  pHandle->pUICfg = pUICfg;
}

/**
  * @brief  Allow to select the MC on which UI operates.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bSelectMC: The new selected MC, zero based, on which UI operates.
  *  @retval Boolean set to true if the bSelectMC is valid oterwise return false.
  */
bool U1UI_SelectMC(U1UI_Handle_t *pHandle,uint8_t bSelectMC)
{
  bool retVal = true;
  if (bSelectMC  >= pHandle->bDriveNum)
  {
    retVal = false;
  }
  else
  {
    pHandle->bSelectedDrive = bSelectMC;
  }
  return retVal;
}

/**
  * @brief  Allow to retrieve the MC on which UI currently operates.
  * @param  pHandle: Pointer on Handle structure of UI component.
  *  @retval Return the currently selected MC, zero based, on which UI operates.
  */
uint8_t U1UI_GetSelectedMC(U1UI_Handle_t *pHandle)
{
  return (pHandle->bSelectedDrive);
}

/**
  * @brief  Retrieve the configuration of the MC on which UI currently operates.
  * @param  pHandle: Pointer on Handle structure of UI component.
  *  @retval Return the currently configuration of selected MC on which UI operates.
  *         It represents a bit field containing one (or more) of
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  */
uint32_t U1UI_GetSelectedMCConfig(U1UI_Handle_t *pHandle)
{
  return pHandle->pUICfg[pHandle->bSelectedDrive];
}

/**
  * @brief  Retrieve the current selected MC tuning component.
  * @param  pHandle: Pointer on Handle structure of UI component.
  *  @retval Return the currently selected MC tuning component on which UI operates.
  */
MCT_Handle_t* U1UI_GetCurrentMCT(U1UI_Handle_t *pHandle)
{
  return (pHandle->pMCT[pHandle->bSelectedDrive]);
}

/**
  * @brief  Allow to execute a SetReg command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bRegID: Code of register to update.
  *         See MC_PROTOCOL_REG_xxx for code definition.
  * @param  wValue: New value to set.
  * @retval Return the currently selected MC, zero based, on which UI operates.
  */
bool U1UI_SetReg(U1UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, int32_t wValue)
{
  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  bool retVal = true;
  switch (bRegID)
  {
  case MC_PROTOCOL_REG_TARGET_MOTOR:
    {
      retVal = U1UI_SelectMC(pHandle,(uint8_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_CONTROL_MODE:
    {
      if ((STC_Modality_t)wValue == STC_TORQUE_MODE)
      {
        MCI_ExecTorqueRamp(pMCI, MCI_GetTeref(pMCI),0);
      }
      if ((STC_Modality_t)wValue == STC_SPEED_MODE)
      {
        MCI_ExecSpeedRamp(pMCI, MCI_GetMecSpeedRef01Hz(pMCI),0);
      }
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KP:
    {
      PID_SetKP(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KI:
    {
      PID_SetKI(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KD:
    {
      PID_SetKD(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(pMCI);
      currComp.qI_Component1 = (int16_t)wValue;
      MCI_SetCurrentReferences(pMCI,currComp);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KP:
    {
      PID_SetKP(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KI:
    {
      PID_SetKI(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KD:
    {
      PID_SetKD(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(pMCI);
      currComp.qI_Component2 = (int16_t)wValue;
      MCI_SetCurrentReferences(pMCI,currComp);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KP:
    {
      PID_SetKP(pMCT->pPIDId,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KI:
    {
      PID_SetKI(pMCT->pPIDId,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KD:
    {
      PID_SetKD(pMCT->pPIDId,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_IQ_SPEEDMODE:
    {
      MCI_SetIdref(pMCI,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      MCI_ExecSpeedRamp(pMCI,(int16_t)(wValue/6),0);
    }
    break;

  default:
    retVal = false;
    break;
  }

  return retVal;
}

/**
  * @brief  Allow to execute a GetReg command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bRegID: Code of register to read. 
  *         See MC_PROTOCOL_REG_xxx values for code definition.
  *  @retval Register value read.
  */
int32_t U1UI_GetReg(U1UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID)
{
  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  int32_t bRetVal = (int32_t)GUI_ERROR_CODE;
  switch (bRegID)
  {
    case MC_PROTOCOL_REG_TARGET_MOTOR:
      {
        bRetVal = (int32_t)U1UI_GetSelectedMC(pHandle);
      }
      break;
    case MC_PROTOCOL_REG_FLAGS:
      {
        bRetVal = (int32_t)STM_GetFaultState(pMCT->pStateMachine);
      }
      break;
    case MC_PROTOCOL_REG_STATUS:
      {
        bRetVal = (int32_t)STM_GetState(pMCT->pStateMachine);
      }
      break;
    case MC_PROTOCOL_REG_CONTROL_MODE:
      {
        bRetVal = (int32_t)MCI_GetControlMode(pMCI);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_REF:
      {
        bRetVal = (int32_t)(MCI_GetMecSpeedRef01Hz(pMCI) * 6);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_KP:
      {
        bRetVal = (int32_t)PID_GetKP(pMCT->pPIDSpeed);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_KP_DIV:
      {
        bRetVal = (int32_t)PID_GetKPDivisor(pMCT->pPIDSpeed);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_KI:
      {
        bRetVal = (int32_t)PID_GetKI(pMCT->pPIDSpeed);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_KI_DIV:
      {
        bRetVal = (int32_t)PID_GetKIDivisor(pMCT->pPIDSpeed);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_KD:
      {
        bRetVal = (int32_t)PID_GetKD(pMCT->pPIDSpeed);
      }
      break;
    case MC_PROTOCOL_REG_TORQUE_REF:
      {
        Curr_Components currComp;
        currComp = MCI_GetIqdref(pMCI);
        bRetVal = (int32_t)currComp.qI_Component1;
      }
      break;
    case MC_PROTOCOL_REG_TORQUE_KP:
      {
        bRetVal = (int32_t)PID_GetKP(pMCT->pPIDIq);
      }
      break;
    case MC_PROTOCOL_REG_TORQUE_KI:
      {
        bRetVal = (int32_t)PID_GetKI(pMCT->pPIDIq);
      }
      break;
    case MC_PROTOCOL_REG_TORQUE_KD:
      {
        bRetVal = (int32_t)PID_GetKD(pMCT->pPIDIq);
      }
      break;
    case MC_PROTOCOL_REG_FLUX_REF:
    case MC_PROTOCOL_REG_IQ_SPEEDMODE:
      {
        Curr_Components currComp;
        currComp = MCI_GetIqdref(pMCI);
        bRetVal = (int32_t)currComp.qI_Component2;
      }
      break;
    case MC_PROTOCOL_REG_FLUX_KP:
      {
        bRetVal = (int32_t)PID_GetKP(pMCT->pPIDId);
      }
      break;
    case MC_PROTOCOL_REG_FLUX_KI:
      {
        bRetVal = (int32_t)PID_GetKI(pMCT->pPIDId);
      }
      break;
    case MC_PROTOCOL_REG_FLUX_KD:
      {
        bRetVal = (int32_t)PID_GetKD(pMCT->pPIDId);
      }
      break;
    case MC_PROTOCOL_REG_BUS_VOLTAGE:
      {
        bRetVal = (int32_t)VBS_GetAvBusVoltage_V(pMCT->pBusVoltageSensor);
      }
      break;
    case MC_PROTOCOL_REG_HEATS_TEMP:
      {
        bRetVal = (int32_t)NTC_GetAvTemp_C(pMCT->pTemperatureSensor);
      }
      break;
    case MC_PROTOCOL_REG_MOTOR_POWER:
      {
        bRetVal = MPM_GetAvrgElMotorPowerW(pMCT->pMPM);
      }
      break;
    case MC_PROTOCOL_REG_SPEED_MEAS:
      {
        bRetVal = (int32_t)(MCI_GetAvrgMecSpeed01Hz(pMCI) * 6);
      }
      break;
    case MC_PROTOCOL_REG_TORQUE_MEAS:
    case MC_PROTOCOL_REG_I_Q:
      {
        bRetVal = MCI_GetIqd(pMCI).qI_Component1;
      }
      break;
    case MC_PROTOCOL_REG_FLUX_MEAS:
    case MC_PROTOCOL_REG_I_D:
      {
        bRetVal = MCI_GetIqd(pMCI).qI_Component2;
      }
      break;
    case MC_PROTOCOL_REG_RUC_STAGE_NBR:
      {
        bRetVal = (int32_t)RUC_GetNumberOfPhases(pMCT->pRevupCtrl);
      }
      break;
    case MC_PROTOCOL_REG_I_A:
      {
        bRetVal = MCI_GetIab(pMCI).qI_Component1;
      }
      break;
    case MC_PROTOCOL_REG_I_B:
      {
        bRetVal = MCI_GetIab(pMCI).qI_Component2;
      }
      break;
    case MC_PROTOCOL_REG_I_ALPHA:
      {
        bRetVal = MCI_GetIalphabeta(pMCI).qI_Component1;
      }
      break;
    case MC_PROTOCOL_REG_I_BETA:
      {
        bRetVal = MCI_GetIalphabeta(pMCI).qI_Component2;
      }
      break;
    case MC_PROTOCOL_REG_I_Q_REF:
      {
        bRetVal = MCI_GetIqdref(pMCI).qI_Component1;
      }
      break;
    case MC_PROTOCOL_REG_I_D_REF:
      {
        bRetVal = MCI_GetIqdref(pMCI).qI_Component2;
      }
      break;
    case MC_PROTOCOL_REG_V_Q:
      {
        bRetVal = MCI_GetVqd(pMCI).qV_Component1;
      }
      break;
    case MC_PROTOCOL_REG_V_D:
      {
        bRetVal = MCI_GetVqd(pMCI).qV_Component2;
      }
      break;
    case MC_PROTOCOL_REG_V_ALPHA:
      {
        bRetVal = MCI_GetValphabeta(pMCI).qV_Component1;
      }
      break;
    case MC_PROTOCOL_REG_V_BETA:
      {
        bRetVal = MCI_GetValphabeta(pMCI).qV_Component2;
      }
      break;
    case MC_PROTOCOL_REG_MEAS_EL_ANGLE:
      {
        uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if ((MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
            (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
        {
          pSPD = pMCT->pSpeedSensorMain;
        }
        if ((AUX_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
            (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
        {
          pSPD = pMCT->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          bRetVal = SPD_GetElAngle(pSPD);
        }
      }
      break;
    case MC_PROTOCOL_REG_MEAS_ROT_SPEED:
      {
        uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
        SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
        if ((MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
            (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
        {
          pSPD = pMCT->pSpeedSensorMain;
        }
        if ((AUX_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
            (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
        {
          pSPD = pMCT->pSpeedSensorAux;
        }
        if (pSPD != MC_NULL)
        {
          bRetVal = SPD_GetS16Speed(pSPD);
        }
      }
      break;
    case MC_PROTOCOL_REG_MAX_APP_SPEED:
      {
        bRetVal = STC_GetMaxAppPositiveMecSpeed01Hz(pMCT->pSpeednTorqueCtrl) * 6;
      }
      break;
    case MC_PROTOCOL_REG_MIN_APP_SPEED:
      {
        bRetVal = STC_GetMinAppNegativeMecSpeed01Hz(pMCT->pSpeednTorqueCtrl) * 6;
      }
      break;
    case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
      {
        bRetVal = (int32_t)(MCI_GetLastRampFinalSpeed(pMCI) * 6);
      }
      else
      {
        bRetVal = (int32_t)(MCI_GetMecSpeedRef01Hz(pMCI) * 6);
      }
    }
    break;

  case MC_PROTOCOL_REG_UID:
    {
      bRetVal = (int32_t)(MC_UID);
    }
    break;

  case MC_PROTOCOL_REG_CTRBDID:
    {
      bRetVal = CTRBDID;
    }
    break;

  case MC_PROTOCOL_REG_PWBDID:
    {
      bRetVal = PWBDID;
    }
    break;

  case MC_PROTOCOL_REG_PWBDID2:
    {
      bRetVal = 0;
    }
    break;

  default:
    break;
  }
  return bRetVal;
}

/**
  * @brief  Allow to execute a command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bCmdID: Code of command to execute.
  *         See MC_PROTOCOL_CMD_xxx for code definition.
  *  @retval Return true if the command executed succesfully, otherwise false.
*/
bool U1UI_ExecCmd(U1UI_Handle_t *pHandle, uint8_t bCmdID)
{
  bool retVal = true;

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  switch (bCmdID)
  {
  case MC_PROTOCOL_CMD_START_MOTOR:
    {
      /* Call MCI Start motor; */
      MCI_StartMotor(pMCI);
    }
    break;
  case MC_PROTOCOL_CMD_STOP_MOTOR:
  case MC_PROTOCOL_CMD_SC_STOP:
    {
      /* Call MCI Stop motor; */
      MCI_StopMotor(pMCI);
    }
    break;
  case MC_PROTOCOL_CMD_STOP_RAMP:
    {
      if (MCI_GetSTMState(pMCI) == RUN)
      {
        MCI_StopSpeedRamp(pMCI);
      }
    }
    break;
  case MC_PROTOCOL_CMD_PING:
    {
    }
    break;
  case MC_PROTOCOL_CMD_START_STOP:
    {
      /* Queries the STM and a command start or stop depending on the state. */
      if (MCI_GetSTMState(pMCI) == IDLE)
      {
        MCI_StartMotor(pMCI);
      }
      else
      {
        MCI_StopMotor(pMCI);
      }
    }
    break;
  case MC_PROTOCOL_CMD_RESET:
    {
    }
    break;
  case MC_PROTOCOL_CMD_FAULT_ACK:
    {
      MCI_FaultAcknowledged(pMCI);
    }
    break;
  case MC_PROTOCOL_CMD_ENCODER_ALIGN:
    {
      MCI_EncoderAlign(pMCI);
    }
    break;
  case MC_PROTOCOL_CMD_IQDREF_CLEAR:
    {
      MCI_Clear_Iqdref(pMCI);
    }
    break;

  default:
    retVal = false;
    break;
  }
  return retVal;
}

/**
  * @brief  Allow to execute a speed ramp command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  wFinalMecSpeedRPM: Final speed value expressed in RPM.
  * @param  hDurationms: Duration of the ramp expressed in milliseconds. 
  *         It is possible to set 0 to perform an instantaneous change in the value.
  *  @retval Return true if the command executed succesfully, otherwise false.
  */
bool U1UI_ExecSpeedRamp(U1UI_Handle_t *pHandle, int32_t wFinalMecSpeedRPM, uint16_t hDurationms)
{

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  /* Call MCI Exec Ramp */
  MCI_ExecSpeedRamp(pMCI,(int16_t)(wFinalMecSpeedRPM/6),hDurationms);
  return true;
}

/**
  * @brief  It is used to execute a torque ramp command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  hTargetFinal: final torque value. See MCI interface for more
            details.
  * @param  hDurationms: the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  *  @retval Return true if the command executed succesfully, otherwise false.
  */
bool U1UI_ExecTorqueRamp(U1UI_Handle_t *pHandle, int16_t hTargetFinal, uint16_t hDurationms)
{

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  /* Call MCI Exec Ramp */
  MCI_ExecTorqueRamp(pMCI,hTargetFinal,hDurationms);
  return true;
}

/**
  * @brief  It is used to execute a get Revup data command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bStage: is the rev up phase, zero based, to be read.
  * @param  pDurationms: is the pointer to an uint16_t variable used to retrieve
  *         the duration of the Revup stage.
  * @param  pFinalMecSpeed01Hz: is the pointer to an int16_t variable used to
  *         retrieve the mechanical speed at the end of that stage expressed in
  *         0.1Hz.
  * @param  pFinalTorque: is the pointer to an int16_t variable used to
  *         retrieve the value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  *  @retval Return true if the command executed succesfully, otherwise false.
  */
bool U1UI_GetRevupData(U1UI_Handle_t *pHandle, uint8_t bStage, uint16_t* pDurationms,
                     int16_t* pFinalMecSpeed01Hz, int16_t* pFinalTorque )
{
  bool hRetVal = true;

  RevUpCtrl_Handle_t *pRevupCtrl = pHandle->pMCT[pHandle->bSelectedDrive]->pRevupCtrl;
  if (pRevupCtrl)
  {
    *pDurationms = RUC_GetPhaseDurationms(pRevupCtrl, bStage);
    *pFinalMecSpeed01Hz = RUC_GetPhaseFinalMecSpeed01Hz(pRevupCtrl, bStage);
    *pFinalTorque = RUC_GetPhaseFinalTorque(pRevupCtrl, bStage);
  }
  else
  {
    hRetVal = false;
  }
  return hRetVal;
}

/**
  * @brief  It is used to execute a set Revup data command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  bStage: is the rev up phase, zero based, to be modified.
  * @param  hDurationms: is the new duration of the Revup stage.
  * @param  hFinalMecSpeed01Hz: is the new mechanical speed at the end of that
  *         stage expressed in 0.1Hz.
  * @param  hFinalTorque: is the new value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  *  @retval Return true if the command executed succesfully, otherwise false.
  */
bool U1UI_SetRevupData(U1UI_Handle_t *pHandle, uint8_t bStage, uint16_t hDurationms,
                     int16_t hFinalMecSpeed01Hz, int16_t hFinalTorque )
{
  RevUpCtrl_Handle_t *pRevupCtrl = pHandle->pMCT[pHandle->bSelectedDrive]->pRevupCtrl;
  RUC_SetPhaseDurationms(pRevupCtrl, bStage, hDurationms);
  RUC_SetPhaseFinalMecSpeed01Hz(pRevupCtrl, bStage, hFinalMecSpeed01Hz);
  RUC_SetPhaseFinalTorque(pRevupCtrl, bStage, hFinalTorque);
  return true;
}

/**
  * @brief  Allow to execute a set current reference command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  hIqRef: Current Iq reference on qd reference frame. 
  *         This value is expressed in digit. 
  * @note   current convertion formula (from digit to Amps):
  *               Current(Amps) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  hIdRef: Current Id reference on qd reference frame. 
  *         This value is expressed in digit. See hIqRef param description.
  * @retval none.
  */
void U1UI_SetCurrentReferences(U1UI_Handle_t *pHandle, int16_t hIqRef, int16_t hIdRef)
{

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
  Curr_Components currComp;
  currComp.qI_Component1 = hIqRef;
  currComp.qI_Component2 = hIdRef;
  MCI_SetCurrentReferences(pMCI,currComp);
}

/**
  * @brief  Allow to get information about MP registers available for each step.
  *         PC send to the FW the list of steps to get the available registers. 
  *         The FW returs the list of available registers for that steps.
  * @param  stepList: List of requested steps.
  * @param  pMPInfo: The returned list of register.
  *         It is populated by this function.
  * @retval true if MP is enabled, false otherwise.
  */
bool U1UI_GetMPInfo(pMPInfo_t stepList, pMPInfo_t pMPInfo)
{
    return false;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
