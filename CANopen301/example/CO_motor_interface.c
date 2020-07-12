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
#include "CO_motor_interface.h"
#include "bus_voltage_sensor.h"
#include "user_debug.h"
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
void MI_Init(UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t ** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg)
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
bool MI_SelectMC(UI_Handle_t *pHandle,uint8_t bSelectMC)
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
  * @brief  Retrieve the current selected MC tuning component.
  * @param  pHandle: Pointer on Handle structure of UI component.
  *  @retval Return the currently selected MC tuning component on which UI operates.
  */
MCT_Handle_t* MI_GetCurrentMCT(UI_Handle_t *pHandle)
{
  return (pHandle->pMCT[pHandle->bSelectedDrive]);
}


/**
  * @brief  Retrieve the configuration of the MC on which UI currently operates.
  * @param  pHandle: Pointer on Handle structure of UI component.
  *  @retval Return the currently configuration of selected MC on which UI operates.
  *         It represents a bit field containing one (or more) of
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  */
uint32_t MI_GetSelectedMCConfig(UI_Handle_t *pHandle)
{
  return pHandle->pUICfg[pHandle->bSelectedDrive];
}


/**
  * @brief  Allow to execute a SetReg command coming from the user.
  * @param  pHandle: Pointer on Handle structure of UI component.
  * @param  pSDO: CO_SDO_PROCESS update.
  *         See CO_SDO.C for code definition.
  * 
  * @retval Return the currently selected MC, zero based, on which UI operates.
  */
bool MI_SetReg(UI_Handle_t *pHandle, CO_SDO_t 	*pSDO)
{
  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  uint16_t	index	=	pSDO->ODF_arg.index	;
  bool retVal = true;
  uint32_t wValue;
	wValue	=	pSDO->CANrxData[4];
	wValue	|=	(uint32_t)(pSDO->CANrxData[5]<<8);
	wValue	|=	(uint32_t)(pSDO->CANrxData[6]<<16);	
	wValue	|=	(uint32_t)(pSDO->CANrxData[6]<<24);	
	
  switch (index)
  {
	/*	SPEED	*/
  case CO_Index_SPEED_KP:
    {
      PID_SetKP(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;
  case CO_Index_SPEED_KI:
    {
      PID_SetKI(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;
  case CO_Index_SPEED_KD:
    {
      PID_SetKD(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;
	
	/*	TORQUE	*/	
  case CO_Index_TORQUE_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(pMCI);
      currComp.qI_Component1 = (int16_t)wValue;
      MCI_SetCurrentReferences(pMCI,currComp);
    }
    break;
  case CO_Index_TORQUE_KP:
    {
      PID_SetKP(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;
  case CO_Index_TORQUE_KI:
    {
      PID_SetKI(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;
  case CO_Index_TORQUE_KD:
    {
      PID_SetKD(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;
	
	/**/
  case CO_Index_CONTROL_MODE:								/*	Control mode	*/
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

  case CO_Index_RAMP_FINAL_SPEED:							/*	TARGET SPEED	*/
    {
      MCI_ExecSpeedRamp(pMCI,(int16_t)(wValue/6),0);
    }
    break;
	
/**************user add motor control command start********/
  case CO_Index_CMD_START_STOP:								/*0x6040 Control Word */
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
	
  case CO_Index_CMD_START_MOTOR:
    {
      /* Call MCI Start motor; */
      MCI_StartMotor(pMCI);
    }
    break;
  case CO_Index_CMD_STOP_MOTOR:
    {
      /* Call MCI Stop motor; */
      MCI_StopMotor(pMCI);
    }
    break;
	
  case CO_Index_CMD_STOP_RAMP:								/*	STOP SPEED	*/
    {
      if (MCI_GetSTMState(pMCI) == RUN)
      {
        MCI_StopSpeedRamp(pMCI);
      }
    }
    break;

  case CO_Index_CMD_FAULT_ACK:
    {
      MCI_FaultAcknowledged(pMCI);
    }
    break;
	
  case CO_Index_CMD_ENCODER_ALIGN:
    {
      MCI_EncoderAlign(pMCI);
    }
    break;
  case CO_Index_CMD_IQDREF_CLEAR:		/* IQ REF CLEAR */
    {
      MCI_Clear_Iqdref(pMCI);
    }
    break;
/**************user add motor control command end********/
  default:
    retVal = false;
		Error_Handler();
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
int32_t MI_GetReg(UI_Handle_t *pHandle, CO_SDO_t 	*pSDO)
{
	
  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
	
	uint16_t	index	=	pSDO->ODF_arg.index	;
  int32_t bRetVal = (int32_t)GUI_ERROR_CODE;
	
  switch (index)
  {
	  	/*	STATUS,VOLTAGE,TEMP,POWER*/  
    case CO_Index_Motor_STATUS:
      {
        bRetVal = (int32_t)STM_GetState(pMCT->pStateMachine);
      }
      break;
    case CO_Index_BUS_VOLTAGE:
      {
        bRetVal = (int32_t)VBS_GetAvBusVoltage_V(pMCT->pBusVoltageSensor);
      }
      break;
    case CO_Index_HEATS_TEMP:
      {
        bRetVal = (int32_t)NTC_GetAvTemp_C(pMCT->pTemperatureSensor);
      }
      break;
    case CO_Index_MOTOR_POWER:
      {
        bRetVal = MPM_GetAvrgElMotorPowerW(pMCT->pMPM);
      }
      break;
	  
	
	 /* speed para*/ 
    case CO_Index_SPEED_REF:
      {
        bRetVal = (int32_t)(MCI_GetMecSpeedRef01Hz(pMCI) * 6);
      }
      break;
    case CO_Index_SPEED_KP:
      {
        bRetVal = (int32_t)PID_GetKP(pMCT->pPIDSpeed);
      }
      break;
    case CO_Index_SPEED_KP_DIV:
      {
        bRetVal = (int32_t)PID_GetKPDivisor(pMCT->pPIDSpeed);
      }
      break;
    case CO_Index_SPEED_KI:
      {
        bRetVal = (int32_t)PID_GetKI(pMCT->pPIDSpeed);
      }
      break;
    case CO_Index_SPEED_KI_DIV:
      {
        bRetVal = (int32_t)PID_GetKIDivisor(pMCT->pPIDSpeed);
      }
      break;
    case CO_Index_SPEED_KD:
      {
        bRetVal = (int32_t)PID_GetKD(pMCT->pPIDSpeed);
      }
      break;
    case CO_Index_MAX_APP_SPEED:
      {
        bRetVal = STC_GetMaxAppPositiveMecSpeed01Hz(pMCT->pSpeednTorqueCtrl) * 6;
      }
      break;
    case CO_Index_MIN_APP_SPEED:
      {
        bRetVal = STC_GetMinAppNegativeMecSpeed01Hz(pMCT->pSpeednTorqueCtrl) * 6;
      }
      break;
	  
	/* torque para*/   
    case CO_Index_TORQUE_REF:
      {
        Curr_Components currComp;
        currComp = MCI_GetIqdref(pMCI);
        bRetVal = (int32_t)currComp.qI_Component1;
      }
      break;
    case CO_Index_TORQUE_KP:
      {
        bRetVal = (int32_t)PID_GetKP(pMCT->pPIDIq);
      }
      break;
    case CO_Index_TORQUE_KI:
      {
        bRetVal = (int32_t)PID_GetKI(pMCT->pPIDIq);
      }
      break;
    case CO_Index_TORQUE_KD:
      {
        bRetVal = (int32_t)PID_GetKD(pMCT->pPIDIq);
      }
      break;
	  

    case CO_Index_CONTROL_MODE:			/*	Control MODE	*/
      {
        bRetVal = (int32_t)MCI_GetControlMode(pMCI);
      }
      break;	 
    case CO_Index_Fault_FLAGS:			/*	Status Word	*/  
      {
        bRetVal = (int32_t)STM_GetFaultState(pMCT->pStateMachine);
      }
      break;	
    case CO_Index_SPEED_MEAS:			/*	current speed	*/  
      {
        bRetVal = (int32_t)(MCI_GetAvrgMecSpeed01Hz(pMCI) * 6);
      }
      break;
    case CO_Index_TORQUE_MEAS:
      {
        bRetVal = MCI_GetIqd(pMCI).qI_Component1;
      }
      break;
    case CO_Index_MEAS_EL_ANGLE:
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
	  
    case CO_Index_MEAS_ROT_SPEED:
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
	  

    case CO_Index_RAMP_FINAL_SPEED:		/*目标速度*/
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

  default:
    break;
  }
  return bRetVal;
}


/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
