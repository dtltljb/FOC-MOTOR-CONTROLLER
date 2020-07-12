/**
  ******************************************************************************
  * @file    motor_control_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the motor_control_protocol component of the Motor Control SDK.
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

#include "uart1_motor_control_protocol.h"
#include "user_interface.h"


/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup UILib
 * @{
 */

/**
 * @addtogroup MCUI
 * @{
 */

/**
 * @defgroup motor_control_protocol Motor Control Protocol
 *
 * @brief Transmission protocol designed to report Motor Control subsystem status and to control motors.
 *
 * The Motor Control Protocol defines a transmission mechanism that allows a Motor Control
 * Application to send in real time the values of a defined set of internal variables to an
 * external recipient and to receive Motor Control commands from this recipient.
 *
 * The Commands that can be sent with the Motor Control Protocol are defined in **TBD**.
 *
 * The variables which values can be exchanged through the Motor Control Protocol are listed in
 * @ref MC_Protocol_REG.
 *
 * An example of external recipient is the Motor Control Monitor tool that is part of the Motor
 * Control Workbench.
 *
 * The Motor Control Protocol defines frames that contain either commands or variables values.
 * These frames are the exchanged between the Application and the external recipient. To that end,
 * the Motor Control Protocol relies on a lower level transport protocol to actually send them.
 *
 * @todo Complete documentation
 * @{
 */

/* Private define ------------------------------------------------------------*/

#define U1_ACK_NOERROR 0xF0
#define U1_ACK_ERROR   0xFF
#define U1_ATR_FRAME_START 0xE0

#define MB_PROTOCOL_CODE_NONE        0x00

/* List of error codes */
typedef enum ERROR_CODE_e
{
	MB_ERROR_NONE = 0,             /**<  0x00 - No error */
	MB_ERROR_BAD_FRAME_ID,         /**<  0x01 - BAD Frame ID. The Frame ID has not been recognized by the firmware. */
	MB_ERROR_CODE_SET_READ_ONLY,   /**<  0x02 - Write on read-only. The master wants to write on a read-only register. */
	MB_ERROR_CODE_GET_WRITE_ONLY,  /**<  0x03 - Read not allowed. The value cannot be read. */
	MB_ERROR_CODE_NO_TARGET_DRIVE, /**<  0x04 - Bad target drive. The target motor is not supported by the firmware. */
	MB_ERROR_CODE_WRONG_SET,       /**<  0x05 - Value used in the frame is out of range expected by the FW. */
	MB_ERROR_CODE_CMD_ID,          /**<  0x06 - NOT USED */
	MB_ERROR_CODE_WRONG_CMD,       /**<  0x07 - Bad command ID. The command ID has not been recognized. */
	MB_ERROR_CODE_OVERRUN,         /**<  0x08 - Overrun error. Transmission speed too fast, frame not received correctly */
	MB_ERROR_CODE_TIMEOUT,         /**<  0x09 - Timeout error. Received frame corrupted or unrecognized by the FW. */
	MB_ERROR_CODE_BAD_CRC,         /**<  0x0A - The computed CRC is not equal to the received CRC byte. */
	MB_ERROR_BAD_MOTOR_SELECTED,   /**<  0x0B - Bad target drive. The target motor is not supported by the firmware. */
	MB_ERROR_MP_NOT_ENABLED        /**<  0x0C - Motor Profiler not enabled. */
} ERROR_CODE;

MPInfo_t MPInfo = {0, 0};

/**
* @brief  Initializes  MCP component parameters
*
* @param  pHandle Pointer on the handle of the component to initialize.
* @param  pFCP Pointer on Frame communication protocol component's handle to use.
* @param  fFcpSend Pointer on FCP's send message function
* @param  fFcpReceive Pointer on FCP's receive message function
* @param  fFcpAbortReceive Pointer on FCP's abort receive message function
* @param  pDAC Pointer on DAC component.
* @param  s_fwVer Pointer on string containing FW release version.
*/
void U1MCP_Init( MCP_Handle_t *pHandle, 
               FCLP_Handle_t * pFCP,
               FCLP_SendFct_t fFcpSend, 
               FCLP_ReceiveFct_t fFcpReceive, 
               FCLP_AbortReceiveFct_t fFcpAbortReceive, 
               DAC_UI_Handle_t * pDAC, 
               const char* s_fwVer )
{
  pHandle->pFCP = pFCP;
  pHandle->pDAC = pDAC;
  pHandle->s_fwVer = s_fwVer;
  
  FCLP_SetClient( pFCP, pHandle,
                 (FCLP_SentFrameCallback_t) & MCP_SentFrame,
                 (FCLP_ReceivedFrameCallback_t) & MCP_ReceivedFrame,
                 (FCLP_RxTimeoutCallback_t) & MCP_OnTimeOut );
                 
  pHandle->fFcpSend = fFcpSend;
  pHandle->fFcpReceive = fFcpReceive;
  pHandle->fFcpAbortReceive = fFcpAbortReceive;

  U1MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to set and report the Time Out.
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_OnTimeOut(MCP_Handle_t *pHandle)
{
     U1MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to check next reception frame
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_WaitNextFrame(MCP_Handle_t *pHandle)
{
  pHandle->fFcpAbortReceive(pHandle->pFCP);
  pHandle->BufferSize = FCLP_MAX_PAYLOAD_SIZE;
  pHandle->fFcpReceive(pHandle->pFCP);
}

/**
* @brief  Function used to transmit the data
*
* @param  pHandle Pointer on the handle of the component.
* @param  Code code value of frame to send.
* @param  buffer frame data buffer.
* @param  Size size of data frame.
*/
void U1MCP_SentFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
    U1MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to decode received data
*
* @param  pHandle Pointer on the handle of the component.
* @param  Code code value of frame to send.
* @param  buffer frame data buffer.
* @param  Size size of data frame.
*/
void U1MCP_ReceivedFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
  bool RequireAck = true;
  bool bNoError = false; // Default is error
  uint8_t bErrorCode;

  /* Protocol version >3.3 motor selection inside Frame ID */
  uint8_t bMotorSelection = (Code & 0xE0) >> 5; /* Mask: 1110|0000 */
  if (bMotorSelection != 0)
  {
    if (U1UI_SetReg(&pHandle->_Super, MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1))
    {
      Code &= 0x1F; /* Mask: 0001|1111 */

      /* Change also the DAC selected motor */
      if (pHandle->pDAC)
      {
        U1UI_SetReg(&pHandle->pDAC->_Super, MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1);
      }
    }
    else
    {
      Code = MB_PROTOCOL_CODE_NONE; /* Error */
      bErrorCode = MB_ERROR_BAD_MOTOR_SELECTED;
    }
  }

  switch (Code)
  {
  case MB_MC_PROTOCOL_CODE_SET_CMD:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = MB_ERROR_CODE_WRONG_SET;

      switch (bRegID)
      {
      case MC_PROTOCOL_REG_TARGET_MOTOR:
        {
          /* Deprecated */
          int32_t wValue = (int32_t)(buffer[1]);

          U1UI_SetReg(&pHandle->pDAC->_Super, bRegID, wValue);		// pDAC  non 
          bNoError = U1UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;
      case MC_PROTOCOL_REG_CONTROL_MODE:
      case MC_PROTOCOL_REG_SC_PP:
        {
          /* 8bit variables */
          bNoError = U1UI_SetReg(&pHandle->_Super, bRegID, (int32_t)(buffer[1]));
        }
        break;

      case MC_PROTOCOL_REG_DAC_OUT1:
        {
          bNoError = true; /* No check inside class return always true*/
        }
        break;

      case MC_PROTOCOL_REG_DAC_OUT2:
        {
          bNoError = true; /* No check inside class return always true*/
        }
        break;

      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_FLUX_REF:
      case MC_PROTOCOL_REG_SPEED_KP:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
      case MC_PROTOCOL_REG_PLL_KI:
      case MC_PROTOCOL_REG_PLL_KP:
      case MC_PROTOCOL_REG_FLUXWK_KP:
      case MC_PROTOCOL_REG_FLUXWK_KI:
      case MC_PROTOCOL_REG_FLUXWK_BUS:
      case MC_PROTOCOL_REG_IQ_SPEEDMODE:
      case MC_PROTOCOL_REG_PFC_DCBUS_REF:
      case MC_PROTOCOL_REG_PFC_I_KP:
      case MC_PROTOCOL_REG_PFC_I_KI:
      case MC_PROTOCOL_REG_PFC_I_KD:
      case MC_PROTOCOL_REG_PFC_V_KP:
      case MC_PROTOCOL_REG_PFC_V_KI:
      case MC_PROTOCOL_REG_PFC_V_KD:
      case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
        {
          /* 16bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8);
          bNoError = U1UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;

      case MC_PROTOCOL_REG_OBSERVER_C1:
      case MC_PROTOCOL_REG_OBSERVER_C2:
      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRENT:
      case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
      case MC_PROTOCOL_REG_SC_LDLQRATIO:
      case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
      case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
      case MC_PROTOCOL_REG_SC_STARTUP_ACC:
        {
          /* 32bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
          bNoError = U1UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;
//------------------control command-------------------------
			  case MC_PROTOCOL_CMD_CONTROL_MOTOR:
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
			      bNoError = true; /* No check inside class return always true*/
			      
			    }
			    break;
			  case MC_PROTOCOL_CMD_CLEAR_FAULT_ACK:
			    {
			      MCI_FaultAcknowledged(pMCI);
			      bNoError = true; /* No check inside class return always true*/
			      
			    }
			    break;
			
			  case MC_PROTOCOL_CMD_CLEAR_IQDREF:
			    {
			      MCI_Clear_Iqdref(pMCI);
			      bNoError = true; /* No check inside class return always true*/
			      
			    }
			    break;
			  case MC_PROTOCOL_CMD_ALIGN_ENCODER:
			    {
			      MCI_EncoderAlign(pMCI);
			      bNoError = true; /* No check inside class return always true*/
			      
			    }
			    break;
      default:
        {
          bErrorCode = MB_ERROR_CODE_SET_READ_ONLY;
        }
        break;
      }
    }
    break;
  case MB_MC_PROTOCOL_CODE_GET_CMD:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = MB_ERROR_CODE_GET_WRITE_ONLY;

      switch (bRegID)
      {
      case MC_PROTOCOL_REG_TARGET_MOTOR:
      case MC_PROTOCOL_REG_STATUS:
      case MC_PROTOCOL_REG_CONTROL_MODE:
      case MC_PROTOCOL_REG_RUC_STAGE_NBR:
      case MC_PROTOCOL_REG_PFC_STATUS:
      case MC_PROTOCOL_REG_PFC_ENABLED:
      case MC_PROTOCOL_REG_SC_CHECK:
      case MC_PROTOCOL_REG_SC_STATE:
      case MC_PROTOCOL_REG_SC_STEPS:
      case MC_PROTOCOL_REG_SC_PP:
      case MC_PROTOCOL_REG_SC_FOC_REP_RATE:
      case MC_PROTOCOL_REG_SC_COMPLETED:
        {
          /* 8bit variables */
          int32_t value = U1UI_GetReg(&pHandle->_Super, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            pHandle->fFcpSend(pHandle->pFCP, U1_ACK_NOERROR, (uint8_t*)(&value), 1);
            bNoError = true;
            RequireAck = false;
          }
        }
        break;

      case MC_PROTOCOL_REG_DAC_OUT1:
        {
        }
        break;

      case MC_PROTOCOL_REG_DAC_OUT2:
        {
        }
        break;

      case MC_PROTOCOL_REG_SPEED_KP:					//property: read and write
      case MC_PROTOCOL_REG_SPEED_KP_DIV:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KI_DIV:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_REF:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
//      case MC_PROTOCOL_REG_OBSERVER_C1:
//      case MC_PROTOCOL_REG_OBSERVER_C2:
      case MC_PROTOCOL_REG_PLL_KP:
      case MC_PROTOCOL_REG_PLL_KI:
      case MC_PROTOCOL_REG_FLUXWK_KP:
      case MC_PROTOCOL_REG_FLUXWK_KI:
      case MC_PROTOCOL_REG_FLUXWK_BUS:
      case MC_PROTOCOL_REG_IQ_SPEEDMODE:
      case MC_PROTOCOL_REG_PFC_DCBUS_REF:
      case MC_PROTOCOL_REG_PFC_I_KP:
      case MC_PROTOCOL_REG_PFC_I_KI:
      case MC_PROTOCOL_REG_PFC_I_KD:
      case MC_PROTOCOL_REG_PFC_V_KP:
      case MC_PROTOCOL_REG_PFC_V_KI:
      case MC_PROTOCOL_REG_PFC_V_KD:
      case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
      	
      case MC_PROTOCOL_REG_OBSERVER_CR_C1:		//property:only read
      case MC_PROTOCOL_REG_OBSERVER_CR_C2:      	
      case MC_PROTOCOL_REG_BUS_VOLTAGE:
      case MC_PROTOCOL_REG_HEATS_TEMP:
      case MC_PROTOCOL_REG_MOTOR_POWER:
      case MC_PROTOCOL_REG_TORQUE_MEAS:
      case MC_PROTOCOL_REG_FLUX_MEAS:
      case MC_PROTOCOL_REG_FLUXWK_BUS_MEAS:
      case MC_PROTOCOL_REG_FF_VQ:
      case MC_PROTOCOL_REG_FF_VD:
      case MC_PROTOCOL_REG_FF_VQ_PIOUT:
      case MC_PROTOCOL_REG_FF_VD_PIOUT:
      case MC_PROTOCOL_REG_PFC_DCBUS_MEAS:
      case MC_PROTOCOL_REG_PFC_ACBUS_FREQ:
      case MC_PROTOCOL_REG_PFC_ACBUS_RMS:
      case MC_PROTOCOL_REG_HFI_EL_ANGLE:
      case MC_PROTOCOL_REG_HFI_ROT_SPEED:
      case MC_PROTOCOL_REG_HFI_CURRENT:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_PLL:
      case MC_PROTOCOL_REG_CTRBDID:
      case MC_PROTOCOL_REG_PWBDID:
        {
          int32_t value = U1UI_GetReg(&pHandle->_Super, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            /* 16bit variables */
            pHandle->fFcpSend(pHandle->pFCP, U1_ACK_NOERROR, (uint8_t*)(&value), 2);
            bNoError = true;
            RequireAck = false;
          }
        }
        break;

      case MC_PROTOCOL_REG_OBSERVER_C1:						//property: w / r 
      case MC_PROTOCOL_REG_OBSERVER_C2:      	
      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
      case MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED:
      case MC_PROTOCOL_REG_SC_CURRENT:
      case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
      case MC_PROTOCOL_REG_SC_LDLQRATIO:
      case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
      case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
      case MC_PROTOCOL_REG_SC_STARTUP_ACC:
      	
      case MC_PROTOCOL_REG_PFC_FAULTS:      			//property: only read  !!!!
      case MC_PROTOCOL_REG_SC_RS:
      case MC_PROTOCOL_REG_SC_LS:
      case MC_PROTOCOL_REG_SC_KE:
      case MC_PROTOCOL_REG_SC_VBUS:  
      case MC_PROTOCOL_REG_SC_J:
      case MC_PROTOCOL_REG_SC_F:
      case MC_PROTOCOL_REG_SC_MAX_CURRENT:   	      	      	    	
      case MC_PROTOCOL_REG_SC_PWM_FREQUENCY:
      case MC_PROTOCOL_REG_FLAGS:									
      case MC_PROTOCOL_REG_SPEED_REF:
      case MC_PROTOCOL_REG_SPEED_MEAS:      	
      case MC_PROTOCOL_REG_UID:

        {
          int32_t value = U1UI_GetReg(&pHandle->_Super, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            /* 32bit variables */
            pHandle->fFcpSend(pHandle->pFCP, U1_ACK_NOERROR, (uint8_t*)(&value), 4);
            bNoError = true;
            RequireAck = false;
          }
        }
        break;

      default:
        bErrorCode = MB_ERROR_CODE_GET_WRITE_ONLY;
        break;
      }
    }
    break;

  default:
    {
      bErrorCode = MB_ERROR_BAD_FRAME_ID;
    }
    break;
  }
  
  if (RequireAck)
  {
    if (bNoError)
    {
      pHandle->fFcpSend(pHandle->pFCP, U1_ACK_NOERROR, MC_NULL, 0);
    }
    else
    {
      pHandle->fFcpSend(pHandle->pFCP, U1_ACK_ERROR, &bErrorCode, 1);
    }
  }

}

/**
* @brief  Allow to report the overrun error message.
*
* Called when received frame has not been received correctly due to the
* transmission speed too fast.
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_SendOverrunMessage(MCP_Handle_t *pHandle)
{
  uint8_t bErrorCode = MB_ERROR_CODE_OVERRUN;
  pHandle->fFcpSend(pHandle->pFCP, U1_ACK_ERROR, &bErrorCode, 1);
}

/**
* @brief  Allow to report the time out error message.
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_SendTimeoutMessage(MCP_Handle_t *pHandle)
{
  uint8_t bErrorCode = MB_ERROR_CODE_TIMEOUT;
  pHandle->fFcpSend(pHandle->pFCP, U1_ACK_ERROR, &bErrorCode, 1);
}

/**
* @brief  Allow to send an ATR message.
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_SendATRMessage(MCP_Handle_t *pHandle)
{
  uint32_t wUID = U1UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_UID);
  unsigned char i;
  uint8_t bFWX;
  uint8_t bFWY;
  uint8_t bFWZ;
  uint16_t hCBDID = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_CTRBDID);
  uint16_t hPBDID = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_PWBDID);
  uint16_t hPBDID2 = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_PWBDID2);
  uint8_t buff[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};

  for (i = 0; (i<29) && (pHandle->s_fwVer[i]!=0); i++);

  bFWX = pHandle->s_fwVer[i+5];
  bFWY = pHandle->s_fwVer[i+7];
  bFWZ = pHandle->s_fwVer[i+9];

  *(uint32_t*)(&buff[0]) = wUID;
  buff[4] = bFWX;
  buff[5] = bFWY;
  buff[6] = bFWZ;
  buff[7] = (uint8_t)(hCBDID);
  buff[8] = (uint8_t)(hCBDID>>8);
  buff[9] = (uint8_t)(hPBDID);
  buff[10]= (uint8_t)(hPBDID>>8);
  buff[11] = (uint8_t)(hPBDID2);
  buff[12]= (uint8_t)(hPBDID2>>8);

  pHandle->fFcpSend(pHandle->pFCP, U1_ATR_FRAME_START, buff, 13);
}

/**
* @brief  Allow to send back a BAD CRC message.
*
* @param  pHandle Pointer on the handle of the component.
*/
void U1MCP_SendBadCRCMessage(MCP_Handle_t *pHandle)
{
  uint8_t bErrorCode = MB_ERROR_CODE_BAD_CRC;
  pHandle->fFcpSend(pHandle->pFCP, U1_ACK_ERROR, &bErrorCode, 1);
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

/**
 * @}
 */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
