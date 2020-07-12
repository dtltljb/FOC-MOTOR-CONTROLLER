/**
  ******************************************************************************
  * @file    lcd_vintage_ui.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the implementation of the LCDVintage module.
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
#include "user_interface.h"
#include "lcd_vintage_ui.h"
#include "mc_type.h"
#include "Timebase.h"
#include "parameters_conversion.h"
#include "feed_forward_ctrl.h"
#include "flux_weakening_ctrl.h"
#include "bus_voltage_sensor.h"
#if (defined(USE_STM32303C_EVAL) || defined (P_NUCLEO_IHM001))
  #include "stm32f30x.h"
  #include "stm32303c_eval.h"
  #include "stm32303c_eval_lcd.h"
#else
  #include "stm32_eval.h"
#endif

/** @addtogroup MCSDK
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

/** @defgroup UserInterface_LCDVintage LCD "Vintage" User Interface
 *
 * @brief TBD.
 *
 * @todo TODO: LCD "Vintage" User Interface module Documentation
 * @{
 */

/* Constants --------------------------------------------------------*/
#define CONTROL_MODE_SPEED_MENU  (uint8_t) 0
#define REF_SPEED_MENU           (uint8_t) 1
#define MOTOR_SPD_MENU           (uint8_t) 26
#define MOTOR_TRQ_MENU           (uint8_t) 27

#define P_SPEED_MENU             (uint8_t) 2
#define I_SPEED_MENU             (uint8_t) 3
#define D_SPEED_MENU             (uint8_t) 4

#define P_TORQUE_MENU            (uint8_t) 5
#define I_TORQUE_MENU            (uint8_t) 6
#define D_TORQUE_MENU            (uint8_t) 7

#define P_FLUX_MENU              (uint8_t) 8
#define I_FLUX_MENU              (uint8_t) 9
#define D_FLUX_MENU              (uint8_t) 10

#define POWER_STAGE_MENU         (uint8_t) 11

#define CONTROL_MODE_TORQUE_MENU (uint8_t) 12
#define IQ_REF_MENU              (uint8_t) 13
#define ID_REF_MENU              (uint8_t) 14

#define FAULT_MENU               (uint8_t) 15

#define WAIT_MENU                (uint8_t) 16

#ifdef OBSERVER_GAIN_TUNING
#define K1_MENU				   (uint8_t) 17
#define K2_MENU				   (uint8_t) 18
#define P_PLL_MENU			   (uint8_t) 19
#define I_PLL_MENU			   (uint8_t) 20
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
#define DAC_PB0_MENU		           (uint8_t) 21
#define DAC_PB1_MENU                       (uint8_t) 22
#endif

#ifdef FLUX_WEAKENING
#define P_VOLT_MENU             (uint8_t)23
#define I_VOLT_MENU             (uint8_t)24
#define TARGET_VOLT_MENU        (uint8_t)25
#endif

#define BLINKING_TIME   5  /* 5 * timebase_display_5 ms */

#define VISUALIZATION_1   (uint8_t)1
#define VISUALIZATION_2   (uint8_t)2
#define VISUALIZATION_3   (uint8_t)3
#define VISUALIZATION_4   (uint8_t)4
#define VISUALIZATION_5   (uint8_t)5
#define VISUALIZATION_6   (uint8_t)6
#define VISUALIZATION_7   (uint8_t)7
#define VISUALIZATION_8   (uint8_t)8
#define VISUALIZATION_9   (uint8_t)9
#define VISUALIZATION_10  (uint8_t)10
#ifdef FLUX_WEAKENING
#define VISUALIZATION_11  (uint8_t)11
#endif
#define CHAR_0            (uint8_t)0 /*First character of the line starting from the left */
#define CHAR_1            (uint8_t)1
#define CHAR_2            (uint8_t)2
#define CHAR_3            (uint8_t)3
#define CHAR_4            (uint8_t)4
#define CHAR_5            (uint8_t)5
#define CHAR_6            (uint8_t)6
#define CHAR_7            (uint8_t)7
#define CHAR_8            (uint8_t)8
#define CHAR_9            (uint8_t)9
#define CHAR_10           (uint8_t)10
#define CHAR_11           (uint8_t)11
#define CHAR_12           (uint8_t)12
#define CHAR_13           (uint8_t)13
#define CHAR_14           (uint8_t)14
#define CHAR_15           (uint8_t)15
#define CHAR_16           (uint8_t)16
#define CHAR_17           (uint8_t)17

#ifdef OBSERVER_GAIN_TUNING
#define CHAR_18           (uint8_t)18
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
#define CHAR_19           (uint8_t)19
#endif

/* Key ------------------------------------------------------------*/
#define  NOKEY      (uint8_t)0
#define  SEL        (uint8_t)1
#define  RIGHT      (uint8_t)2
#define  LEFT       (uint8_t)3
#define  UP         (uint8_t)4
#define  DOWN       (uint8_t)5
#define  KEY_HOLD   (uint8_t)6

#define KEY_UP_PORT UP_BUTTON_GPIO_PORT
#define KEY_UP_BIT  UP_BUTTON_PIN

#define KEY_DOWN_PORT DOWN_BUTTON_GPIO_PORT
#define KEY_DOWN_BIT  DOWN_BUTTON_PIN

#define KEY_RIGHT_PORT RIGHT_BUTTON_GPIO_PORT
#define KEY_RIGHT_BIT  RIGHT_BUTTON_PIN

#define KEY_LEFT_PORT LEFT_BUTTON_GPIO_PORT
#define KEY_LEFT_BIT  LEFT_BUTTON_PIN

#define KEY_SEL_PORT SEL_BUTTON_GPIO_PORT
#define KEY_SEL_BIT  SEL_BUTTON_PIN

#define USER_BUTTON_PORT KEY_BUTTON_GPIO_PORT
#define USER_BUTTON_BIT  KEY_BUTTON_PIN

#define  SEL_FLAG        (uint8_t)0x02
#define  RIGHT_FLAG      (uint8_t)0x04
#define  LEFT_FLAG       (uint8_t)0x08
#define  UP_FLAG         (uint8_t)0x10
#define  DOWN_FLAG       (uint8_t)0x20

/* Variable increment and decrement */
#define SPEED_INC_DEC     (uint16_t)10
#define SPEED_INC_DEC_DURATION (uint16_t)100
#define KP_GAIN_INC_DEC   (uint16_t)250
#define KI_GAIN_INC_DEC   (uint16_t)25
#define KD_GAIN_INC_DEC   (uint16_t)100

#ifdef FLUX_WEAKENING
#define KP_VOLT_INC_DEC   (uint8_t)50
#define KI_VOLT_INC_DEC   (uint8_t)10
#define VOLT_LIM_INC_DEC  (uint8_t)5
#endif

#define TORQUE_INC_DEC    (uint16_t)250
#define FLUX_INC_DEC      (uint16_t)250

#define K1_INC_DEC        (int16_t)(250)
#define K2_INC_DEC        (int16_t)(5000)

#define PLL_IN_DEC        (uint16_t)(25)

/* Private typedef */
typedef enum InputKey_e
{
  KEY_NONE           = 0,
  KEY_JOYSTICK_UP    = 1,
  KEY_JOYSTICK_DOWN  = 2,
  KEY_JOYSTICK_LEFT  = 3,
  KEY_JOYSTICK_RIGHT = 4,
  KEY_JOYSTICK_SEL   = 5,
  KEY_USER_BUTTON    = 6
} InputKey;

/* Private variables ---------------------------------------------------------*/
volatile static uint16_t hTimebase_Blinking;
static uint8_t bPrevious_Visualization = 0;
static uint8_t bPresent_Visualization;
static uint8_t bMenu_index;
static uint8_t bKey;
static uint8_t bPrevious_key;
static uint8_t bKey_Flag;


static void Display_5DigitSignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number);
static void Display_3DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number);
static void Display_1DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, uint8_t number);
static void Display_5dot_line(uint16_t Line, uint16_t Column);
static void DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
static bool JOY_Pressed(InputKey inKey);
static uint8_t ComputeVisualization(uint8_t bLocal_MenuIndex, State_t State);
static uint8_t LCDV_DACIDToSel(MC_Protocol_REG_t ID);
void LCDV_DACSelUpdate(UI_Handle_t *pHandle, int8_t bInc, DAC_Channel_t bCh);

#define GUI_DAC_ChID_LAST_ELEMENT 0xFF

const char * const GUI_DAC_ChTxt[] = {
  "Ia               ","Ib               ",
  "Ialpha           ","Ibeta            ","Iq               ",
  "Id               ","Iq ref           ","Id ref           ",
  "Vq               ","Vd               ","Valpha           ",
  "Vbeta            ","Meas. El Angle   ","Meas. Rotor Speed",
  "HFI El Angle     ","HFI Rotor Speed  ","HFI debug current","HFI debug angle  ",
  "Obs. El Ang.(PLL)","Obs. Rot.Spd(PLL)","Obs. Ialpha (PLL)",
  "Obs. Ibeta  (PLL)","Obs. Bemf a.(PLL)","Obs. Bemf b.(PLL)",
  "Exp. Bemf l.(PLL)","Obs. Bemf l.(PLL)",
  "Obs. El Ang. (CR)","Obs. Rot.Spd (CR)","Obs. Ialpha  (CR)",
  "Obs. Ibeta   (CR)","Obs. Bemf a. (CR)","Obs. Bemf b. (CR)",
  "Exp. Bemf l. (CR)","Obs. Bemf l. (CR)",
  "User 1           ","User 2           ", MC_NULL};

const MC_Protocol_REG_t GUI_DAC_ChID[] = {
  MC_PROTOCOL_REG_I_A,
  MC_PROTOCOL_REG_I_B,
  MC_PROTOCOL_REG_I_ALPHA,
  MC_PROTOCOL_REG_I_BETA,
  MC_PROTOCOL_REG_I_Q,
  MC_PROTOCOL_REG_I_D,
  MC_PROTOCOL_REG_I_Q_REF,
  MC_PROTOCOL_REG_I_D_REF,
  MC_PROTOCOL_REG_V_Q,
  MC_PROTOCOL_REG_V_D,
  MC_PROTOCOL_REG_V_ALPHA,
  MC_PROTOCOL_REG_V_BETA,
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,
  MC_PROTOCOL_REG_HFI_EL_ANGLE,
  MC_PROTOCOL_REG_HFI_ROT_SPEED,
  MC_PROTOCOL_REG_HFI_CURRENT,
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,
  MC_PROTOCOL_REG_OBS_EL_ANGLE,
  MC_PROTOCOL_REG_OBS_ROT_SPEED,
  MC_PROTOCOL_REG_OBS_I_ALPHA,
  MC_PROTOCOL_REG_OBS_I_BETA,
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,
  MC_PROTOCOL_REG_OBS_BEMF_BETA,
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,
  MC_PROTOCOL_REG_OBS_CR_I_BETA,
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,
  MC_PROTOCOL_REG_DAC_USER1,
  MC_PROTOCOL_REG_DAC_USER2,
  (MC_Protocol_REG_t)(GUI_DAC_ChID_LAST_ELEMENT)
};

/**
  * @brief  Initialization of LCD object. It must be called after the UI_Init.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  pDAC related DAC object upcasted to CUI. It can be MC_NULL.
  * @param  s_fwVer String contating firmware version.
  * @retval none.
  */
void LCDV_Init(UI_Handle_t *pHandle, UI_Handle_t *pDAC, const char* s_fwVer)
{

  LCDV_Handle_t *pHdl = (LCDV_Handle_t *)pHandle;

  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  SpeednTorqCtrl_Handle_t *pSTC = pMCT->pSpeednTorqueCtrl;
	uint8_t *ptr;

  /* Initialize the LCD */
  LCD_HW_Init();

  LCD_Clear(White);

  LCD_SetBackColor(White);
  LCD_SetTextColor(Black);

  /* Initialize Joystick */
  STM_EVAL_JOYInit();

  /* Welcome message */
  ptr = " STM32 Motor Control";

  LCD_DisplayStringLine(Line0, ptr);

  ptr = "  PMSM FOC ver 5.0  ";
  LCD_DisplayStringLine(Line1, ptr);

  ptr = " <> Move  ^| Change ";
  LCD_DisplayStringLine(Line9, ptr);

  /* Initialize vars */
  pHdl->pDAC = (DAC_UI_Handle_t *)pDAC;
  pHdl->bDAC_CH0_ID = MC_PROTOCOL_REG_UNDEFINED;
  pHdl->bDAC_CH1_ID = MC_PROTOCOL_REG_UNDEFINED;
  pHdl->bDAC_CH0_Sel = 0;
  pHdl->bDAC_CH1_Sel = 0;
  pHdl->bDAC_Size = LCDV_DACIDToSel((MC_Protocol_REG_t)(GUI_DAC_ChID_LAST_ELEMENT));

  pHdl->Iqdref = STC_GetDefaultIqdref(pSTC);
}

/**
  * @brief  Execute the LCD execution and refreshing. It must be called
  *         periodically.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCDV_Exec(UI_Handle_t *pHandle,  UI_Handle_t *pDAC)
{

  LCDV_Handle_t *pHdl = (LCDV_Handle_t *)pHandle;
  DAC_UI_Handle_t *pHdlDAC = (DAC_UI_Handle_t *)pDAC;

  uint8_t bSel,bSize = pHdl->bDAC_Size;
  MC_Protocol_REG_t ID;

  Display_LCD(pHandle);
  KEYS_process(pHandle);

  /* Update selected DAC variable */
  ID = UI_GetDAC(&pHdlDAC->_Super, DAC_CH0);
  if (pHdl->bDAC_CH0_ID != ID)
  {
    pHdl->bDAC_CH0_ID = ID;
    bSel = LCDV_DACIDToSel(ID);
    if (bSel < bSize)
    {
      pHdl->bDAC_CH0_Sel = bSel;
    }
  }
  ID = UI_GetDAC(&pHdlDAC->_Super, DAC_CH1);
  if (pHdl->bDAC_CH1_ID != ID)
  {
    pHdl->bDAC_CH1_ID = ID;
    bSel = LCDV_DACIDToSel(ID);
    if (bSel < bSize)
    {
      pHdl->bDAC_CH1_Sel = bSel;
    }
  }
}

/**
  * @brief  It is used to force a refresh of all LCD values.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCDV_UpdateAll(UI_Handle_t *pHandle)
{
}

/**
  * @brief  It is used to force a refresh of only measured LCD values.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCDV_UpdateMeasured(UI_Handle_t *pHandle)
{
}

static uint8_t LCDV_DACIDToSel(MC_Protocol_REG_t ID)
{
  uint8_t retVal;
  for (retVal = 0; retVal < GUI_DAC_ChID_LAST_ELEMENT; retVal++)
  {
    if ((GUI_DAC_ChID[retVal] == ID) ||
        (GUI_DAC_ChID[retVal] == GUI_DAC_ChID_LAST_ELEMENT) ||
        (retVal == 0xFF))
    {
      break;
    }
  }
  return retVal;
}

void LCDV_DACSelUpdate(UI_Handle_t *pHandle, int8_t bInc, DAC_Channel_t bCh)
{

  LCDV_Handle_t *pHdl = (LCDV_Handle_t *)pHandle;

  uint8_t bSize= pHdl->bDAC_Size;
  int8_t bSel;

  if (bCh == DAC_CH0)
  {
    bSel = (int8_t)(pHdl->bDAC_CH0_Sel);
  }
  else if (bCh == DAC_CH1)
  {
    bSel = (int8_t)(pHdl->bDAC_CH1_Sel);
  }

  bSel += bInc;

  if (bSel < 0)
  {
    bSel += bSize;
  }
  if (bSel >= bSize)
  {
    bSel -= bSize;
  }

  if (bCh == DAC_CH0)
  {
    pHdl->bDAC_CH0_Sel = (uint8_t)(bSel);
  }
  else if (bCh == DAC_CH1)
  {
    pHdl->bDAC_CH1_Sel = (uint8_t)(bSel);
  }

  UI_SetDAC(pHandle, bCh, GUI_DAC_ChID[bSel]);
}

/*******************************************************************************
* Function Name  : Display_LCD
* Description    : Display routine for LCD management
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display_LCD(UI_Handle_t *pHandle)
{
  LCDV_Handle_t *pHdl = (LCDV_Handle_t *)pHandle;

  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
  PID_Handle_t* pSpeedLoopPID = pMCT->pPIDSpeed;
  PID_Handle_t* pIqLoopPID = pMCT->pPIDIq;
  PID_Handle_t* pIdLoopPID = pMCT->pPIDId;
  PID_Handle_t* pFluxWeakeningLoopPID = pMCT->pPIDFluxWeakening;
  FW_Handle_t* pFWCtrl = pMCT->pFW;
  BusVoltageSensor_Handle_t *pVBS = pMCT->pBusVoltageSensor;
  NTC_Handle_t *pTNC = pMCT->pTemperatureSensor;
  State_t State = MCI_GetSTMState(pMCI);
  uint16_t hCurrentFault = MCI_GetCurrentFaults(pMCI);
  uint16_t hOccurredFault = MCI_GetOccurredFaults(pMCI);

  bPrevious_Visualization = bPresent_Visualization;

  if (((bMenu_index == CONTROL_MODE_SPEED_MENU) ||
       (bMenu_index == CONTROL_MODE_TORQUE_MENU)) &&
      (State != START))
  {
    if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
    {
      bMenu_index = CONTROL_MODE_SPEED_MENU;
    }
    else
    {
      bMenu_index = CONTROL_MODE_TORQUE_MENU;
    }
  }

  bPresent_Visualization = ComputeVisualization(bMenu_index, State);

  switch(bPresent_Visualization)
  {
    uint8_t *ptr;
    int16_t temp;

  case VISUALIZATION_1:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_ClearLine(Line3);

      LCD_ClearLine(Line4);

      ptr = " Target     Measured";
      LCD_DisplayStringLine(Line5,ptr);

      ptr = "       (rpm)        ";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line6);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    if (bMenu_index == MOTOR_SPD_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }

    ptr = "      Motor ";
    LCD_DisplayStringLine(Line2,ptr);

    temp = pHandle->bSelectedDrive;
    Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */

    if(bMenu_index == CONTROL_MODE_SPEED_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }

    ptr = " Speed control mode";
    LCD_DisplayStringLine(Line3,ptr);

    if(bMenu_index == REF_SPEED_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }

    /* Compute target speed in rpm */
    temp = (int16_t)(MCI_GetLastRampFinalSpeed(pMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_0, temp);

    LCD_SetTextColor(Blue);

    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(pMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_13, temp);

    break;

  case VISUALIZATION_2:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      ptr = "       Speed        ";
      LCD_DisplayStringLine(Line2,ptr);

      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);

      ptr = " Target        (rpm)";
      LCD_DisplayStringLine(Line6,ptr);

      ptr = " Measured      (rpm)";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case P_SPEED_MENU:
      LCD_SetTextColor(Red);
      temp = PID_GetKP(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);

      temp = PID_GetKI(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif

      break;

    case I_SPEED_MENU:
      temp = PID_GetKP(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKI(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_SPEED_MENU:
      temp = PID_GetKP(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      temp = PID_GetKI(pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)pSpeedLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);

      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current
    //and measured speeds

    /* Display target speed in rpm */
    temp = (int16_t)(MCI_GetLastRampFinalSpeed(pMCI) * 6);
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);

    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(pMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);
    break;

  case VISUALIZATION_3:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      ptr = "       Torque       ";
      LCD_DisplayStringLine(Line2,ptr);

      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);

      ptr = " Target         (Iq)";
      LCD_DisplayStringLine(Line6,ptr);

      ptr = " Measured       (Iq)";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case P_TORQUE_MENU:
      LCD_SetTextColor(Red);
      temp = PID_GetKP(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);

      temp = PID_GetKI(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif
      break;

    case I_TORQUE_MENU:
      temp = PID_GetKP(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKI(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_TORQUE_MENU:
      temp = PID_GetKP(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      temp = PID_GetKI(pIqLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)pIqLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);

      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current
    //and measured Iq

    temp = MCI_GetIqdref(pMCI).qI_Component1;
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);

    temp = MCI_GetIqd(pMCI).qI_Component1;
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);
    break;

  case VISUALIZATION_4:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      ptr = "        Flux        ";
      LCD_DisplayStringLine(Line2,ptr);

      ptr = "    P     I     D   ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);

      ptr = " Target         (Id)";
      LCD_DisplayStringLine(Line6,ptr);

      ptr = " Measured       (Id)";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case P_FLUX_MENU:
      LCD_SetTextColor(Red);
      temp = PID_GetKP(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      LCD_SetTextColor(Blue);

      temp = PID_GetKI(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif
      break;

    case I_FLUX_MENU:
      temp = PID_GetKP(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKI(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);

#ifdef DIFFERENTIAL_TERM_ENABLED
      temp = PID_GetKD((CPID_PI)pIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else
      Display_5dot_line(Line4, 18);
#endif
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_FLUX_MENU:
      temp = PID_GetKP(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      temp = PID_GetKI(pIdLoopPID);;
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKD((CPID_PI)pIdLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_13, temp);
      LCD_SetTextColor(Blue);

      break;
#endif
    default:
      break;
    }
    //Independently from the menu, this visualization must display current
    //and measured Id

    temp = MCI_GetIqdref(pMCI).qI_Component2;
    Display_5DigitSignedNumber(Line6, CHAR_9, temp);

    temp = MCI_GetIqd(pMCI).qI_Component2;
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);
    break;

#ifdef FLUX_WEAKENING
  case VISUALIZATION_11:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      ptr = "Flux Weakening Ctrl ";
      LCD_DisplayStringLine(Line2,ptr);

      ptr = "    P     I         ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);
      LCD_ClearLine(Line5);

      ptr = " Target        (Vs%)";
      LCD_DisplayStringLine(Line6,ptr);

      ptr = " Measured      (Vs%)";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case P_VOLT_MENU:
      LCD_SetTextColor(Red);
      temp = PID_GetKP(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);

      LCD_SetTextColor(Blue);
      temp = PID_GetKI(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      temp = FW_GetVref(pFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);
      LCD_DrawRect(161,97,1,2);

      Display_5dot_line(Line4, 18);

      break;

    case I_VOLT_MENU:
      temp = PID_GetKP(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      temp = FW_GetVref(pFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);

      LCD_SetTextColor(Red);
      temp = PID_GetKI(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);
      LCD_SetTextColor(Blue);
      LCD_DrawRect(161,97,1,2);

      Display_5dot_line(Line4, 18);

      break;

    case TARGET_VOLT_MENU:
      LCD_SetTextColor(Red);
      temp = FW_GetVref(pFWCtrl);
      Display_5DigitSignedNumber(Line6, CHAR_9, temp);
      LCD_DrawRect(161,97,1,2);

      LCD_SetTextColor(Blue);
      temp = PID_GetKP(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_1, temp);
      temp = PID_GetKI(pFluxWeakeningLoopPID);
      Display_5DigitSignedNumber(Line4, CHAR_7, temp);

      Display_5dot_line(Line4, 18);

      break;

    default:
      break;
    }
    //Independently from the menu, this visualization must display current
    //and measured voltage level

    //Compute applied voltage in int16_t
    temp = FW_GetAvVPercentage(pFWCtrl);
    Display_5DigitSignedNumber(Line7, CHAR_9, temp);
    LCD_DrawRect(185,97,1,2);
    break;
#endif

  case VISUALIZATION_5:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_ClearLine(Line2);

      ptr = " Power Stage Status ";
      LCD_DisplayStringLine(Line3, ptr);

      LCD_ClearLine(Line4);

      ptr = "  DC bus =     Volt ";
      LCD_DisplayStringLine(Line5, ptr);

      LCD_ClearLine(Line6);

      ptr = "  T =      Celsius  ";
      LCD_DisplayStringLine(Line7, ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move            ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    temp = VBS_GetAvBusVoltage_V(pVBS);
    Display_3DigitUnsignedNumber(Line5, CHAR_11, temp);

    temp = NTC_GetAvTemp_C(pTNC);
    Display_3DigitUnsignedNumber(Line7, CHAR_6, temp);

    break;

  case VISUALIZATION_6:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_ClearLine(Line3);

      ptr = "     Target Measured";
      LCD_DisplayStringLine(Line4,ptr);

      ptr = "Iq                  ";
      LCD_DisplayStringLine(Line5,ptr);

      ptr = "Id                  ";
      LCD_DisplayStringLine(Line6,ptr);

      ptr = "Speed (rpm)         ";
      LCD_DisplayStringLine(Line7,ptr);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }
    if (bMenu_index == MOTOR_TRQ_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }

    ptr = "      Motor ";
    LCD_DisplayStringLine(Line2,ptr);

    temp = pHandle->bSelectedDrive;
    Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */

    if (bMenu_index == CONTROL_MODE_TORQUE_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    ptr = "Torque control mode ";
    LCD_DisplayStringLine(Line3,ptr);

    if (bMenu_index == IQ_REF_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    temp = pHdl->Iqdref.qI_Component1;
    Display_5DigitSignedNumber(Line5, CHAR_5, temp);

    if (bMenu_index == ID_REF_MENU)
    {
      LCD_SetTextColor(Red);
    }
    else
    {
      LCD_SetTextColor(Blue);
    }
    temp = pHdl->Iqdref.qI_Component2;
    Display_5DigitSignedNumber(Line6, CHAR_5, temp);

    LCD_SetTextColor(Blue);

    temp = MCI_GetIqd(pMCI).qI_Component1;
    Display_5DigitSignedNumber(Line5, CHAR_13, temp);

    temp = MCI_GetIqd(pMCI).qI_Component2;
    Display_5DigitSignedNumber(Line6, CHAR_13, temp);

    /* Compute measured speed in rpm */
    temp = (int16_t)(MCI_GetAvrgMecSpeed01Hz(pMCI) * 6);
    Display_5DigitSignedNumber(Line7, CHAR_13, temp);
    break;

  case VISUALIZATION_7:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_SetTextColor(Red);

      ptr = "      Motor ";
      LCD_DisplayStringLine(Line2,ptr);

      temp = pHandle->bSelectedDrive;
      Display_1DigitUnsignedNumber(Line2, CHAR_12, temp + 1); /* Zero based */

      ptr = "    !!! FAULT !!!   ";
      LCD_DisplayStringLine(Line3,ptr);
      LCD_SetTextColor(Blue);

      if ( (hOccurredFault & MC_UNDER_VOLT) == MC_UNDER_VOLT)
      {
        ptr = " Bus Under Voltage  ";
        LCD_DisplayStringLine(Line4, ptr);
      }
      else if ( (hOccurredFault & MC_BREAK_IN) ==  MC_BREAK_IN)
      {
        ptr = "   Over Current    ";
        LCD_DisplayStringLine(Line4, ptr);
      }
      else if ( (hOccurredFault & MC_OVER_TEMP) ==  MC_OVER_TEMP)
      {
        ptr = "   Over Heating    ";
        LCD_DisplayStringLine(Line4, ptr);
      }
      else if ( (hOccurredFault & MC_OVER_VOLT) ==  MC_OVER_VOLT)
      {
        ptr = "  Bus Over Voltage  ";
        LCD_DisplayStringLine(Line4, ptr);
      }
      else if ( (hOccurredFault & MC_START_UP) ==  MC_START_UP)
      {
        ptr = "  Start-up failed   ";
        LCD_DisplayStringLine(Line4, ptr);
      }
      else if ( (hOccurredFault & MC_SPEED_FDBK) ==  MC_SPEED_FDBK)
      {
        ptr = "Error on speed fdbck";
        LCD_DisplayStringLine(Line4, ptr);
      }
      LCD_ClearLine(Line5);
      LCD_ClearLine(Line7);
    }

    if ((hCurrentFault & ( MC_OVER_TEMP | MC_UNDER_VOLT | MC_OVER_VOLT)) == 0)
    {
      LCD_ClearLine(Line6);
      ptr = "   Press 'Key' to   ";
      LCD_DisplayStringLine(Line8,ptr);

      ptr = "   return to menu   ";
      LCD_DisplayStringLine(Line9,ptr);
    }
    else
    {
      if ((hCurrentFault & (MC_UNDER_VOLT | MC_OVER_VOLT)) == 0)
      {
        /* Under or over voltage */
        if (bPresent_Visualization != bPrevious_Visualization)
        {
          LCD_ClearLine(Line6);
        }
        temp = NTC_GetAvTemp_C(pTNC);
        ptr = "       T =";
        LCD_DisplayStringLine(Line6, ptr);
        Display_3DigitUnsignedNumber(Line6, CHAR_11, temp);
        DisplayChar(Line6, CHAR_14, ' ');
        DisplayChar(Line6, CHAR_15, 'C');
      }
      else
      {
        if (bPresent_Visualization != bPrevious_Visualization)
        {
          LCD_ClearLine(Line6);
        }
        ptr = "  DC bus =";
        LCD_DisplayStringLine(Line6, ptr);
        temp = VBS_GetAvBusVoltage_V(pVBS);
        Display_3DigitUnsignedNumber(Line6, CHAR_11, temp);
        DisplayChar(Line6, CHAR_14, ' ');
        DisplayChar(Line6, CHAR_15, 'V');
      }
      LCD_ClearLine(Line8);
      LCD_ClearLine(Line9);
    }
    break;

  case VISUALIZATION_8:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_ClearLine(Line2);

      ptr = " Motor is stopping  ";
      LCD_DisplayStringLine(Line3,ptr);

      ptr = "   please wait...   ";
      LCD_DisplayStringLine(Line4,ptr);

      LCD_ClearLine(Line5);
      LCD_ClearLine(Line6);
      LCD_ClearLine(Line7);
      LCD_ClearLine(Line8);
      LCD_ClearLine(Line9);
    }
    break;

#ifdef OBSERVER_GAIN_TUNING
  case VISUALIZATION_9 :
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      ptr = "   Observer Gains   ";
      LCD_DisplayStringLine(Line2,ptr);

      ptr = "     K1       K2    ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);

      ptr = "      PLL Gains     ";
      LCD_DisplayStringLine(Line5,ptr);

      ptr = "     P        I     ";
      LCD_DisplayStringLine(Line6,ptr);

      LCD_ClearLine(Line7);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case K1_MENU:
      LCD_SetTextColor(Red);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);

      LCD_SetTextColor(Blue);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;

    case K2_MENU:
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);

      LCD_SetTextColor(Red);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);

      LCD_SetTextColor(Blue);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;

    case P_PLL_MENU:
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);

      LCD_SetTextColor(Red);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);

      LCD_SetTextColor(Blue);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      break;

    case I_PLL_MENU :
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1);
      Display_5DigitSignedNumber(Line4, CHAR_3, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2);
      Display_5DigitSignedNumber(Line4, CHAR_12, temp);

      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP);
      Display_5DigitSignedNumber(Line7, CHAR_3, temp);

      LCD_SetTextColor(Red);
      temp = UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI);
      Display_5DigitSignedNumber(Line7, CHAR_12, temp);
      LCD_SetTextColor(Blue);
      break;
    default:
      break;
    }
    break;
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
  case VISUALIZATION_10:
    if (bPresent_Visualization != bPrevious_Visualization)
    {
      LCD_ClearLine(Line2);

      ptr = "    Signal on PB0   ";
      LCD_DisplayStringLine(Line3,ptr);

      LCD_ClearLine(Line4);

      LCD_ClearLine(Line5);

      ptr = "    Signal on PB1   ";
      LCD_DisplayStringLine(Line6,ptr);

      LCD_ClearLine(Line7);

      LCD_ClearLine(Line8);

      ptr = " <> Move  ^| Change ";
      LCD_DisplayStringLine(Line9, ptr);
    }

    switch(bMenu_index)
    {
    case DAC_PB0_MENU:
      LCD_SetTextColor(Red);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pHdl->bDAC_CH0_Sel]);
      LCD_DisplayStringLine(Line4, ptr);

      LCD_SetTextColor(Blue);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pHdl->bDAC_CH1_Sel]);
      LCD_DisplayStringLine(Line7, ptr);
      break;

    case DAC_PB1_MENU:
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pHdl->bDAC_CH0_Sel]);
      LCD_DisplayStringLine(Line4, ptr);

      LCD_SetTextColor(Red);
      ptr = (uint8_t*)(GUI_DAC_ChTxt[pHdl->bDAC_CH1_Sel]);
      LCD_DisplayStringLine(Line7, ptr);
      LCD_SetTextColor(Blue);
      break;

    default:
      break;
    }
    break;
#endif
  default:
    break;
  }
}

/*******************************************************************************
* Function Name  : Display_5DigitSignedNumber
* Description    : It Displays a 5 digit signed number in the specified line,
*                  starting from a specified element of LCD display matrix
* Input          : Line, starting point in LCD dysplay matrix, 5 digit signed
*                  number
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_5DigitSignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number)
{
  uint32_t i;
  uint16_t h_aux=1;

  if (number<0)
  {
    DisplayChar(Line,(uint16_t)(bFirstchar), '-');
    number = -number;
  }
  else
  {
    DisplayChar(Line,(uint16_t)(bFirstchar), ' ');
  }

  for (i=0; i<4; i++)
  {
    DisplayChar(Line, (uint16_t)(bFirstchar+5-i),
                                        (uint8_t)(((number%(10*h_aux))/h_aux)+0x30));
    h_aux *= 10;
  }
  DisplayChar(Line,(uint16_t)(bFirstchar+1), (uint8_t)(((number/10000))+0x30));
}

/*******************************************************************************
* Function Name  : Display_3DigitUnsignedNumber
* Description    : It Displays a 3 digit unsigned number in the specified line,
*                  starting from a specified element of LCD display matrix
* Input          : Line, starting point in LCD dysplay matrix, 3 digit unsigned
*                  number
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_3DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, int16_t number)
{
  DisplayChar(Line, (bFirstchar), (uint8_t)(((number%1000)/100)+0x30));
  DisplayChar(Line, (bFirstchar+1), (uint8_t)(((number%100)/10)+0x30));
  DisplayChar(Line, (bFirstchar+2), (uint8_t)((number%10)+0x30));
}

/*******************************************************************************
* Function Name  : Display_1DigitUnsignedNumber
* Description    : It Displays a 1 digit unsigned number in the specified line,
*                  starting from a specified element of LCD display matrix
* Input          : Line, starting point in LCD dysplay matrix, 1 digit unsigned
*                  number
* Output         : None
* Return         : None
*******************************************************************************/
static void Display_1DigitUnsignedNumber(uint8_t Line, uint8_t bFirstchar, uint8_t number)
{
  DisplayChar(Line, bFirstchar, (uint8_t)((number%10)+0x30));
}

static void Display_5dot_line(uint16_t Line, uint16_t Column)
{
  uint8_t i;
  for(i = 0; i < 5; i++)
  {
    DisplayChar(Line, (uint16_t)(Column-i), '-');
  }
}

/*******************************************************************************
* Function Name  : DisplayChar
* Description    : Wrapper function for the Std.lib.  LCD_DisplayChar
* Input          : Line, Column, Ascii
*                  number
* Output         : None
* Return         : None
*******************************************************************************/
static void DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
  uint16_t xpos = 16*Column;
  if (LCD_GetXAxesDirection() == LCD_X_AXES_INVERTED)
  {
    xpos = 320-xpos;
  }
  LCD_DisplayChar(Line, xpos, Ascii);
}


/*******************************************************************************
* Function Name  : ComputeVisualization
* Description    : Starting from the value of the bMenuIndex, this function
*                  extract the information about the present menu to be
*                  displayed on LCD
* Input          : bMenuIndex variable, State
* Output         : Present visualization
* Return         : None
*******************************************************************************/

uint8_t ComputeVisualization(uint8_t bLocal_MenuIndex, State_t State)
{
  uint8_t bTemp;

  switch(bLocal_MenuIndex)
  {
  case CONTROL_MODE_SPEED_MENU:
    bTemp = VISUALIZATION_1;
    break;
  case REF_SPEED_MENU:
    bTemp = VISUALIZATION_1;
    break;
  case MOTOR_SPD_MENU:
    bTemp = VISUALIZATION_1;
    break;

  case P_SPEED_MENU:
    bTemp = VISUALIZATION_2;
    break;
  case I_SPEED_MENU :
    bTemp = VISUALIZATION_2;
    break;
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_SPEED_MENU:
    bTemp = VISUALIZATION_2;
    break;
#endif

  case P_TORQUE_MENU:
    bTemp = VISUALIZATION_3;
    break;
  case I_TORQUE_MENU :
    bTemp = VISUALIZATION_3;
    break;
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_TORQUE_MENU :
    bTemp = VISUALIZATION_3;
    break;
#endif

  case P_FLUX_MENU :
    bTemp = VISUALIZATION_4;
    break;
  case I_FLUX_MENU :
    bTemp = VISUALIZATION_4;
    break;
#ifdef DIFFERENTIAL_TERM_ENABLED
  case D_FLUX_MENU :
    bTemp = VISUALIZATION_4;
    break;
#endif

#ifdef FLUX_WEAKENING
  case P_VOLT_MENU :
    bTemp = VISUALIZATION_11;
    break;
  case I_VOLT_MENU:
    bTemp = VISUALIZATION_11;
  case TARGET_VOLT_MENU :
    bTemp = VISUALIZATION_11;
    break;
#endif

  case POWER_STAGE_MENU :
    bTemp = VISUALIZATION_5;
    break;

  case CONTROL_MODE_TORQUE_MENU:
    bTemp = VISUALIZATION_6;
    break;
  case IQ_REF_MENU :
    bTemp = VISUALIZATION_6;
    break;
  case ID_REF_MENU:
    bTemp = VISUALIZATION_6;
    break;
  case MOTOR_TRQ_MENU:
    bTemp = VISUALIZATION_6;
    break;

#ifdef OBSERVER_GAIN_TUNING
  case K1_MENU:
    bTemp = VISUALIZATION_9;
    break;
  case K2_MENU:
    bTemp = VISUALIZATION_9;
    break;
  case P_PLL_MENU:
    bTemp = VISUALIZATION_9;
    break;
  case I_PLL_MENU:
    bTemp = VISUALIZATION_9;
    break;
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
  case DAC_PB0_MENU:
    bTemp = VISUALIZATION_10;
    break;
  case DAC_PB1_MENU:
    bTemp = VISUALIZATION_10;
    break;
#endif
  default:
    bTemp = VISUALIZATION_1;
    break;
  }

  if ((State == FAULT_NOW) || (State == FAULT_OVER))
  {
    bTemp = VISUALIZATION_7;
  }
  else if (State == STOP)
  {
    bTemp = VISUALIZATION_8;
  }

  return (bTemp);
}

#if (defined(USE_STM3210C_EVAL) || defined(USE_STM322xG_EVAL) || defined(USE_STM324xG_EVAL))
static bool JOY_Pressed(InputKey inKey)
{
  bool retVal = false;
  uint8_t Button = JOY_SEL;
	JOYState_TypeDef joystate;
  switch (inKey)
  {
  case KEY_JOYSTICK_UP:
    Button = JOY_UP;
    break;
  case KEY_JOYSTICK_DOWN:
    Button = JOY_DOWN;
    break;
  case KEY_JOYSTICK_LEFT:
    Button = JOY_LEFT;
    break;
  case KEY_JOYSTICK_RIGHT:
    Button = JOY_RIGHT;
    break;
  case KEY_JOYSTICK_SEL:
    Button = JOY_SEL;
    break;
  }
  joystate = IOE_JoyStickGetState();
  if (joystate == Button)
  {
    retVal = true;
  }
  else
  {
    retVal = false;
  }
  return (retVal);
}
#else
static bool JOY_Pressed(InputKey inKey)
{
  bool retVal = false;
  Button_TypeDef Button = BUTTON_SEL;
  switch (inKey)
  {
  case KEY_JOYSTICK_UP:
    Button = BUTTON_UP;
    break;
  case KEY_JOYSTICK_DOWN:
    Button = BUTTON_DOWN;
    break;
  case KEY_JOYSTICK_LEFT:
    Button = BUTTON_LEFT;
    break;
  case KEY_JOYSTICK_RIGHT:
    Button = BUTTON_RIGHT;
    break;
  case KEY_JOYSTICK_SEL:
    Button = BUTTON_SEL;
    break;
  }
  if (STM_EVAL_PBGetState(Button) == JOYSTIK_ACTIVE)
  {
    retVal = true;
  }
  else
  {
    retVal = false;
  }
  return (retVal);
}
#endif

/*******************************************************************************
* Function Name  : KEYS_Read
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN, KEY_HOLD or NOKEY
*******************************************************************************/
uint8_t KEYS_Read ( void )
{
  /* "RIGHT" key is pressed */
  if (JOY_Pressed(KEY_JOYSTICK_RIGHT))
  {
    if (bPrevious_key == RIGHT)
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = RIGHT;
      return RIGHT;
    }
  }
  /* "LEFT" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_LEFT))
  {
    if (bPrevious_key == LEFT)
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = LEFT;
      return LEFT;
    }
  }
  /* "SEL" key is pressed */
  if (JOY_Pressed(KEY_JOYSTICK_SEL))
  {
    if (bPrevious_key == SEL)
    {
      return KEY_HOLD;
    }
    else
    {
      if ( (TB_DebounceDelay_IsElapsed() == false) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == true) && ( (bKey_Flag & SEL_FLAG) == 0) )
      {
        bKey_Flag |= SEL_FLAG;
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce
      }
      else if ( (TB_DebounceDelay_IsElapsed() == true) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )
      {
        bKey_Flag &= (uint8_t)(~SEL_FLAG);
        bPrevious_key = SEL;
        return SEL;
      }
      return NOKEY;
      }
    }
  }
  /* "SEL" key is pressed */
  else if(!LL_GPIO_IsInputPinSet(USER_BUTTON_PORT, USER_BUTTON_BIT))
  {
    if (bPrevious_key == SEL)
    {
      return KEY_HOLD;
    }
    else
    {
      if ( (TB_DebounceDelay_IsElapsed() == false) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == true) && ( (bKey_Flag & SEL_FLAG) == 0) )
      {
        bKey_Flag |= SEL_FLAG;
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce
      }
      else if ( (TB_DebounceDelay_IsElapsed() == true) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )
      {
        bKey_Flag &= (uint8_t)(~SEL_FLAG);
        bPrevious_key = SEL;
        return SEL;
      }
      return NOKEY;
      }
    }
  }
   /* "UP" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_UP))
  {
    if (bPrevious_key == UP)
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = UP;
      return UP;
    }
  }
  /* "DOWN" key is pressed */
  else if (JOY_Pressed(KEY_JOYSTICK_DOWN))
  {
    if (bPrevious_key == DOWN)
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = DOWN;
      return DOWN;
    }
  }

  /* No key is pressed */
  else
  {
    bPrevious_key = NOKEY;
    return NOKEY;
  }
}



/*******************************************************************************
* Function Name  : KEYS_process
* Description    : Process key
* Input          : Key code
* Output         : None
* Return         : None
*******************************************************************************/
static void KEYS_process(UI_Handle_t *pHandle)
{

  LCDV_Handle_t *pHdl = (LCDV_Handle_t *)pHandle;

  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t* pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
  PID_Handle_t* pSpeedLoopPID = pMCT->pPIDSpeed;
  PID_Handle_t* pIqLoopPID = pMCT->pPIDIq;
  PID_Handle_t* pIdLoopPID = pMCT->pPIDId;
  PID_Handle_t* pFluxWeakeningLoopPID = pMCT->pPIDFluxWeakening;
  FW_Handle_t* pFWCtrl = pMCT->pFW;
  State_t State = MCI_GetSTMState(pMCI);
  SpeednTorqCtrl_Handle_t *pSTC = pMCT->pSpeednTorqueCtrl;

  bKey = KEYS_Read();    // read key pushed (if any...)

  if (bKey == SEL)
  {
    /* Queries the STM and send start, stop or fault ack depending on the state. */
    switch (MCI_GetSTMState(pMCI))
    {
    case FAULT_OVER:
      {
        MCI_FaultAcknowledged(pMCI);
      }
      break;
    case IDLE:
      {
        MCI_StartMotor(pMCI);
      }
      break;
    default:
      {
        MCI_StopMotor(pMCI);
      }
      break;
    }
  }


  if (bPresent_Visualization == VISUALIZATION_7)
  {
    /* Fault condition */
#ifdef DUALDRIVE
    switch(bKey)
    {
    case UP:
    case DOWN:
      {
        uint8_t bSelDrv = pHandle->bSelectedDrive;
        if (bSelDrv == 0)
        {
          pHandle->bSelectedDrive = 1;
          bPresent_Visualization--; /* Forces display update */
        }
        else
        {
          pHandle->bSelectedDrive = 0;
          bPresent_Visualization--; /* Forces display update */
        }
      }
      break;
    default:
      break;
    }
#endif
  }
  else
  {
    /* Manage Joy input if no fault */
    switch (bMenu_index)
    {
    case MOTOR_SPD_MENU:
    case MOTOR_TRQ_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
#ifdef DUALDRIVE
        {
          uint8_t bSelDrv = pHandle->bSelectedDrive;
          if (bSelDrv == 0)
          {
            pHandle->bSelectedDrive = 1;
          }
          else
          {
            pHandle->bSelectedDrive = 0;
          }
        }
#endif
        break;

      case RIGHT:
        if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
        {
          bMenu_index = CONTROL_MODE_SPEED_MENU;
        }
        else
        {
          bMenu_index = CONTROL_MODE_TORQUE_MENU;
        }
        break;

      case LEFT:
#if (DAC_FUNCTIONALITY == ENABLE)
        bMenu_index = DAC_PB1_MENU;
#elif defined OBSERVER_GAIN_TUNING
        bMenu_index = I_PLL_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif
        break;

      default:
        break;
      }
      break;

    case CONTROL_MODE_SPEED_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE) == STC_SPEED_MODE)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE, STC_TORQUE_MODE);
        }
        else
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE, STC_SPEED_MODE);
        }
        break;

      case RIGHT:
        bMenu_index = REF_SPEED_MENU;
        break;

      case LEFT:
        bMenu_index = MOTOR_SPD_MENU;
        break;

      default:
        break;
      }
      break;

    case REF_SPEED_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetLastRampFinalSpeed(pMCI) <= (MAX_APPLICATION_SPEED / 6))
        {
          MCI_ExecSpeedRamp(pMCI, MCI_GetLastRampFinalSpeed(pMCI) + SPEED_INC_DEC, SPEED_INC_DEC_DURATION);
        }
        break;

      case DOWN:
        if (MCI_GetLastRampFinalSpeed(pMCI) >= - (MAX_APPLICATION_SPEED / 6))
        {
          MCI_ExecSpeedRamp(pMCI, MCI_GetLastRampFinalSpeed(pMCI) - SPEED_INC_DEC, SPEED_INC_DEC_DURATION);
        }
        break;

      case RIGHT:
        bMenu_index = P_SPEED_MENU;
        break;

      case LEFT:
        bMenu_index = CONTROL_MODE_SPEED_MENU;
        break;

      default:
        break;
      }
      break;

    case CONTROL_MODE_TORQUE_MENU:
      switch(bKey)
      {
      case UP:
      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE) == STC_SPEED_MODE)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE, STC_TORQUE_MODE);
        }
        else
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_CONTROL_MODE, STC_SPEED_MODE);
        }
        break;

      case RIGHT:
        bMenu_index = IQ_REF_MENU;
        break;

      case LEFT:
        bMenu_index = MOTOR_TRQ_MENU;
        break;

      default:
        break;
      }
      break;

    case IQ_REF_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetIqdref(pMCI).qI_Component1 <= NOMINAL_CURRENT - TORQUE_INC_DEC)
        {
          pHdl->Iqdref.qI_Component1 += TORQUE_INC_DEC;
          MCI_SetCurrentReferences(pMCI,pHdl->Iqdref);
        }
        break;

      case DOWN:
        if (MCI_GetIqdref(pMCI).qI_Component1 >= -NOMINAL_CURRENT + TORQUE_INC_DEC)
        {
          pHdl->Iqdref.qI_Component1 -= TORQUE_INC_DEC;
          MCI_SetCurrentReferences(pMCI,pHdl->Iqdref);
        }
        break;

      case RIGHT:
        bMenu_index = ID_REF_MENU;
        break;

      case LEFT:
        if(State == IDLE)
        {
          bMenu_index = CONTROL_MODE_TORQUE_MENU;
        }
        else
        {
#if (DAC_FUNCTIONALITY == ENABLE)
          bMenu_index = DAC_PB1_MENU;
#elif defined OBSERVER_GAIN_TUNING
          bMenu_index = I_PLL_MENU;
#else
          bMenu_index = POWER_STAGE_MENU;
#endif
        }
        break;

      default:
        break;
      }
      break;

    case ID_REF_MENU:
      switch(bKey)
      {
      case UP:
        if (MCI_GetIqdref(pMCI).qI_Component2 <= NOMINAL_CURRENT - FLUX_INC_DEC)
        {
          pHdl->Iqdref.qI_Component2 += FLUX_INC_DEC;
          MCI_SetCurrentReferences(pMCI,pHdl->Iqdref);
        }
        break;

      case DOWN:
        if (MCI_GetIqdref(pMCI).qI_Component2 >= FLUX_INC_DEC - NOMINAL_CURRENT)
        {
          pHdl->Iqdref.qI_Component2 -= FLUX_INC_DEC;
          MCI_SetCurrentReferences(pMCI,pHdl->Iqdref);
        }
        break;

      case RIGHT:
        bMenu_index = P_TORQUE_MENU;
        break;

      case LEFT:
        bMenu_index = IQ_REF_MENU;
        break;

      default:
        break;
      }
      break;

    case P_SPEED_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKP(pSpeedLoopPID) <= INT16_MAX-KP_GAIN_INC_DEC)
        {
          PID_SetKP(pSpeedLoopPID,PID_GetKP(pSpeedLoopPID) + KP_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKP(pSpeedLoopPID) >= KP_GAIN_INC_DEC)
        {
          PID_SetKP(pSpeedLoopPID,PID_GetKP(pSpeedLoopPID) - KP_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = I_SPEED_MENU;
        break;

      case LEFT:
        bMenu_index = REF_SPEED_MENU;
        break;

      default:
        break;
      }
      break;

    case I_SPEED_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKI(pSpeedLoopPID) <= INT16_MAX-KI_GAIN_INC_DEC)
        {
          PID_SetKI(pSpeedLoopPID,PID_GetKI(pSpeedLoopPID) + KI_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKI(pSpeedLoopPID) >= KI_GAIN_INC_DEC)
        {
          PID_SetKI(pSpeedLoopPID,PID_GetKI(pSpeedLoopPID) - KI_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_SPEED_MENU;
#else
        bMenu_index = P_TORQUE_MENU;
#endif
        break;

      case LEFT:
        bMenu_index = P_SPEED_MENU;
        break;

      default:
        break;
      }
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_SPEED_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)pSpeedLoopPID) <= INT16_MAX-KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pSpeedLoopPID,PID_GetKD((CPID_PI)pSpeedLoopPID) + KD_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKD((CPID_PI)pSpeedLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pSpeedLoopPID,PID_GetKD((CPID_PI)pSpeedLoopPID) - KD_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = P_TORQUE_MENU;
        break;

      case LEFT:
        bMenu_index = I_SPEED_MENU;
        break;

      default:
        break;
      }
      break;
#endif

    case P_TORQUE_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKP(pIqLoopPID) <= INT16_MAX - KP_GAIN_INC_DEC)
        {
          PID_SetKP(pIqLoopPID,PID_GetKP(pIqLoopPID) + KP_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKP(pIqLoopPID) >= KP_GAIN_INC_DEC)
        {
          PID_SetKP(pIqLoopPID,PID_GetKP(pIqLoopPID) - KP_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = I_TORQUE_MENU;
        break;

      case LEFT:
        if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
        {
#ifdef DIFFERENTIAL_TERM_ENABLED
          bMenu_index = D_SPEED_MENU;
#else
          bMenu_index = I_SPEED_MENU;
#endif
        }
        else
        {
          bMenu_index = ID_REF_MENU;
        }
        break;

      default:
        break;
      }
      break;

    case I_TORQUE_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKI(pIqLoopPID) <= INT16_MAX - KI_GAIN_INC_DEC)
        {
          PID_SetKI(pIqLoopPID,PID_GetKI(pIqLoopPID) + KI_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKI(pIqLoopPID) >= KI_GAIN_INC_DEC)
        {
          PID_SetKI(pIqLoopPID,PID_GetKI(pIqLoopPID) - KI_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_TORQUE_MENU;
#else
        bMenu_index = P_FLUX_MENU;
#endif
        break;

      case LEFT:
        bMenu_index = P_TORQUE_MENU;
        break;

      default:
        break;
      }
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_TORQUE_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)pIqLoopPID) <= INT16_MAX - KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pIqLoopPID,PID_GetKD((CPID_PI)pIqLoopPID) + KD_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKD((CPID_PI)pIqLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pIqLoopPID,PID_GetKD((CPID_PI)pIqLoopPID) + KD_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = P_FLUX_MENU;
        break;

      case LEFT:
        bMenu_index = I_TORQUE_MENU;
        break;

      default:
        break;
      }
      break;
#endif
    case P_FLUX_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKP(pIdLoopPID) <= INT16_MAX-KP_GAIN_INC_DEC)
        {
          PID_SetKP(pIdLoopPID,PID_GetKP(pIdLoopPID) + KP_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKP(pIdLoopPID) >= KP_GAIN_INC_DEC)
        {
          PID_SetKP(pIdLoopPID,PID_GetKP(pIdLoopPID) - KP_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = I_FLUX_MENU;
        break;

      case LEFT:
#ifdef  DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_TORQUE_MENU;
#else
        bMenu_index = I_TORQUE_MENU;
#endif
        break;

      default:
        break;
      }
      break;

    case I_FLUX_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKI(pIdLoopPID) <= INT16_MAX-KI_GAIN_INC_DEC)
        {
          PID_SetKI(pIdLoopPID,PID_GetKI(pIdLoopPID) + KI_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKI(pIdLoopPID) >= KI_GAIN_INC_DEC)
        {
          PID_SetKI(pIdLoopPID,PID_GetKI(pIdLoopPID) - KI_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
#ifdef DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_FLUX_MENU;
#elif defined FLUX_WEAKENING
        bMenu_index = P_VOLT_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif
        break;

      case LEFT:
        bMenu_index = P_FLUX_MENU;
        break;

      default:
        break;
      }
      break;

#ifdef DIFFERENTIAL_TERM_ENABLED
    case D_FLUX_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKD((CPID_PI)pIdLoopPID) <= INT16_MAX - KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pIdLoopPID,PID_GetKD((CPID_PI)pIdLoopPID) + KD_GAIN_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKD((CPID_PI)pIdLoopPID) >= KD_GAIN_INC_DEC)
        {
          PID_SetKD((CPID_PI)pIdLoopPID,PID_GetKD((CPID_PI)pIdLoopPID) - KD_GAIN_INC_DEC);
        }
        break;

      case RIGHT:
#ifdef FLUX_WEAKENING
        bMenu_index = P_VOLT_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif
        break;

      case LEFT:
        bMenu_index = I_FLUX_MENU;
        break;

      default:
        break;
      }
      break;
#endif

#ifdef FLUX_WEAKENING
    case P_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKP(pFluxWeakeningLoopPID) <= 32500)
        {
          PID_SetKP(pFluxWeakeningLoopPID,PID_GetKP(pFluxWeakeningLoopPID) + KP_VOLT_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKP(pFluxWeakeningLoopPID) >= KP_VOLT_INC_DEC)
        {
          PID_SetKP(pFluxWeakeningLoopPID,PID_GetKP(pFluxWeakeningLoopPID) - KP_VOLT_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = I_VOLT_MENU;
        break;

      case LEFT:
#ifdef DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_FLUX_MENU;
#else
        bMenu_index = I_FLUX_MENU;
#endif
        break;

      default:
        break;
      }
      break;

    case I_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (PID_GetKI(pFluxWeakeningLoopPID) <= 32500)
        {
          PID_SetKI(pFluxWeakeningLoopPID,PID_GetKI(pFluxWeakeningLoopPID) + KI_VOLT_INC_DEC);
        }
        break;

      case DOWN:
        if (PID_GetKI(pFluxWeakeningLoopPID) >= KI_VOLT_INC_DEC)
        {
          PID_SetKI(pFluxWeakeningLoopPID,PID_GetKI(pFluxWeakeningLoopPID) - KI_VOLT_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = TARGET_VOLT_MENU;
        break;

      case LEFT:
        bMenu_index = P_VOLT_MENU;
        break;

      default:
        break;
      }
      break;

    case TARGET_VOLT_MENU:
      switch(bKey)
      {
      case UP:
        if (FW_GetVref(pFWCtrl) <= (1000-VOLT_LIM_INC_DEC))
        {
          FW_SetVref(pFWCtrl, FW_GetVref(pFWCtrl) + VOLT_LIM_INC_DEC);
        }
        break;

      case DOWN:
        if (FW_GetVref(pFWCtrl) >= VOLT_LIM_INC_DEC)
        {
          FW_SetVref(pFWCtrl, FW_GetVref(pFWCtrl) - VOLT_LIM_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = POWER_STAGE_MENU;
        break;

      case LEFT:
        bMenu_index = I_VOLT_MENU;
        break;

      default:
        break;
      }
      break;

#endif

    case POWER_STAGE_MENU:
      switch(bKey)
      {
      case RIGHT:
#ifdef OBSERVER_GAIN_TUNING
        bMenu_index = K1_MENU;
#elif (DAC_FUNCTIONALITY == ENABLE)
        bMenu_index = DAC_PB0_MENU;
#else
        if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
#endif
        break;

      case LEFT:
#ifdef FLUX_WEAKENING
        bMenu_index = TARGET_VOLT_MENU;
#elif defined DIFFERENTIAL_TERM_ENABLED
        bMenu_index = D_FLUX_MENU;
#else
        bMenu_index = I_FLUX_MENU;
#endif
        break;

      default:
        break;
      }
      break;

#ifdef OBSERVER_GAIN_TUNING
    case K1_MENU:
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) <= -K1_INC_DEC)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) +
                      K1_INC_DEC);
        }
        break;

      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) >= -600000)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) -
                      K1_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = K2_MENU;
        break;

      case LEFT:
        bMenu_index = POWER_STAGE_MENU;
        break;

      default:
        break;
      }
      break;

    case K2_MENU:
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) <= INT16_MAX - K2_INC_DEC)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2) +
                      K2_INC_DEC);
        }
        break;

      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C1) >= -INT16_MAX + K2_INC_DEC)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_OBSERVER_C2) -
                      K2_INC_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = P_PLL_MENU;
        break;

      case LEFT:
        bMenu_index = K1_MENU;
        break;

      default:
        break;
      }
      break;

    case P_PLL_MENU:
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP) <= 32000)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_PLL_KP,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP) +
                      PLL_IN_DEC);
        }
        break;

      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP) > 0)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_PLL_KP,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KP) -
                      PLL_IN_DEC);
        }
        break;

      case RIGHT:
        bMenu_index = I_PLL_MENU;
        break;

      case LEFT:
        bMenu_index = K2_MENU;
        break;

      default:
        break;
      }
      break;

    case I_PLL_MENU:
      switch(bKey)
      {
      case UP:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI) <= 32000)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_PLL_KI,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI) +
                      PLL_IN_DEC);
        }
        break;

      case DOWN:
        if (UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI) > 0)
        {
          UI_SetReg(pHandle, MC_PROTOCOL_REG_PLL_KI,
                    UI_GetReg(pHandle, MC_PROTOCOL_REG_PLL_KI) -
                      PLL_IN_DEC);
        }
        break;

      case RIGHT:
#if (DAC_FUNCTIONALITY == ENABLE)
        bMenu_index = DAC_PB0_MENU;
#else
        if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
#endif
        break;

      case LEFT:
        bMenu_index = P_PLL_MENU;
        break;

      default:
        break;
      }
      break;
#endif

#if (DAC_FUNCTIONALITY == ENABLE)
    case DAC_PB0_MENU:
      switch(bKey)
      {
      case UP:
        LCDV_DACSelUpdate(pHandle, 1, DAC_CH0);
        break;

      case DOWN:
        LCDV_DACSelUpdate(pHandle, -1, DAC_CH0);
        break;

      case RIGHT:
        bMenu_index = DAC_PB1_MENU;
        break;

      case LEFT:
#ifdef OBSERVER_GAIN_TUNING
        bMenu_index = I_PLL_MENU;
#else
        bMenu_index = POWER_STAGE_MENU;
#endif
        break;

      default:
        break;
      }
      break;

    case DAC_PB1_MENU:
      switch(bKey)
      {
      case UP:
        LCDV_DACSelUpdate(pHandle, 1, DAC_CH1);
        break;

      case DOWN:
        LCDV_DACSelUpdate(pHandle, -1, DAC_CH1);
        break;

      case RIGHT:
        if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
        {
          bMenu_index = MOTOR_SPD_MENU;
        }
        else
        {
          bMenu_index = MOTOR_TRQ_MENU;
        }
        break;

      case LEFT:
        bMenu_index = DAC_PB0_MENU;
        break;

      default:
        break;
      }
      break;
#endif

    default:
      break;
    }
  }
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

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
