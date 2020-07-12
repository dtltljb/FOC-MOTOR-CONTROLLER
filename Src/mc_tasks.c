	

/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition
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

#include "pid_regulator.h"
#include "digital_output.h"
#include "mc_type.h"
#include "mc_math.h"
#include "mc_irq_handler.h"
#include "mc_config.h"
#include "state_machine.h"
#include "mc_api.h"
#include "circle_limitation.h"
#include "pwm_curr_fdbk.h"
/* PWMC derived class includes */
#include "r3_1_f30X_pwm_curr_fdbk.h"
#include "revup_ctrl.h"
#include "bus_voltage_sensor.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "ntc_temperature_sensor.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "ramp_ext_mngr.h"
#include "SystemNDriveParams.h"

#include "mc_tasks.h"
#include "mc_extended_api.h"

/* USER CODE BEGIN Includes */

#include "CANopen.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

static uint8_t UDC_Channel = 0u;
static uint16_t UDC_ConvertedValue = 0u;
static volatile UDRC_State_t UDC_State = UDRC_STATE_IDLE;

uint8_t bMCBootCompleted = 0;
/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
static void TSK_MediumFrequencyTaskM1(void);
static void FOC_Clear(uint8_t bMotor);
static void FOC_InitAdditionalMethods(uint8_t bMotor);
static void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrController(uint8_t bMotor);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
static void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
static bool TSK_StopPermanencyTimeHasElapsedM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
  bMCBootCompleted = 0;
  pCLM[M1] = &CircleLimitationM1;

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWMC_R3_1_F3_Handle_M1._Super;
  R3_1_F30X_Init(&PWMC_R3_1_F3_Handle_M1);
     
  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  STM_Init(&STM[M1]);
  
  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);
  pPIDSpeed[M1] = &PIDSpeedHandle_M1;
  pSTC[M1] = &SpeednTorqCtrlM1;
  HALL_Init (&HALL_M1);
  
  STC_Init(pSTC[M1],pPIDSpeed[M1], &HALL_M1._Super);
  
  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);
  pPIDIq[M1] = &PIDIqHandle_M1;
  pPIDId[M1] = &PIDIdHandle_M1;
  pBusSensorM1 = &RealBusVoltageSensorParamsM1;
  RVBS_Init(pBusSensorM1, pwmcHandle[M1]);
  
  //Power Measurement M1
  pMPM[M1] = &PQD_MotorPowMeasM1;
  pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];

  NTC_Init(&TempSensorParamsM1,pwmcHandle[M1]);    
  pTemperatureSensor[M1] = &TempSensorParamsM1;
    
  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);
  FOC_Clear(M1);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).qI_Component2;
  oMCInterface[M1] = & Mci[M1];
  MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
  MCI_ExecSpeedRamp(oMCInterface[M1],
  STC_GetMecSpeedRef01HzDefault(pSTC[M1]),0); /*First command to STC*/
  pMCIList[M1] = oMCInterface[M1];
  MCT[M1].pPIDSpeed = pPIDSpeed[M1];
  MCT[M1].pPIDIq = pPIDIq[M1];
  MCT[M1].pPIDId = pPIDId[M1];
  MCT[M1].pPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
  MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
  MCT[M1].pRevupCtrl = MC_NULL;              /* only if M1 is not sensorless*/
  MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &HALL_M1; 
  MCT[M1].pSpeedSensorAux = MC_NULL;
  MCT[M1].pSpeedSensorVirtual = MC_NULL;
  MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
  MCT[M1].pStateMachine = &STM[M1];
  MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
  MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
  MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
  MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
  MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
  MCT[M1].pFW = MC_NULL;
  MCT[M1].pFF = MC_NULL;
  MCT[M1].pSCC = MC_NULL;
  MCT[M1].pOTT = MC_NULL;
  pMCTList[M1] = &MCT[M1];
 
  bMCBootCompleted = 1;
  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */
}

/**
  * @brief  It executes MC tasks: safety task and medium frequency for all
  *         drive instances. It have to be clocked with Systick frequnecy.
  * @param  None
  * @retval None
  */
void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief  It executes some of the control duties on Motor 1 accordingly with
  *         the present state of its state machine. In particular, duties
  *         requiring a specific timing (e.g. speed controller) are here
  *         executed
  * @param  None
  * @retval void
  */
void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */
  State_t StateM1;
  int16_t wAux = 0;

  (void) HALL_CalcAvrgMecSpeed01Hz(&HALL_M1,&wAux);
  PQD_CalcElMotorPower(pMPM[M1]);  
  StateM1 = STM_GetState(&STM[M1]);
  switch(StateM1)
  {
  case IDLE_START:
    R3_1_F30X_TurnOnLowSides(pwmcHandle[M1]);
    TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
    STM_NextState(&STM[M1],CHARGE_BOOT_CAP);
    break;
  case CHARGE_BOOT_CAP:
    if (TSK_ChargeBootCapDelayHasElapsedM1())
    {
      PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_START);
      /* USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */

      /* USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */
      STM_NextState(&STM[M1],OFFSET_CALIB);
    }
    break;
  case OFFSET_CALIB:
    if (PWMC_CurrentReadingCalibr(pwmcHandle[M1],CRC_EXEC))
    {
      STM_NextState(&STM[M1],CLEAR);
    }
    break;
  case CLEAR:
    HALL_Clear(&HALL_M1);
    if(STM_NextState(&STM[M1], START) == true)
    {
      FOC_Clear(M1);
      R3_1_F30X_SwitchOnPWM(pwmcHandle[M1]);
    }
    break;  
  case START:
    {
      STM_NextState(&STM[M1], START_RUN); /* only for sensored*/
    }
    break;
  case START_RUN:
    {
      /* USER CODE BEGIN MediumFrequencyTask M1 1 */

      /* USER CODE END MediumFrequencyTask M1 1 */      
	  FOC_InitAdditionalMethods(M1);
      FOC_CalcCurrRef(M1);
      STM_NextState(&STM[M1], RUN);
    }
    STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M1]); /* Exec the speed ramp after changing of the speed sensor */
	
    break;
  case RUN:
    /* USER CODE BEGIN MediumFrequencyTask M1 2 */
	  FOC_InitAdditionalMethods(M1);
    /* USER CODE END MediumFrequencyTask M1 2 */
    MCI_ExecBufferedCommands(oMCInterface[M1]);
    FOC_CalcCurrRef(M1);
 
 
    /* USER CODE BEGIN MediumFrequencyTask M1 3 */

    /* USER CODE END MediumFrequencyTask M1 3 */
    break;
  case ANY_STOP:
    R3_1_F30X_SwitchOffPWM(pwmcHandle[M1]);
    FOC_Clear(M1);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[M1]);
    TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
    /* USER CODE BEGIN MediumFrequencyTask M1 4 */

    /* USER CODE END MediumFrequencyTask M1 4 */
    STM_NextState(&STM[M1], STOP);
    break;
  case STOP:
    if(TSK_StopPermanencyTimeHasElapsedM1())
    {
      STM_NextState(&STM[M1], STOP_IDLE);
    }
    break;
  case STOP_IDLE:
    /* USER CODE BEGIN MediumFrequencyTask M1 5 */

    /* USER CODE END MediumFrequencyTask M1 5 */
    STM_NextState(&STM[M1], IDLE);
    break;
  default:
    break;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  
  Curr_Components Inull = {(int16_t)0, (int16_t)0};
  Volt_Components Vnull = {(int16_t)0, (int16_t)0};

  FOCVars[bMotor].Iab = Inull;
  FOCVars[bMotor].Ialphabeta = Inull;
  FOCVars[bMotor].Iqd = Inull;
  FOCVars[bMotor].Iqdref = Inull;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = Vnull;
  FOCVars[bMotor].Valphabeta = Vnull;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
  PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */
	
/*
	if(oMCInterface[bMotor]->lastCommand	== MCI_EXECSPEEDRAMP){
			if( oMCInterface[bMotor]->hFinalSpeed < 0 )
					HALL_M1.PhaseShift	=	(int16_t)(HALL_PHASE_SHIFT_N * 65536/360);	
			else
					HALL_M1.PhaseShift	=	(int16_t)(HALL_PHASE_SHIFT_P * 65536/360);
	} else {
			if( oMCInterface[bMotor]->hFinalTorque < 0 )
					HALL_M1.PhaseShift	=	(int16_t)(HALL_PHASE_SHIFT_N * 65536/360);	
			else
					HALL_M1.PhaseShift	=	(int16_t)(HALL_PHASE_SHIFT_P * 65536/360);
	}
*/
	
		if(oMCInterface[bMotor]->lastCommand	== MCI_EXECSPEEDRAMP){
			if( oMCInterface[bMotor]->hFinalSpeed < 0 )
					HALL_M1.PhaseShift	=	(int16_t)(CO_OD_RAM.HallPhaseN * 65536/360);	
			else
					HALL_M1.PhaseShift	=	(int16_t)(CO_OD_RAM.HallPhaseP * 65536/360);
	} else {
			if( oMCInterface[bMotor]->hFinalTorque < 0 )
					HALL_M1.PhaseShift	=	(int16_t)(CO_OD_RAM.HallPhaseN * 65536/360);	
			else
					HALL_M1.PhaseShift	=	(int16_t)(CO_OD_RAM.HallPhaseP * 65536/360);
	}
	
  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
void FOC_CalcCurrRef(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */
	/*
    FOCVars[bMotor].hTeref = 0x03e8;		//debug	test use 0x03e8
    FOCVars[bMotor].Iqdref.qI_Component1 = FOCVars[bMotor].hTeref;
	*/
  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Accordingly with the present state(s) of the state machine(s), it
  *         executes those motor control duties requiring a high frequency rate
  *         and a precise timing (e.g. FOC current control loop)
  * @param  None
  * @retval uint8_t It return the motor instance number of last executed FOC.
  */
uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  
  uint8_t bMotorNbr = 0;
  uint16_t hFOCreturn;
 
  HALL_CalcElAngle (&HALL_M1); 

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrController(M1);
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_FOC_DURATION)
  {
    STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
  }
  else
  {
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */  
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */
  return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_FOC_DURATION otherwise
  */
#pragma inline
uint16_t FOC_CurrController(uint8_t bMotor)
{
  Curr_Components Iab, Ialphabeta, Iqd;
  Volt_Components Valphabeta, Vqd;
  int16_t hElAngledpp;
  uint16_t hCodeError;

  hElAngledpp = SPD_GetElAngle(STC_GetSpeedSensor(pSTC[bMotor]));
  PWMC_GetPhaseCurrents(pwmcHandle[bMotor], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngledpp);
  Vqd.qV_Component1 = PI_Controller(pPIDIq[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component1) - Iqd.qI_Component1);

  Vqd.qV_Component2 = PI_Controller(pPIDId[bMotor],
            (int32_t)(FOCVars[bMotor].Iqdref.qI_Component2) - Iqd.qI_Component2);
  FOCVars[bMotor].Vqd = Vqd;
  Vqd = Circle_Limitation(pCLM[bMotor], Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngledpp);
	//----------debug circute----
//	Valphabeta.qV_Component1	=	0x051a;
//	Valphabeta.qV_Component2	=	0x00e2;
	//----------debug circute----
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[bMotor], Valphabeta);
  FOCVars[bMotor].Iab = Iab;
  FOCVars[bMotor].Ialphabeta = Ialphabeta;
  FOCVars[bMotor].Iqd = Iqd;
  FOCVars[bMotor].Valphabeta = Valphabeta;
  FOCVars[bMotor].hElAngle = hElAngledpp;
  return(hCodeError);
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
void MC_RequestRegularConv(uint8_t bChannel, uint8_t bSamplTime)
{
  ADConv_t ADConv_struct;
  if (UDC_State == UDRC_STATE_IDLE)
  {
    ADConv_struct.Channel = bChannel;
    ADConv_struct.SamplTime = bSamplTime;
    PWMC_ADC_SetSamplingTime(pwmcHandle[M1],ADConv_struct);
    UDC_State = UDRC_STATE_REQUESTED;
    UDC_Channel = bChannel;
  }
}

/**
* @brief  Get the last user-defined regular conversion.
* @retval uint16_t It returns converted value or oxFFFF for conversion error.
*         This function returns a valid result if the state returned by
*         MC_RegularConvState is UDRC_STATE_EOC.
*/
uint16_t MC_GetRegularConv(void)
{
  uint16_t hRetVal = 0xFFFFu;
  if (UDC_State == UDRC_STATE_EOC)
  {
    hRetVal = UDC_ConvertedValue;
    UDC_State = UDRC_STATE_IDLE;
  }
  return hRetVal;
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
UDRC_State_t MC_RegularConvState(void)
{
  return UDC_State;
}

/**
  * @brief  It executes safety checks (e.g. bus voltage and temperature) for all
  *         drive instances. Faults flags are also here updated
  * @param  None
  * @retval None
  */
void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {  
    TSK_SafetyTask_PWMOFF(M1);					/*	check temp\voltage\current	*/
    /* User conversion execution */
  if (UDC_State == UDRC_STATE_REQUESTED)
  {
    UDC_ConvertedValue = PWMC_ExecRegularConv (pwmcHandle[M1], UDC_Channel);
    UDC_State = UDRC_STATE_EOC;
  }
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  
  uint16_t CodeReturn;

  CodeReturn = NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* Clock temperature sensor and check for fault. It returns MC_OVER_TEMP or MC_NO_ERROR */
  CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]); /* Clock current sensor and check for fault. It return MC_BREAK_IN or MC_NO_FAULTS (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */

  if(bMotor == M1)
  {
    CodeReturn |= RVBS_CalcAvVbusFilt(pBusSensorM1);					/*over load voltage,updown voltage*/
  }
  STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
	
	
  switch (STM_GetState(&STM[bMotor])) /* Acts on PWM outputs in case of faults */
  {
  case FAULT_NOW:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    FOC_Clear(bMotor);
    MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
    break;
  case FAULT_OVER:
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
	/* USER CODE BEGIN TSK_SafetyTask_PWMOFF 2 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 2 */
    break;
  default:
    break;
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if ((oMCInterface != MC_NULL) && (bMotor < MC_NUM))
  {
    retVal = oMCInterface[bMotor];
  }
  return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
MCT_Handle_t* GetMCT(uint8_t bMotor)
{
  MCT_Handle_t* retVal = MC_NULL;
  if (bMotor < MC_NUM)
  {
    retVal = &MCT[bMotor];
  }
  return retVal;
}

/**
  * @brief  It is executed when a general hardware failure has been detected by
  *         the microcontroller and is used to put the system in safety
  *         condition.
  * @param  None
  * @retval None
  */
void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  
  R3_1_F30X_SwitchOffPWM(pwmcHandle[M1]);
  STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
 
 
 /**
  * @brief  It locks GPIO pins used for Motor Control. This prevents accidental reconfiguration 
  *
  * @param  None
  * @retval None
  */

void mc_lock_pins (void)
{
HAL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
HAL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
HAL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
HAL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
HAL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
HAL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
HAL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
HAL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
HAL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
HAL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
HAL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
HAL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
HAL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
HAL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
