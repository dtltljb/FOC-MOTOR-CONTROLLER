/**
  ******************************************************************************
  * @file    hifreqinj_fpu_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the HiFrequency
  *          Injection Control component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "hifreqinj_fpu_ctrl.h"
#include "speed_pos_fdbk.h"

#include "mc_type.h"
#include "mc_math.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup HFIControl High Frequency Injection Control
  * @brief HiFrequency Injection Control component
  *
  *
  * @{
  */

#define PI_4 ((uint16_t)8192)

/**
  * @brief  Initializes all the component variables.
  * @param  pHandle related HFI_FP_Ctrl Handle.
  * @param  pHFI_FP_Init HFI init strutcture.
  * @retval none.
  */
void HFI_FP_Init(HFI_FP_Ctrl_Handle_t *pHandle)
{
  pHandle->PIDRegulator = pHandle->PIDHandle;
  PID_HandleInit(&pHandle->PIDRegulator);

  pHandle->HiFrW = (int16_t)
                    ((((int32_t)65536) * pHandle->HiFrFreq) / (pHandle->PWMFr));

  pHandle->SineSign = 1;

  pHandle->DemEventDelay = pHandle->IdhPhase;

  pHandle->PhaseDetIndexMax = ((int32_t)pHandle->ScanRotationsNo + 1) * (int32_t)UINT16_MAX;

  pHandle->FirstScanDone = false;

  /* Assign filter stages */
  pHandle->LPIDIItransp.numStages = 1u;
  /* Assign coefficient pointer */
  pHandle->LPIDIItransp.pCoeffs = (float32_t *)((void*)(pHandle->aLPIDIIcoefficients));
  /* Clear state buffer */
  pHandle->aLPIDIIstates[0] = 0.0f;
  pHandle->aLPIDIIstates[1] = 0.0f;
  /* Assign state pointer */
  pHandle->LPIDIItransp.pState = (float32_t *)((void*)(pHandle->aLPIDIIstates));

  /* Assign filter stages */
  pHandle->HPIDIItransp.numStages = 1u;
  /* Assign coefficient pointer */
  pHandle->HPIDIItransp.pCoeffs = (float32_t *)((void*)(pHandle->aHPIDIIcoefficients));
  /* Clear state buffer */
  pHandle->aHPIDIIstates[0] = 0.0f;
  pHandle->aHPIDIIstates[1] = 0.0f;
  /* Assign state pointer */
  pHandle->HPIDIItransp.pState = (float32_t *)((void*)(pHandle->aHPIDIIstates));

  /* Assign filter stages */
  pHandle->NotchIqDIItransp.numStages = 1u;
  /* Assign coefficient pointer */
  pHandle->NotchIqDIItransp.pCoeffs = (float32_t *)((void*)(pHandle->aNotchDIIcoefficients));
  /* Clear state buffer */
  pHandle->aNotchIqDIIstates[0] = 0.0f;
  pHandle->aNotchIqDIIstates[1] = 0.0f;
  /* Assign state pointer */
  pHandle->NotchIqDIItransp.pState = (float32_t *)((void*)(pHandle->aNotchIqDIIstates));

  /* Assign filter stages */
  pHandle->NotchIdDIItransp.numStages = 1u;
  /* Assign coefficient pointer */
  pHandle->NotchIdDIItransp.pCoeffs = (float32_t *)((void*)(pHandle->aNotchDIIcoefficients));
  /* Clear state buffer */
  pHandle->aNotchIdDIIstates[0] = 0.0f;
  pHandle->aNotchIdDIIstates[1] = 0.0f;
  /* Assign state pointer */
  pHandle->NotchIdDIItransp.pState = (float32_t *)((void*)(pHandle->aNotchIdDIIstates));

  /* Assign filter stages */
  pHandle->PLLDIItransp.numStages = 1u;
  /* Assign coefficient pointer */
  pHandle->PLLDIItransp.pCoeffs = (float32_t *)((void*)(pHandle->aPLLDIIcoefficients));
  /* Clear state buffer */
  pHandle->aPLLDIIstates[0] = 0.0f;
  pHandle->aPLLDIIstates[1] = 0.0f;
  /* Assign state pointer */
  pHandle->PLLDIItransp.pState = (float32_t *)((void*)(pHandle->aPLLDIIstates));

  pHandle->PLLKp = pHandle->DefPLLKP;
  pHandle->PLLKi = pHandle->DefPLLKI;
  pHandle->MinSaturationDifference = pHandle->DefMinSaturationDifference;
}


/**
  * @brief  It should be called before each motor restart and clears the HFI
  *         internal variables
  * @param  pHandle related HFI_FP_Ctrl Handle.
  * @retval none
  */
void HFI_FP_Clear(HFI_FP_Ctrl_Handle_t *pHandle)
{
  float32_t hiframplstep;

  uint16_t averagebusvoltagevolts = VBS_GetAvBusVoltage_V(pHandle->pVbusSensor);

  float32_t hiframpls16v = ((float)(pHandle->HiFrAmplVolts) *
                             (1.732f) * (32767.0f)) / (float)(averagebusvoltagevolts);

  float32_t hiframplscans16v = ((float)(pHandle->HiFrAmplScanVolts) *
                                 (1.732f) * (32767.0f)) / (float)(averagebusvoltagevolts);

  pHandle->HiFrAmpls16V =  (int16_t)(hiframpls16v);
  pHandle->HiFrAmplScans16V = (int16_t)(hiframplscans16v);

  pHandle->HiFrAmpl = 0;

  hiframplstep = hiframpls16v / ((float32_t)(pHandle->HiFrFreq) * 2.0f
                                   * pHandle->RestartTime);
  pHandle->HiFrAmplStep = (int16_t)hiframplstep + 1;

  pHandle->VqMax = 32767 - pHandle->HiFrAmpls16V - 1;

  PID_SetIntegralTerm(&(pHandle->PIDRegulator), (int32_t)0);

  pHandle->SineSign = +1;
  pHandle->Vhf = 0;
  pHandle->HiFrTrSpeed = 0;
  pHandle->HiFrAngle = PI_4;
  pHandle->IntSumTrAngle = 0.0f;
  pHandle->PhaseDetIndex = 0;
  pHandle->HiFrTrAngle = 0;
  pHandle->PhaseDetAngle = 0;
  pHandle->PhDetAngleOld = 0;
  pHandle->DebugHifrCurrent = 0;

  HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, pHandle->LockFreq);

  pHandle->PhaseDetRotNo = 0u;
  pHandle->PhaseDetRotMax = pHandle->ScanRotationsNo * 2u + 1u;

  pHandle->STMRUN = false;

  pHandle->StateDuration = 0u;

  pHandle->IdSatDetIndex = 0u;

  pHandle->IdSatPosSum = 0;
  pHandle->IdSatNegSum = 0;

  pHandle->FirstScanDone = false;
  pHandle->IsHiFrReliable = false;

  pHandle->HiFrW = (int16_t)
                    ((((int32_t)65536) * pHandle->HiFrFreq) / (pHandle->PWMFr));

  PID_SetUpperOutputLimit(pHandle->pPIDq, pHandle->VqMax);
  PID_SetUpperIntegralTermLimit(pHandle->pPIDq, pHandle->VqMax *
                                (int32_t)PID_GetKIDivisor(pHandle->pPIDq));

  /* Clear state buffer */
  pHandle->aLPIDIIstates[0] = 0.0f;
  pHandle->aLPIDIIstates[1] = 0.0f;

  /* Clear state buffer */
  pHandle->aHPIDIIstates[0] = 0.0f;
  pHandle->aHPIDIIstates[1] = 0.0f;

  /* Clear state buffer */
  pHandle->aNotchIqDIIstates[0] = 0.0f;
  pHandle->aNotchIqDIIstates[1] = 0.0f;

  /* Clear state buffer */
  pHandle->aNotchIdDIIstates[0] = 0.0f;
  pHandle->aNotchIdDIIstates[1] = 0.0f;

  /* Clear state buffer */
  pHandle->aPLLDIIstates[0] = 0.0f;
  pHandle->aPLLDIIstates[1] = 0.0f;
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif

/**
  * @brief  It filters motor phase currents from HFI components
  * @param  pHandle related HFI_FP_Ctrl Handle.
  * @param  IqdHF current components, as measured by current sensing stage.
  * @retval Iqd current components, filtered from HFI
  */
Curr_Components HFI_FP_PreProcessing(HFI_FP_Ctrl_Handle_t *pHandle, Curr_Components IqdHF)
{
  Curr_Components iqdfilt;
  float32_t iqmeasured, idmeasured, iqfiltered, idfiltered_float;

  if (pHandle->HiFrGenEnabled == true)
  {
    iqmeasured = (float32_t)IqdHF.qI_Component1;
    idmeasured = (float32_t)IqdHF.qI_Component2;

    arm_biquad_cascade_df2T_f32 (&(pHandle->NotchIqDIItransp),
                                 &iqmeasured,
                                 &iqfiltered,
                                 1u);
    arm_biquad_cascade_df2T_f32 (&(pHandle->NotchIdDIItransp),
                                 &idmeasured,
                                 &idfiltered_float,
                                 1u);

    iqdfilt.qI_Component1 = (int16_t)iqfiltered;
    iqdfilt.qI_Component2 = (int16_t)idfiltered_float;
  }
  else
  {
    iqdfilt.qI_Component1 = IqdHF.qI_Component1;
    iqdfilt.qI_Component2 = IqdHF.qI_Component2;
  }
  return iqdfilt;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif

/**
  * @brief  It adds the HFI voltage in the qd reference frame
  * @param  pHandle related HFI_FP_Ctrl Handle.
  * @param  Vqd voltage components as calculated by current regulation stage.
  * @retval Vqd voltage components, added with HFI voltage
  */
Volt_Components HFI_FP_VqdConditioning(HFI_FP_Ctrl_Handle_t *pHandle, Volt_Components Vqd)
{
  if (pHandle->HiFrGenEnabled == true)
  {
    switch (pHandle->HiFrTrState)
    {
      case SCAN:
        Vqd.qV_Component1 = 0;
        Vqd.qV_Component2 = pHandle->Vhf;
        break;

      case DECAY1:
        Vqd.qV_Component1 = 0;
        Vqd.qV_Component2 = 0;
        break;

      case ORIENT:
        Vqd.qV_Component1 = 0;
        Vqd.qV_Component2 = pHandle->Vhf;
        break;

      case DECAY2:
        Vqd.qV_Component1 = 0;
        Vqd.qV_Component2 = 0;
        break;

      case SYNCH:
        Vqd.qV_Component1 = pHandle->Vhf;
        Vqd.qV_Component2 = 0;
        break;

      case TRACK:
        Vqd.qV_Component1 = pHandle->Vhf;
        Vqd.qV_Component2 = 0;
        break;

      case READY:
        Vqd.qV_Component1 = pHandle->Vhf;
        Vqd.qV_Component2 = 0;
        break;

      case GO:
        if (pHandle->debugmode)
        {
          /* In debug mode no torque is provided to run the motor */
          Vqd.qV_Component1 = pHandle->Vhf;
          Vqd.qV_Component2 = 0;
        }
        else
        {
          Vqd.qV_Component1 += pHandle->Vhf;
        }
        break;

      case RESTART:
        Vqd.qV_Component1 += pHandle->Vhf;
        break;

      case RESTARTSYNCH:
        Vqd.qV_Component1 += pHandle->Vhf;
        break;

      case RESTARTREADY:
        Vqd.qV_Component1 += pHandle->Vhf;
        break;

      case FAULT:
        Vqd.qV_Component1 = 0;
        Vqd.qV_Component2 = 0;
        break;

      default:
        break;
    }
  }

  return(Vqd);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif

/**
  * @brief  It synthesizes the HFI signal, and processes its results
  * @param  pHandle related HFI_FP_Ctrl Handle.
  * @param  IqdHF current components, qd phase currents (not filtered)
  * @retval none
  */
void HFI_FP_DataProcess(HFI_FP_Ctrl_Handle_t *pHandle, Curr_Components IqdHF)
{
  float32_t idmeasured, temp, idfiltered_float;
  int32_t idfiltered;
  int16_t vhf;
  Trig_Components hifrangletrigs;
  bool dodemodulation = false;

  if (pHandle->HiFrGenEnabled == true)
  {
    /*********** ALL STATES **************************************************/

    /* High frequency voltage signal generation*/
    pHandle->HiFrAngle += pHandle->HiFrW;
    hifrangletrigs = MCM_Trig_Functions(pHandle->HiFrAngle);

    vhf = (int16_t)(((int32_t)(hifrangletrigs.hSin) * pHandle->HiFrAmpl) / INT16_MAX);
    pHandle->Vhf = vhf;

    /* High frequency demodulation scheduler*/
    if (pHandle->SineSign == 1)
    {
      if (hifrangletrigs.hCos <= -pHandle->DemEventDelay)
      {
        dodemodulation = true;
        pHandle->SineSign = -1;
      }
    }
    else
    {
      if (hifrangletrigs.hCos >= pHandle->DemEventDelay)
      {
        dodemodulation = true;
        pHandle->SineSign = 1;
      }
    }

    /************ STATE MACHINE RELATED **************************************/
    switch (pHandle->HiFrTrState)
    {
      case SCAN:
        /* PLL angle generation, angle detection phase (SCAN)*/
        pHandle->PhaseDetIndex += pHandle->LockFreq;
        pHandle->PhaseDetAngle += (pHandle->LockFreq * 2);
        pHandle->DebugHifrCurrent = IqdHF.qI_Component1;
        pHandle->DebugHifrAngle = pHandle->HiFrTrAngle;
        break;

      case DECAY1:
      case ORIENT:
      case DECAY2:
        pHandle->DebugHifrCurrent = IqdHF.qI_Component1;
        pHandle->DebugHifrAngle = IqdHF.qI_Component2;
        break;

      case SYNCH:
        pHandle->DebugHifrCurrent = IqdHF.qI_Component2;
        pHandle->DebugHifrAngle = pHandle->HiFrTrAngle;
        break;

      case TRACK:
      case READY:
      case GO:
      case RESTARTREADY:
        /* Band Pass filter for High Frequency Current Id*/
        idmeasured = (float32_t)(IqdHF.qI_Component2);

        arm_biquad_cascade_df2T_f32 (&(pHandle->LPIDIItransp),
                                     &idmeasured,
                                     &temp,
                                     1u);
        arm_biquad_cascade_df2T_f32 (&(pHandle->HPIDIItransp),
                                     &temp,
                                     &idfiltered_float,
                                     1u);
        idfiltered = (int16_t)idfiltered_float;

        pHandle->DebugHifrCurrent = (int16_t)idfiltered_float;
        pHandle->DebugHifrAngle = SPD_GetElAngle(& pHandle->pHfiFpSpeedSensor->_Super);
        break;

      default:
        break;
    }

    /************ DEMODULATION AND STATE MACHINE RELATED *********************/
    /* High Frequency Current demodulation and Flux deviation algorithm*/
    if (dodemodulation == true)
    {
      pHandle->StateDuration ++;

      switch (pHandle->HiFrTrState)
      {
        case SCAN:
          if (pHandle->PhaseDetIndex < pHandle->PhaseDetIndexMax )
          {
            int32_t pllanglediff, temp_int32;
            Trig_Components plltrigs;
            float32_t plldccomponent, propact, intact, temp2;

            int16_t iqhf = IqdHF.qI_Component1 * pHandle->SineSign;

            /* Isofreq generation*/
            int16_t pllangle = pHandle->PhaseDetAngle + pHandle->HiFrTrAngle;

            int32_t phasedetanglediff = (int32_t)pHandle->PhaseDetAngle - pHandle->PhDetAngleOld;
            pHandle->PhDetAngleOld = pHandle->PhaseDetAngle;

            pllanglediff = (int32_t)pllangle - pHandle->PLLAngle;
            pHandle->PLLAngle = pllangle;

            if (phasedetanglediff < 0)
            {
              pHandle->PhaseDetRotNo ++;
            }

            plltrigs = MCM_Trig_Functions(pllangle);

            /* Mixer */
            temp_int32 = -((int32_t)iqhf * (int32_t)plltrigs.hSin);

            temp = ((float32_t)temp_int32) / 32768.0f;

            /*DC PASS*/
            arm_biquad_cascade_df2T_f32 (&(pHandle->PLLDIItransp),
                                         &temp,
                                         &plldccomponent,
                                         1u);

            /*Phase detector*/
            propact = pHandle->PLLKp * plldccomponent; /*proportional term*/
            intact = pHandle->PLLKi * plldccomponent; /*integral term*/
            temp2 = pHandle->IntSumTrAngle + intact;
            pHandle->IntSumTrAngle = temp2; /*integral sum update*/
            temp = propact + temp2;

            temp_int32 = (int32_t)temp;
            pHandle->HiFrTrAngle = (int16_t)temp_int32;

            if (pHandle->PhaseDetRotNo == pHandle->PhaseDetRotMax)
            {
              /* first scan */
              if (pllanglediff < 0)
              {
                pHandle->FirstScanDone = true;


                temp_int32 = (int32_t)IqdHF.qI_Component1 * IqdHF.qI_Component2;
                if (temp_int32 < 0 )
                {
                  pHandle->QuadDetSign1 = 1;
                }
                else
                {
                  pHandle->QuadDetSign1 = -1;
                }

                pHandle->PhaseDetRotNo ++;
              }
            }

            if (pHandle->FirstScanDone == true)
            {
              bool error = false;
              /* second scan */
              if (pHandle->PLLAngle > 0)
              {

                temp_int32 = (int32_t)IqdHF.qI_Component1 * IqdHF.qI_Component2;
                if (temp_int32 < 0 )
                {
                  pHandle->QuadDetSign2 = 1;
                }
                else
                {
                  pHandle->QuadDetSign2 = -1;
                }

                /* Decision */
                if (pHandle->QuadDetSign1 > 0)
                {
                  if (pHandle->QuadDetSign2 < 0)
                  {
                    pHandle->HiFrTrAngle =
                      SPD_GetElAngle(& pHandle->pHfiFpSpeedSensor->_Super) + PI_4;
                  }
                  else
                  {
                    error = true;
                  }
                }
                else if (pHandle->QuadDetSign1 < 0)
                {
                  if (pHandle->QuadDetSign2 > 0)
                  {
                    pHandle->HiFrTrAngle =
                      SPD_GetElAngle(& pHandle->pHfiFpSpeedSensor->_Super) - PI_4;
                  }
                  else
                  {
                    error = true;
                  }
                }
                else
                {
                  error = true;
                }

                if (error)
                {
                  HFI_FP_SPD_SetHFState(pHandle->pHfiFpSpeedSensor, true);
                  pHandle->IsHiFrReliable = false;
                  pHandle->HiFrTrState = FAULT;
                }
                else
                {
                  HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, 0);
                  HFI_FP_SPD_SetElAngle(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrAngle);
                  HFI_FP_SPD_SetAvrgMecSpeed01Hz(pHandle->pHfiFpSpeedSensor, 0);

                  pHandle->HiFrTrState = DECAY1;
                  pHandle->StateDuration = 0u;
                }
              }
            }
          }
          else
          {
            HFI_FP_SPD_SetHFState(pHandle->pHfiFpSpeedSensor, true);
            pHandle->IsHiFrReliable = false;
            pHandle->HiFrTrState = FAULT;
          }
          break;

        case DECAY1:
          if (pHandle->StateDuration > pHandle->WaitBeforeNS)
          {
            pHandle->HiFrAmpl = pHandle->HiFrAmplScans16V;
            pHandle->HiFrTrState = ORIENT;
            pHandle->StateDuration = 0u;
          }
          break;

        case ORIENT:
          if (IqdHF.qI_Component2 > 0)
          {
            pHandle->aIdSatDetPos[pHandle->IdSatDetIndex] = IqdHF.qI_Component2;
          }
          else
          {
            pHandle->aIdSatDetNeg[pHandle->IdSatDetIndex] = IqdHF.qI_Component2;
            pHandle->IdSatDetIndex ++;
          }

          if (pHandle->IdSatDetIndex == pHandle->NSMaxDetPoints)
          {
            /* SATURATION TEST COMPLETED, ORIENTATION CHECK NOW */
            pHandle->IdSatDetIndex = pHandle->NSDetPointsSkip;

            while (pHandle->IdSatDetIndex < pHandle->NSMaxDetPoints)
            {
              pHandle->IdSatPosSum += pHandle->aIdSatDetPos[pHandle->IdSatDetIndex];
              pHandle->IdSatNegSum +=  pHandle->aIdSatDetNeg[pHandle->IdSatDetIndex];
              pHandle->IdSatDetIndex ++;
            }

            pHandle->IdSatNegSum  *= -1;

            if (pHandle->RevertDirection)
            {
              if (pHandle->IdSatPosSum < pHandle->IdSatNegSum )
              {
                HFI_FP_SPD_SetElAngle(pHandle->pHfiFpSpeedSensor,
                                      (int16_t)(pHandle->HiFrTrAngle - INT64_MIN));
                pHandle->HiFrTrAngle -= INT64_MIN;
              }
            }
            else if (pHandle->IdSatPosSum > pHandle->IdSatNegSum )
            {
              HFI_FP_SPD_SetElAngle(pHandle->pHfiFpSpeedSensor,
                                    (int16_t)(pHandle->HiFrTrAngle - INT64_MIN));
              pHandle->HiFrTrAngle -= INT64_MIN;
            }
            else
            {}

            pHandle->SaturationDifference =  (int16_t)(pHandle->IdSatPosSum - pHandle->IdSatNegSum);

            pHandle->HiFrTrState = DECAY2;
            pHandle->StateDuration = 0u;
          }
          break;

        case DECAY2:
          if (pHandle->StateDuration > pHandle->WaitAfterNS)
          {
            pHandle->HiFrAmpl = pHandle->HiFrAmpls16V;
            HFI_FP_SPD_SetElAngle(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrAngle + pHandle->StepAngle);

            pHandle->HiFrTrState = SYNCH;
            pHandle->StateDuration = 0u;
          }
          break;

        case SYNCH:
          if (pHandle->StateDuration > pHandle->WaitSynch)
          {
            if (IqdHF.qI_Component1 > 0)
            {
              if (IqdHF.qI_Component2 > 0)
              {
                pHandle->SineSign = 1;
              }
              else
              {
                pHandle->SineSign = -1;
              }
            }
            else
            {
              if (IqdHF.qI_Component2 > 0)
              {
                pHandle->SineSign = -1;
              }
              else
              {
                pHandle->SineSign = 1;
              }
            }

            pHandle->HiFrTrState = TRACK;
            pHandle->StateDuration = 0u;
          }
          break;

        case TRACK:
          idfiltered = idfiltered * pHandle->SineSign * (-1);

          pHandle->HiFrTrSpeed = PI_Controller(&(pHandle->PIDRegulator),
                                                 (int32_t)(- idfiltered));
          HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrSpeed);

          if (pHandle->StateDuration > pHandle->WaitTrack)
          {
            int16_t elangletest = SPD_GetElAngle(& pHandle->pHfiFpSpeedSensor->_Super);
            int16_t angledifference = elangletest - pHandle->HiFrTrAngle;
            int16_t saturationdifferenceabsolute = pHandle->SaturationDifference;
            if (angledifference < 0)
            {
              angledifference = -angledifference;
            }

            if (saturationdifferenceabsolute < 0)
            {
              saturationdifferenceabsolute = -saturationdifferenceabsolute;
            }

            if (saturationdifferenceabsolute > pHandle->MinSaturationDifference)
            {
              if (angledifference < pHandle->MaxangleDiff)
              {
                pHandle->IsHiFrReliable = true;
              }
              else
              {
                pHandle->IsHiFrReliable = false;
              }
            }
            else
            {
              pHandle->IsHiFrReliable = false;
            }

            HFI_FP_SPD_SetHFState(pHandle->pHfiFpSpeedSensor, true);
            pHandle->StateDuration = 0u;
            if (pHandle->IsHiFrReliable)
            {
              pHandle->HiFrTrState = READY;
            }
            else
            {
              pHandle->HiFrTrState = FAULT;
            }
          }
          break;

        case READY:
          idfiltered = idfiltered * pHandle->SineSign * (-1);

          pHandle->HiFrTrSpeed = PI_Controller(&(pHandle->PIDRegulator),
                                                 (int32_t)(- idfiltered));
          HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrSpeed);

          if (pHandle->STMRUN == true) /* received command from HFI_FP_STMRUN function*/
          {
            PID_SetIntegralTerm(pHandle->pPIDq, 0);
            PID_SetIntegralTerm(pHandle->pPIDd, 0);
            pHandle->FOCVars->Vqd.qV_Component1 = 0;
            pHandle->FOCVars->Vqd.qV_Component2 = 0;
            pHandle->FOCVars->Valphabeta.qV_Component1 = 0;
            pHandle->FOCVars->Valphabeta.qV_Component2 = 0;

            pHandle->HiFrTrState = GO;
          }
          break;

        case GO:
          idfiltered = idfiltered * pHandle->SineSign * (-1);

          pHandle->HiFrTrSpeed = PI_Controller(&(pHandle->PIDRegulator),
                                                 (int32_t)(- idfiltered));
          HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrSpeed);
          break;

        case RESTART:
          pHandle->HiFrAmpl += pHandle->HiFrAmplStep;
          if (pHandle->HiFrAmpl > pHandle->HiFrAmpls16V)
          {
            pHandle->HiFrAmpl = pHandle->HiFrAmpls16V;
            pHandle->HiFrTrState = RESTARTSYNCH;
            pHandle->StateDuration = 0u;
          }
          break;

        case RESTARTSYNCH:
          if (pHandle->StateDuration > pHandle->WaitSynch)
          {
            if (IqdHF.qI_Component1 > 0)
            {
              if (IqdHF.qI_Component2 > 0)
              {
                pHandle->SineSign = -1;
              }
              else
              {
                pHandle->SineSign = 1;
              }
            }
            else
            {
              if (IqdHF.qI_Component2 > 0)
              {
                pHandle->SineSign = 1;
              }
              else
              {
                pHandle->SineSign = -1;
              }
            }

            pHandle->HiFrTrState = RESTARTREADY;
            HFI_FP_SPD_SetHFState(pHandle->pHfiFpSpeedSensor, true);
            pHandle->IsHiFrReliable = true;
            pHandle->StateDuration = 0u;
          }
          break;

        case RESTARTREADY:
          if (pHandle->STMRUN == true) /* received command from HFI_FP_STMRUN function*/
          {
            int16_t hifrforcedspeed = SPD_GetElSpeedDpp(& pHandle->pHfiFpSpeedSensor->_Super);

            idfiltered = idfiltered * pHandle->SineSign * (-1);
            PID_SetIntegralTerm(&(pHandle->PIDRegulator), (int32_t)hifrforcedspeed *
                                (int32_t)PID_GetKIDivisor(&(pHandle->PIDRegulator)));

            pHandle->HiFrTrSpeed = PI_Controller(&(pHandle->PIDRegulator),
                                                   (int32_t)(- idfiltered));
            HFI_FP_SPD_SetElSpeedDpp(pHandle->pHfiFpSpeedSensor, pHandle->HiFrTrSpeed);

            pHandle->HiFrTrState = GO;
          }
          break;

        case FAULT:
          break;

        default:
          break;
      }
    }
  }
}

/**
  * @brief  Disable HF voltage gen when STO takes over
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval none
  */
void HFI_FP_DisHFGeneration(HFI_FP_Ctrl_Handle_t *pHandle)
{
  pHandle->HiFrGenEnabled = false;
  pHandle->HiFrTrState = OFFSTATE;
  PID_SetUpperOutputLimit(pHandle->pPIDq, 32767);
  PID_SetUpperIntegralTermLimit(pHandle->pPIDq, 32767 * (int32_t)PID_GetKIDivisor(pHandle->pPIDq));
}

/**
  * @brief  Gives information about HFI initial angle measurement stage completion
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval bool; true if HFI has locked on rotor angle;
  *         false if HFI ended with an error;
  */
bool HFI_FP_STMRUN(HFI_FP_Ctrl_Handle_t *pHandle)
{
  bool retval;

  if (pHandle->IsHiFrReliable == true)
  {
    pHandle->STMRUN = true;
    retval = true;
  }
  else
  {
    retval = false;
  }

  return (retval);
}

/**
  * @brief  It returns the rotor angle lock value
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval int16_t Rotor angle lock value
  */
int16_t HFI_FP_GetRotorAngleLock(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return pHandle->DebugHifrAngle;
}

/**
  * @brief  It returns the saturation difference measured during the last
  *         north/south identification stage.
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval int16_t Saturation difference measured during the last north/south
  *         identification stage.
  */
int16_t HFI_FP_GetSaturationDifference(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return pHandle->SaturationDifference;
}

/**
  * @brief  It set the min saturation difference used to validate the
  *         north/south identification stage.
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @param  hMinSaturationDifference Min Saturation difference used to validate
  *         the north/south identification stage.
  * @retval none
  */
void HFI_FP_SetMinSaturationDifference(HFI_FP_Ctrl_Handle_t *pHandle, int16_t MinSaturationDifference)
{
  pHandle->MinSaturationDifference = MinSaturationDifference;
}

/**
  * @brief  It return the quantity that shall be put in the DAC to tune the HFI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval int16_t HFI current
  */
int16_t HFI_FP_GetCurrent(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return pHandle->DebugHifrCurrent;
}

/**
  * @brief  It restarts the HF injection before the swap between STO-HFI;
  *         To be called continuously to check if the HFI insertion phase
  *         has been completed.
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval bool. true if HFI insertion has been completed;
  *         false if HFI insertion has not been completed yet
  */
bool HFI_FP_Restart(HFI_FP_Ctrl_Handle_t *pHandle)
{
  bool retval = false;

  if (pHandle->HiFrTrState == OFFSTATE)
  {
    HFI_FP_Clear(pHandle);
    pHandle->HiFrAmpl = 0;
    pHandle->HiFrTrState = RESTART;
    pHandle->HiFrGenEnabled = true;
  }
  else if (pHandle->HiFrTrState == RESTARTREADY)
  {
    retval = true;
  }
  else {}

  return retval;
}

/**
  * @brief  It starts the HF injection; to be called
  *         continuously to check the status of angle detection phase.
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval bool. true if HFI angle detection has been completed;
  *         false if HFI angle detection has not been completed yet
  */
bool HFI_FP_Start(HFI_FP_Ctrl_Handle_t *pHandle)
{
  bool retval = false;

  if (pHandle->HiFrTrState == OFFSTATE)
  {
    HFI_FP_Clear(pHandle);
    pHandle->HiFrAmpl = pHandle->HiFrAmpls16V;
    pHandle->HiFrTrState = SCAN;
    pHandle->HiFrGenEnabled = true;
  }
  else if (pHandle->HiFrTrState == READY)
  {
    retval = true;
  }
  else if (pHandle->HiFrTrState == FAULT)
  {
    retval = true;
    pHandle->HiFrTrState = OFFSTATE;
  }
  else {}

  return retval;
}

/**
  * @brief  It updates the Kp gain of PLL PI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @param  float32_t New Kp gain
  * @retval None
  */
void HFI_FP_PLLSetKP(HFI_FP_Ctrl_Handle_t *pHandle, float32_t KpGain)
{
  pHandle->PLLKp = KpGain;
}

/**
  * @brief  It updates the Ki gain of PLL PI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @param  float32_t New Ki gain
  * @retval None
  */
void HFI_FP_PLLSetKI(HFI_FP_Ctrl_Handle_t *pHandle, float32_t KiGain)
{
  pHandle->PLLKi = KiGain;
}

/**
  * @brief  It returns the Kp gain of PLL PI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval float32_t Kp gain
  */
float32_t HFI_FP_PLLGetKP(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return pHandle->PLLKp;
}

/**
  * @brief  It returns the Ki gain of PLL PI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval float32_t Ki gain
  */
float32_t HFI_FP_PLLGetKI(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return pHandle->PLLKi;
}

/**
  * @brief  It returns the Track PI
  * @param  pHandle related HFI_FP_Ctrl Handle
  * @retval address of Track PI handle
  */
PID_Handle_t* HFI_FP_GetPITrack(HFI_FP_Ctrl_Handle_t *pHandle)
{
  return &pHandle->PIDRegulator;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
