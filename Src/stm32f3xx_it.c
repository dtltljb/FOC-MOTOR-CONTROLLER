/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32f3xx_ll_usart.h"
#include "UITask.h"
#include "mc_config.h"
#include "user_config.h"
#include "uart1_frame_communication_protocol.h"

#include "CANopen.h"
//extern	UART_HandleTypeDef huart1;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles CAN TX and USB high priority interrupts.
*/
void USB_HP_CAN_TX_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_CAN_TX_IRQn 0 */

  /* USER CODE END USB_HP_CAN_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_HP_CAN_TX_IRQn 1 */

  /* USER CODE END USB_HP_CAN_TX_IRQn 1 */
}
/**
* @brief This function handles CAN RX0 and USB low priority interrupts.
*/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 1 */
}

/**
* @brief This function handles CAN RX1 interrupt.
*/
void CAN_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN_RX1_IRQn 0 */

  /* USER CODE END CAN_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN_RX1_IRQn 1 */

  /* USER CODE END CAN_RX1_IRQn 1 */
}

/**
* @brief This function handles CAN SCE interrupt.
*/
void CAN_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN_SCE_IRQn 0 */

  /* USER CODE END CAN_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN_SCE_IRQn 1 */

  /* USER CODE END CAN_SCE_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint8_t tmp;
	 if (LL_USART_IsActiveFlag_RXNE(USART1)) /* Valid data have been received */
  {
    uint16_t retVal;
    retVal = *(uint16_t*)U1FCP_RX_IRQ_Handler(&pUSART1,LL_USART_ReceiveData8(USART1));
    if (retVal == 1)
    {
      UI_Serial1CommunicationTimeOutStart();
    }
    if (retVal == 2)
    {
      UI_Serial1CommunicationTimeOutStop();
    }

  }

  else if (LL_USART_IsActiveFlag_TXE(USART1))
  {
    U1FCP_TX_IRQ_Handler(&pUSART1);

  }
  else if (LL_USART_IsActiveFlag_ORE(USART1)) /* Overrun error occurs */
  {
    /* Send Overrun message */
    U1FCP_OVR_IRQ_Handler(&pUSART1);
    LL_USART_ClearFlag_ORE(USART1); /* Clear overrun flag */
    UI_Serial1CommunicationTimeOutStop();
 
  }
	
	
  /* USER CODE END USART1_IRQn 0 */

/* USER CODE BEGIN USART1_IRQn 1 */

// if (LL_USART_IsActiveFlag_RXNE(USART1)) /* Valid data have been received */
// {
//	tmp	=	LL_USART_ReceiveData8(USART1);
//	LL_USART_TransmitData8(USART1,tmp); 
// }
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
