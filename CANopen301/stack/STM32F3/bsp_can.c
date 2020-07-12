/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  ljb
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   can驱动（回环模式）
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

#include "bsp_can.h"
#include "CANopen.h"
#include <string.h>
/* public pertory */

CAN_RxHeaderTypeDef	RxMsg;
uint8_t					CanRxData[8]={0,0,0,0,0,0,0,0};
volatile	int8_t					CAN_RxStatus	=	0;

/*
*
*/

void MX_CAN_Init(CAN_HandleTypeDef 		*pCan,uint16_t	CANbitRate)
{
    /* Init CAN controler */
//    CAN_InitTypeDef CAN_InitStruct;
	pCan->Instance = CAN;
	
    // CAN_DeInit(CANmodule->CANbaseAddress);
	//HAL_CAN_DeInit(CANmodule->CANbaseAddress);
	HAL_CAN_DeInit(pCan);

	/* 
	*	CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 36 MHz) 
	*	BTR-BRP 波特率分频器  定义了时间单元的时间长度 36 /(2+9+7)/2 = 1 Mbps
	*/
	// CAN Baudrate = 500kbps (CAN clocked at 36 MHz),sure  okay 	by lijiabo
	pCan->Init.Prescaler	= 6;
	pCan->Init.SyncJumpWidth = CAN_SJW_1TQ;
	pCan->Init.TimeSeg1 = CAN_BS1_5TQ;
	pCan->Init.TimeSeg2 = CAN_BS2_6TQ;
	
	/*
    switch (CANbitRate) {
        case 1000: pCan->Init.Prescaler = 2;
            break;
        case 500: pCan->Init.Prescaler = 4;
            break;
        default:
        case 250: pCan->Init.Prescaler = 8;
            break;
        case 125: pCan->Init.Prescaler = 16;
            break;
        case 100: pCan->Init.Prescaler = 20;
            break;
        case 50: pCan->Init.Prescaler = 40;
            break;
        case 20: pCan->Init.Prescaler = 100;
            break;
        case 10: pCan->Init.Prescaler = 200;
            break;
    }

	pCan->Init.SyncJumpWidth = CAN_SJW_2TQ;
	pCan->Init.TimeSeg1 = CAN_BS1_9TQ;
	pCan->Init.TimeSeg2 = CAN_BS2_7TQ;
	*/

	pCan->Init.Mode = CAN_MODE_NORMAL;
	pCan->Init.TimeTriggeredMode = DISABLE;
	pCan->Init.AutoBusOff = DISABLE;
	pCan->Init.AutoWakeUp = DISABLE;
	pCan->Init.AutoRetransmission = DISABLE;
	pCan->Init.ReceiveFifoLocked = DISABLE;
	pCan->Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(pCan) != HAL_OK)
	  {
		_Error_Handler(__FILE__, __LINE__);
	  }
	  
//    CAN_FilterRegister_TypeDef CAN_FilterInitStruct;	
		CAN_FilterTypeDef	CAN_FilterInitStruct;
    memset(&CAN_FilterInitStruct, 0, sizeof (CAN_FilterInitStruct));
    //CAN_FilterInitStruct.FilterNumber = 0;
		CAN_FilterInitStruct.FilterBank	=	13; 					/*it is FilterNumber */
    CAN_FilterInitStruct.FilterIdHigh = 0;
    CAN_FilterInitStruct.FilterIdLow = 0;
    CAN_FilterInitStruct.FilterMaskIdHigh = 0;
    CAN_FilterInitStruct.FilterMaskIdLow = 0;
    CAN_FilterInitStruct.FilterFIFOAssignment = CAN_FILTER_FIFO0; // pouzivame jen FIFO0	CAN_Filter_FIFO0
    CAN_FilterInitStruct.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStruct.FilterScale = CAN_FILTERSCALE_32BIT;	
    CAN_FilterInitStruct.FilterActivation = ENABLE;
    //CAN_FilterInit(&CAN_FilterInitStruct);
		HAL_CAN_ConfigFilter(pCan, &CAN_FilterInitStruct);

	
    // /* Can_init function of ST Driver puts the controller into the normal mode */
    CAN_ITConfig(pCan->Instance, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING), ENABLE);
		//HAL_CAN_ActivateNotification(pCan,CAN_IT_RX_FIFO0_MSG_PENDING);
		//HAL_CAN_ActivateNotification(pCan,CAN_IT_TX_MAILBOX_EMPTY);					
		HAL_CAN_Start(pCan);
}		


/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */	 

void CAN_SetMsg(CAN_TxHeaderTypeDef *TxMessage)
{	  

  TxMessage->StdId=0x00;						 
//  TxMessage->ExtId=0x1314;					 //使用的扩展ID
  TxMessage->IDE=CAN_ID_EXT;					 //扩展模式
  TxMessage->RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage->DLC=8;							 //数据长度为8字节
	
}


/**
  * @brief  Enables or disables the specified CANx interrupts.
  * @param  CANx: where x can be 1,2 or 3 to select the CAN peripheral.
  * @param  CAN_IT: specifies the CAN interrupt sources to be enabled or disabled.
  *          This parameter can be: 
  *            @arg CAN_IT_TME: Transmit mailbox empty Interrupt 
  *            @arg CAN_IT_FMP0: FIFO 0 message pending Interrupt 
  *            @arg CAN_IT_FF0: FIFO 0 full Interrupt
  *            @arg CAN_IT_FOV0: FIFO 0 overrun Interrupt
  *            @arg CAN_IT_FMP1: FIFO 1 message pending Interrupt 
  *            @arg CAN_IT_FF1: FIFO 1 full Interrupt
  *            @arg CAN_IT_FOV1: FIFO 1 overrun Interrupt
  *            @arg CAN_IT_WKU: Wake-up Interrupt
  *            @arg CAN_IT_SLK: Sleep acknowledge Interrupt  
  *            @arg CAN_IT_EWG: Error warning Interrupt
  *            @arg CAN_IT_EPV: Error passive Interrupt
  *            @arg CAN_IT_BOF: Bus-off Interrupt  
  *            @arg CAN_IT_LEC: Last error code Interrupt
  *            @arg CAN_IT_ERR: Error Interrupt
  * @param  NewState: new state of the CAN interrupts.
  * @note   CAN3 peripheral is available only for STM32F413_423xx devices
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IT(CAN_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected CANx interrupt */
    CANx->IER |= CAN_IT;
  }
  else
  {
    /* Disable the selected CANx interrupt */
    CANx->IER &= ~CAN_IT;
  }
}


/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//HAL_StatusTypeDef	hst;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
	CO_CANinterrupt_Rx(CO->CANmodule[0]);
	CAN_RxStatus	=	'R';
/*	
	 hst = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg, CanRxData);
	if ( hst == HAL_OK )
		CAN_RxStatus	=	'R';
*/
	
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_TxMailbox0CompleteCallback could be implemented in the
            user file
   */
	CO_CANinterrupt_Tx(CO->CANmodule[0]);
	
}


/**
  * @brief  Sleep callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_SleepCallback could be implemented in the user file
   */
	
	
}

/**
  * @brief  WakeUp from Rx message callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_WakeUpFromRxMsgCallback could be implemented in the
            user file
   */
	
	
}


/**
  * @brief  Error CAN callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_ErrorCallback could be implemented in the user file
   */
	
	CO_CANverifyErrors(CO->CANmodule[0]);
	
	
}

/**
  * @brief  Transmission Mailbox 0 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_TxMailbox0AbortCallback could be implemented in the
            user file
   */
}


/**************************END OF FILE************************************/











