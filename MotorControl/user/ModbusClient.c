/***********************************************Copyright (c)*********************************************
**																						
**------------------------------------------------File Info-----------------------------------------------
** File name:						ModbusUser.c
** Last modified Date:	2010-04-27
** Last Version:				1.0
** Descriptions:				MODBUS通讯文件
** Dependencies:				none
** Processor:						LPC23xx
** Compiler:						ARM Developer Suite v1.2
**--------------------------------------------------------------------------------------------------------
** Author		Date					Version		Comment
** dtlt			2010-04-23		1.0				Original
** 
*********************************************************************************************************/

#include 		<stdio.h>
#include 		<stdlib.h>
#include 		<stdint.h>

#include 		"crc16.h"
#include 		"ModbusClient.h"
#include 		"motor_control_protocol.h"

#ifndef __MODBUS_USER_C
#define __MODBUS_USER_C
/*********************************************************************************************************
    Modbus协议功能码    
*********************************************************************************************************/
#define MB_FUN_RCS			01			//01 Read Coil Status
#define MB_FUN_RIS			02			//02 Read Input Status
#define MB_FUN_RHR			03			//03 Read Holding Registers
#define MB_FUN_RIR			04			//04 Read Input Registers
#define MB_FUN_FSC			05			//05 Force Single Coil					
#define MB_FUN_PSR			06			//06 Preset Single Register					
//#define MB_FUN_RES			07			//07 Read Exception Status					
//#define MB_FUN_FCEC			11			//11 (0B Hex) Fetch Comm Event Ctrl
//#define MB_FUN_FCEL			12			//12 (0C Hex) Fetch Comm Event Log
#define MB_FUN_FMC			15			//15 (0F Hex) Force Multiple Coils
#define MB_FUN_PMR			16			//16 (10 Hex) Preset Multiple Regs
//#define MB_FUN_RSID			17			//17 (11 Hex) Report Slave ID					
#define MB_FUN_RGR			20			//20 (14Hex) Read General Reference
//#define MB_FUN_WGR			21			//21 (15Hex) Write General Reference
//#define MB_FUN_MWR			22			//22 (16Hex) Mask Write 4X Register
//#define MB_FUN_RWR			23			//23 (17Hex) Read/Write 4X Registers
#define MB_FUN_RFQ			24			//24 (18Hex) Read FIFO Queue

uint8_t mbAddr;
//STR_MB_MODE mbMode;
//*************************** private constant define ***************************


/**********************************************************************
**函数原型： void  MB_CFGInit(void)
**入口参数:	无
**出口参数:	无
**返 回 值：无
**说    明：MODBUS协议初始化程序
************************************************************************/
void  ModBus_CFGInit(void){
    mbAddr = 1;

}

/**********************************************************************
**函数原型： uint8_t RTU_RHR(uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen,uint16_t lenLim)
**入口参数:	*pRecvBuf 	:接收数据指针
**			*pSendBuf 	:发送数据指针
**			*pLen 		:接收到的字节数
**			lenLim		:发送数据的长度限值（最大值，包括从地址到CRC校验之前的全部长度）
**出口参数:	*pSendBuf 	:填充到发送数据指针中的数据
**			*pLen 		:返回的字节数
**返 回 值：0 成功，其它 异常
**说    明：读保持寄存器，03功能码,RTU指令格式
************************************************************************/
static uint8_t RTU_RHR(MCP_Handle_t *pHandle,uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen,uint16_t lenLim)
{
//接收数据结构：macAddr+function+regAddr+dataLen+crc
//				1byte	1byte	2byte	 2byte	2byte
//				i		i+1		i+2		i+4		i+6
    uint8_t     *pSend;
    uint8_t     *pRecv;
    bool	res	= true;
	uint16_t	regAddr;
	uint16_t	w,dataLen;
	uint32_t	wValue;
	pSend = pSendBuf;
	pRecv = pRecvBuf;
	regAddr = pRecv[3] + (((uint16_t)pRecv[2]) << 8);
	dataLen = (uint16_t)pRecv[5] + (((uint16_t)pRecv[4]) << 8);
	//w = dataLen*2;
	w=4;
	*pSend++ = pRecv[0];
    if((*pLen<6)||(w>(lenLim-3))){
		*pSend++ = pRecv[1] + 0x80;
		*pSend++ = ILLEGAL_LEN;
		*pLen = pSend-pSendBuf;
		return ILLEGAL_LEN;
	}
	
	*pSend++ = pRecv[1];				//功能码
	*pSend++ = w;								//字节数量
	 wValue	=	UI_GetReg(&pHandle->_Super,(MC_Protocol_REG_t)(regAddr>>1));
	*pSend++ = (uint8_t)(wValue>>24);
	*pSend++ = (uint8_t)(wValue>>16);
	*pSend++ = (uint8_t)(wValue>>8);
	*pSend++ = (uint8_t)(wValue>>0);
	*pLen = pSend-pSendBuf;
	 return MB_NO_ERR;

}

/**********************************************************************
**函数原型： uint8_t RTU_PMR(uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen)
**入口参数:	*pRecvBuf 	:接收数据指针
**			*pSendBuf 	:发送数据指针
**			*pLen 		:接收到的字节数
**			lenLim		:发送数据的长度限值（最大值，包括从地址到CRC校验的全部长度）
**出口参数:	*pSendBuf 	:填充到发送数据指针中的数据
**			*pLen 		:返回的字节数
**返 回 值：0 成功，其它 异常
**说    明：预置多个寄存器，16功能码,RTU指令格式
************************************************************************/
static uint8_t RTU_PMR(MCP_Handle_t *pHandle,uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen)
{
//接收数据结构：macAddr+function+regAddr+dataLen+byteCnt+data+...+crc
//				1byte	1byte	2byte	 2byte	1byte    2byte
//				i		i+1		i+2		i+4		i+6      i+7  ... i+7+byteCnt
	uint8_t	*pSend;
	uint8_t	*pRecv;
	bool	res;
	uint16_t	regAddr;
	uint16_t	dataLen;
	uint32_t	wValue	=	0;
	pSend = pSendBuf;
	pRecv = pRecvBuf;
	*pSend++ = pRecv[0];
	dataLen = (uint16_t)pRecv[5] + (((uint16_t)pRecv[4]) << 8);//此处为要写入保持寄存器的值
	if(*pLen<(dataLen*2+7)){
		*pSend++ = pRecv[1] + 0x80;
		*pSend++ = ILLEGAL_LEN;
		*pLen = pSend-pSendBuf;
		return ILLEGAL_LEN;
	}
	regAddr = pRecv[3] + (((uint16_t)pRecv[2]) << 8);
	wValue	=	(uint32_t)pRecv[7]<<24 ;
	wValue	|=	(uint32_t)pRecv[8]<<16 ;
	wValue	|=	(uint32_t)pRecv[9]<<8 ;
	wValue	|=	(uint32_t)pRecv[10]<<0 ;
	res	=	UI_SetReg(&pHandle->_Super,(MC_Protocol_REG_t)(regAddr>>1),wValue);
	if(!res)			//16预制多个寄存器，与03对应
	{
		*pSend++ = pRecv[1] + 0x80;
		*pSend++ = SLAVE_DEVICE_FAILURE;
		*pLen = pSend-pSendBuf;
		return SLAVE_DEVICE_FAILURE;
	}
	else{
	  	*pSend++ = pRecv[1];			//功能码
		*pSend++ = pRecv[2];			//起始地址
		*pSend++ = pRecv[3];
		*pSend++ = pRecv[4];			//数据长度
		*pSend++ = pRecv[5];
		*pLen = pSend-pSendBuf;
		return MB_NO_ERR;
	}
}
/**********************************************************************
**函数原型： uint8_t RTU_ERR(uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen)
**入口参数:	*pRecvBuf 	:接收数据指针
**			*pSendBuf 	:发送数据指针
**			*pLen 		:接收到的字节数
**			lenLim		:发送数据的长度限值（最大值，包括从地址到CRC校验的全部长度）
**出口参数:	*pSendBuf 	:填充到发送数据指针中的数据
**			*pLen 		:返回的字节数
**返 回 值：0 成功，其它 异常
**说    明：功能码不能识别,RTU指令格式
************************************************************************/
static uint8_t RTU_ERR(uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen)
{
//接收数据结构：macAddr+function+regAddr+crc
//				1byte	1byte	2byte	 2byte
//				i		i+1		i+2		i+4
	uint8_t	*pSend;
	uint8_t	*pRecv;
	
	pSend = pSendBuf;
	pRecv = pRecvBuf;
	*pSend++ = pRecv[0];				//设备地址
	*pSend++ = pRecv[1]|0x80;			//功能码
	*pSend++ = ILLEGAL_FUNCTION;
	*pLen = pSend-pSendBuf;
	return ILLEGAL_FUNCTION;
}

/**********************************************************************
**函数原型： uint8_t RtuModbusParse(uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen)
**入口参数:	*pRecvBuf 	:接收数据指针
**			*pSendBuf 	:发送数据指针
**			*pLen 		:接收到的字节数
**			lenLim		:发送数据的长度限值（最大值，包括从地址到CRC校验的全部长度）
**出口参数:	*pSendBuf 	:填充到发送数据指针中的数据
**			*pLen 		:发送的字节数
**返 回 值：见 函数返回出错代码
**说    明：modbus client 解析程序,仅支持读、写2个寄存器内容,mc_extended_api.h寄存器编号×2对应modbus寄存器编号
************************************************************************/
uint8_t RtuModbusParse(MCP_Handle_t *pHandle,uint8_t *pRecvBuf,uint8_t *pSendBuf,uint16_t *pLen,uint16_t lenLim)
{
	uint8_t err;
	uint16_t crc;
	crc = pRecvBuf[*pLen-2]+(((uint16_t)pRecvBuf[*pLen-1])<<8);
	if(crc != crc16(pRecvBuf,*pLen-2))	{
		*pLen = 0;
		return ILLEGAL_CRC;
	}
	else if((*pRecvBuf == mbAddr)||(*pRecvBuf == BROADCAST_ADDRESS))
	{
		switch(pRecvBuf[1])					//功能码判断
		{
			case MB_FUN_RHR:				//03读保持寄存器
				err = RTU_RHR(pHandle,pRecvBuf,pSendBuf,pLen,lenLim-2);
				break;
//			case MB_FUN_PSR:	  			//06预制单个寄存器
//				err = RTU_PSR(pRecvBuf,pSendBuf,pLen);
//				break;  		  	
			case MB_FUN_PMR:				//16预置多个寄存器
				err = RTU_PMR(pHandle,pRecvBuf,pSendBuf,pLen);
				break;
	  	default:
				err = RTU_ERR(pRecvBuf,pSendBuf,pLen);
				break;
		}
		crc = crc16(pSendBuf,*pLen);
		pSendBuf[(*pLen)++] = crc;
		pSendBuf[(*pLen)++] = crc>>8;
		return err;
	}
	else	{
		*pLen = 0;
		return ILLEGAL_MODBUS_ADDR;
	}
}



#endif // __MODBUS_USER_C
/*********************************************************************************************************
**  End
**********************************************************************************************************/
