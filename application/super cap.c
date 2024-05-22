#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include  "drv_can.h"
#include "judge.h"
#include "Chassis_task.h"
#include "super_cap.h"
//#include "memory.h"
#include "stdlib.h"
#include "VideoTransmitter.h"

extern uint8_t Hero_level;
extern int flag[1];
extern float powerdata[4];
extern RC_ctrl_t rc_ctrl;
extern Video_ctrl_t video_ctrl[2];
extern CAN_HandleTypeDef hcan2;

SuperCapTx_t SuperCapTx;
SuperCapRx_t SuperCapRx;
extern referee_infantry_t referee_infantry;
uint8_t supercap_mode_Tx = 0;

static uint8_t tx2up[8];		 // 用于发送数据给上位机

int supercap_flag1 = 0;

void read_keyboard();

void Supercap_task(void const *argument)
{
   	while (1)
	{
		supercap_flag1++; 
		
		read_keyboard(); 
		SuperCapSet(referee_infantry.buffer_energy, referee_infantry.chassis_power_limit, supercap_mode_Tx);
		SuperCapSend(SuperCapTx);
		osDelay(10);
	}
}

void SuperCapSet(uint16_t buffer, uint16_t power, uint8_t state)
{
    SuperCapTx.buffer  = buffer;
    SuperCapTx.power  = power;
    SuperCapTx.state  = state;
}

void SuperCapSend(SuperCapTx_t TxData)
{
	CAN_TxHeaderTypeDef tx_header;

	uint8_t send_buffer[5];
	tx_header.StdId = 0x302;
	tx_header.IDE = CAN_ID_STD;	  // 标准帧
	tx_header.RTR = CAN_RTR_DATA; // 数据帧

	tx_header.DLC = 5; // 发送数据长度（字节）
	send_buffer[0] = (TxData.buffer >> 8) & 0xff;
	send_buffer[1] = TxData.buffer & 0xff;
	send_buffer[2] = (TxData.power >> 8) & 0xff;
	send_buffer[3] = TxData.power & 0xff;
	send_buffer[4] = TxData.state;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) // 等待发送邮箱空闲
	{

	}
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_buffer, (uint32_t *)CAN_TX_MAILBOX0);  
}

void read_keyboard()
{
	if (rc_ctrl.rc.s[0])
	{

	}
	//图传链路
	else
	{
		switch (video_ctrl[TEMPV].key_count[V_KEY_PRESS][V_Key_C] % 2)
		{
		case 0:
			supercap_mode_Tx = SUPERCAP_AUTO;
			break;
		case 1:
			supercap_mode_Tx = SUPERCAP_OFF;
			break;
		default:
			supercap_mode_Tx = SUPERCAP_AUTO;
			break;
		}
	}
}

// void super_cap(void const *pvParameters)
// {
// 	for(;;)
// 	{  
// 		if(Hero_level==1)
// 		{
// 			supercap(5500);
// 		}
// 		if(Hero_level==2)
// 		{
// 			supercap(6000);
// 		}
// 		if(Hero_level==3)
// 		{
// 			supercap(6500);
// 		}
// 		else
// 		{
// 			supercap(5500);
// 		}
// 			//can_remote(sbuss_buf,0x33);
// 			//CAN_rc_forward(power,power1);
// 				osDelay(1);
// 	}

// }

