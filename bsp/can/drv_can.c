#include "drv_can.h"
#include "VideoTransmitter.h"
#include "super_cap.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define POWERDATA_ID 0x301

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
extern chassis_t chassis;
extern SuperCapRx_t SuperCapRx;

uint8_t rx_data2[8];
uint16_t can_cnt_1 = 0;
int16_t yy;
int16_t Roll0;
int16_t Pitch0;
int16_t Yaw0;
double Yaw1;
float Yaw_top;

float powerdata[4];
uint16_t pPowerdata[8];
uint16_t setpower = 5500;
int canerror = 0;

//图传数据
static uint8_t video_buf[12]; // 图传接收的buffer
uint8_t vision_is_tracking;
uint8_t friction_mode;

void CAN1_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 0;                      // filter 0
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);                         // init can filter
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can的FIFO0中断
  HAL_CAN_Start(&hcan1);                                             // 启动can1
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 14;                     // filter 14
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &can_filter); // init can filter
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2); // 启动can2
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;

  if (hcan->Instance == CAN2)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can1 data

    // 底盤电机信息接收
    if ((rx_header.StdId >= 0x201)     // 201-204
        && (rx_header.StdId <= 0x204)) // 判断标识符，标识符为0x200+ID
    {
      uint8_t index = rx_header.StdId - 0x201; // get motor index by can_id
      chassis.motor_info[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
      chassis.motor_info[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      chassis.motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      chassis.motor_info[index].temp = rx_data[6];
      if (index == 0)
      {
        can_cnt_1++;
      }
    }
  }

  if (hcan->Instance == CAN1)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can2 data

    if (rx_header.StdId == 0x388)                                //IMU   // 上C向下C传IMU数据
    {
      Roll0= ((rx_data[1] << 8) | rx_data[0]);
      Pitch0=((rx_data[3] << 8) | rx_data[2]);
      Yaw0=((rx_data[5] << 8) | rx_data[4]);
      Yaw1=(float)Yaw0*0.01;//小数部分
    }

    if (rx_header.StdId == 0x35)                                   // 上C向下C传IMU数据
    {
        yy = (((rx_data[0] << 8) | rx_data[1])); // yaw
        Yaw_top = yy / 100.0f;
    }

    if (rx_header.StdId == 0x36) // 接收上C传来的图传数据
    {
      memcpy(video_buf, rx_data, 8);
    }

    if (rx_header.StdId == 0x37) // 接收上C传来的图传数据
    {
      memcpy(video_buf + 8, rx_data, 4);
      VideoRead(video_buf);
    }

    if (rx_header.StdId == 0x38) // 接收上C传来的图传数据
    {
      vision_is_tracking = rx_data[0];
      friction_mode = rx_data[1];
    }

    //超级电容数据接收
    if (rx_header.StdId == POWERDATA_ID) // 0x301
    {
      SuperCapRx.voltage = (((uint16_t)rx_data[0] << 8) | rx_data[1]) / 1000;
      SuperCapRx.power = (uint16_t)(rx_data[2] << 8) | rx_data[3] / 1000;
      SuperCapRx.state = rx_data[4];
    }
  }
}

//******************************************* 调用can来传输数据 *******************************************************
void can_remote(uint8_t sbus_buf[], uint8_t can_send_id) 
{
  CAN_TxHeaderTypeDef tx_header;

  tx_header.StdId = can_send_id; // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE = CAN_ID_STD;    // 标准帧
  tx_header.RTR = CAN_RTR_DATA;  // 数据帧
  tx_header.DLC = 8;             // 发送数据长度（字节）

   HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf, (uint32_t *)CAN_TX_MAILBOX0);
}

//******************************************** 调用can来传输超级电容数据 *******************************************************
// void can_remote(uint8_t tx_buff[], CAN_HandleTypeDef *_can_ins, uint32_t can_send_id, uint32_t len) // 调用can来发送遥控器数据
// {
// CAN_TxHeaderTypeDef tx_header;

// tx_header.StdId = can_send_id;                         // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
// tx_header.IDE = CAN_ID_STD;                            // 标准帧
// tx_header.RTR = CAN_RTR_DATA;                          // 数据帧
// tx_header.DLC = len;                                   // 发送数据长度（字节）
// while (HAL_CAN_GetTxMailboxesFreeLevel(_can_ins) == 0) // 等待邮箱空闲
// {
// }
// HAL_CAN_AddTxMessage(_can_ins, &tx_header, tx_buff, (uint32_t *)CAN_TX_MAILBOX0);
// }
// 发送电容数据
// void Send_Cap_Data(CAN_HandleTypeDef *_hcan, float Cap_Vol, float Power_3508, uint8_t state)
// {
//   static CAN_TxHeaderTypeDef TX_MSG;
//   static uint8_t CAN_Send_Data[8];
//   static uint16_t Cap_Vol_send;
//   static uint16_t Dipan_W;
//   uint32_t send_mail_box;

//   Cap_Vol_send = (int16_t)(Cap_Vol * 1000); // 将浮点型*1000，并转化为int16类型
//   Dipan_W = (int16_t)(Power_3508 * 1000);

//   TX_MSG.StdId = CAN_Transmit_Cap; ////电容发送标识符CAN_Transmit_Cap 0x301  //电容信息发送标识符
//   TX_MSG.IDE = CAN_ID_STD;
//   TX_MSG.RTR = CAN_RTR_DATA;
//   TX_MSG.DLC = 0x05;

//   CAN_Send_Data[0] = (Cap_Vol_send >> 8); ////将Cap_Vol_send的高字节移动到低字节位置
//   CAN_Send_Data[1] = Cap_Vol_send;
//   CAN_Send_Data[2] = (Dipan_W >> 8);
//   CAN_Send_Data[3] = Dipan_W;
//   CAN_Send_Data[4] = state;
 

//   HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
// }


//******************************************** 调用can来控制电机 *******************************************************、
// 底盤電機控制
void set_motor_current_chassis(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); // 如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
  tx_header.IDE = CAN_ID_STD;                            // 标准帧
  tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 雲臺電機控制
void set_motor_current_gimbal(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 雲臺電機控制 -- CAN2
void set_motor_current_gimbal2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 發射機構電機控制
void set_motor_current_shoot(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); // 如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 试运行代码，一步解决所有电机控制
void set_curruent(uint32_t motor_range, CAN_HandleTypeDef can_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = motor_range; // 控制电机的ID号
  // tx_header.IDE = CAN_ID_STD;    // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;  // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&can_id, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

//*************************************解析图传数据************************************************