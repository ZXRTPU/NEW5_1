/**
 * @file VideoTransmitter.h
 * @author wexhi (wexhi@qq.com)
 * @brief  ����ͼ����·�Ľ����Լ�����
 * @version 0.1
 * @date 2024-03-27
 * @todo ͼ����·Ӧ������ң�������Ƶĸ�ӹ���������Ǻϲ���ת��
 *
 * @copyright Copyright (c) 2024 CQU QianLi EC 2024 all rights reserved
 *
 */

#ifndef VIDEOTRANSMITTER_H
#define VIDEOTRANSMITTER_H

#include "stdint.h"
#include "main.h"
#include "usart.h"

#include "remote_control.h"
#include "referee_protocol.h"

#pragma pack(1)

// ͼ��+ң����+����
typedef struct
{
    xFrameHeader FrameHeader;        // ���յ���֡ͷ��Ϣ
    uint16_t CmdID;                  // ������
    custom_robot_data_t custom_data; // �Զ�������
    remote_control_t key_data;       // ң��������

    Key_t key[3]; // ��Ϊλ���ļ�������,�ռ����8��,�ٶ�����16~��

    uint8_t key_count[3][16];
} Video_ctrl_t;

#pragma pack()

Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);

#endif // !VIDEOTRANSMITTER_H