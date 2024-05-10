#include "VideoTransmitter.h"
#include "bsp_usart.h"
#include "drv_can.h"
#include "daemon.h"
#include "string.h"
#include "crc_ref.h"
#include "referee_protocol.h"
#include "remote_control.h"
#include "ins_task.h"

#define RE_RX_BUFFER_SIZE 255u // ����ϵͳ���ջ�������С

Video_ctrl_t video_ctrl[2]; // ���ڴ洢ͼ����·�Ŀ�������,[0]:��ǰ����TEMP,[1]:��һ�ε�����LAST.���ڰ����������º��л����ж�
static uint8_t is_init;
static uint8_t send_buff[8]; // �������ݻ�����
static int16_t pitch;
static int16_t yaw;

// ͼ��ӵ�еĴ���ʵ��,��Ϊͼ���ǵ���,��������ֻ��һ��,�Ͳ���װ��
static USART_Instance *video_usart_instance;
static Daemon_Instance *video_daemon_instance;

extern int32_t vision_is_tracking;
extern uint8_t friction_flag;
extern RC_ctrl_t rc_ctrl[2];
extern INS_t INS;

static void VideoDataContorl()
{
    if (video_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl������
        video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (video_ctrl[TEMP].key[KEY_PRESS].shift) // shift������
        video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = video_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = video_ctrl[TEMP].key[KEY_PRESS].keys,                   // ��ǰ�����Ƿ���
        key_last = video_ctrl[LAST].key[KEY_PRESS].keys,                       // ��һ�ΰ����Ƿ���
        key_with_ctrl = video_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // ��ǰctrl��ϼ��Ƿ���
        key_with_shift = video_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  ��ǰshift��ϼ��Ƿ���
        key_last_with_ctrl = video_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // ��һ��ctrl��ϼ��Ƿ���
        key_last_with_shift = video_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // ��һ��shift��ϼ��Ƿ���

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5λΪctrl��shift,ֱ������
            continue;
        // �����ǰ��������,��һ�ΰ���û�а���,��ctrl��shift��ϼ�û�а���,�򰴼����¼�����1(��⵽������)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // ��ǰctrl��ϼ�����,��һ��ctrl��ϼ�û�а���,��ctrl��ϼ����¼�����1(��⵽������)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // ��ǰshift��ϼ�����,��һ��shift��ϼ�û�а���,��shift��ϼ����¼�����1(��⵽������)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            video_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
    video_ctrl[LAST] = video_ctrl[TEMP];
}

/**
 * @brief ͼ�����ݽ�������
 *
 * @param buff ͼ������
 */ 
static void VideoRead(uint8_t *buff)
{
    uint16_t judge_length; // ͳ��һ֡���ݳ���
    if (buff == NULL)      // �����ݰ��������κδ���
        return;
    // д��֡ͷ����(5-byte),�����ж��Ƿ�ʼ�洢��������
    memcpy(&video_ctrl[TEMP].FrameHeader, buff, LEN_HEADER);
    // �ж�֡ͷ����(0)�Ƿ�Ϊ0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // ֡ͷCRC8У��
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            // ͳ��һ֡���ݳ���(byte),����CR16У��
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // ֡βCRC16У��
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                // 2��8λƴ��16λint
                video_ctrl[TEMP].CmdID = (buff[6] << 8 | buff[5]);
                // ��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
                // ��8���ֽڿ�ʼ�������� data=7
                switch (video_ctrl[TEMP].CmdID)
                {
                case ID_custom_robot_data: // �Զ�������
                    memcpy(&video_ctrl[TEMP].custom_data, (buff + DATA_Offset), LEN_custom_robot_data);
                    break;
                case ID_remote_control_data: // ͼ����·��������
                    memcpy(&video_ctrl[TEMP].key_data, (buff + DATA_Offset), LEN_remote_control_data);

                    if (!rc_ctrl[TEMP].rc.switch_left)
                    {
                        // ���͸���C��
                        memcpy(send_buff, buff + DATA_Offset, 8);
                        can_remote(send_buff, 0x36,8);

                        memcpy(send_buff, buff + DATA_Offset + 8, 4);
                        pitch = INS.Roll * 50;
                        yaw = INS.yaw_update * 50;
                        send_buff[4] = (pitch >> 8) & 0xff;
                        send_buff[5] = pitch & 0xff;
                        send_buff[6] = (yaw >> 8) & 0xff;
                        send_buff[7] = yaw & 0xff;
                        can_remote(send_buff, 0x37,8);

                        send_buff[0] = (uint8_t)vision_is_tracking;
                        send_buff[1] = friction_flag;
                        can_remote(send_buff, 0x38,8);
                    }

                    *(uint16_t *)&video_ctrl[TEMP].key[KEY_PRESS] = video_ctrl[TEMP].key_data.keyboard_value;
                    VideoDataContorl();
                    break;
                default:
                    break;
                }
            }
        }
    }
}

/**
 * @brief  ͼ�����ݽ��ջص�����
 *
 */
static void VideoTransmitterCallback()
{
    DaemonReload(video_daemon_instance);
    VideoRead(video_usart_instance->recv_buff);
}

static void VideoTransmitterLostCallback()
{
    USARTServiceInit(video_usart_instance);
}

/**
 * @brief
 *
 * @param vedeo_usart_handle
 * @return Video_ctrl_t*
 */
Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle)
{
    if (is_init)
        return video_ctrl;
    USART_Init_Config_s conf;
    conf.module_callback = VideoTransmitterCallback;
    conf.usart_handle = video_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE;
    video_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VideoTransmitterLostCallback,
        .owner_id = video_usart_instance,
        .reload_count = 30, // 0.3s
    };
    video_daemon_instance = DaemonRegister(&daemon_conf);

    is_init = 1;
    return video_ctrl;
}