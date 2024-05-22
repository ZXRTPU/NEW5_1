#include "struct_typedef.h"
#include "Exchange_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "drv_can.h"
#include <string.h>
#include "usart.h"
#include "miniPC_process.h"
#include "remote_control.h"
#include "VideoTransmitter.h"

#define SEND_PITCH_ANGLE 0.54

extern INS_t INS;
extern gimbal_t gimbal_Pitch;
extern Video_ctrl_t video_ctrl[2];
extern RC_ctrl_t rc_ctrl[2]; // ң������Ϣ�ṹ�� 

static Vision_Recv_s *vision_recv_data;
static RC_ctrl_t *rc_data;       // ң��������,��ʼ��ʱ���ص�ָ��
static Video_ctrl_t *video_data; // ͼ������,��ʼ��ʱ���ص�ָ��
float send_vision_pitch;

ins_data_t ins_data;

uint8_t temp_remote[8];
float yaw = 0;

void Exchange_task(void const * argument)
{
	Vision_Init_Config_s config = {
		.recv_config = {
			.header = VISION_RECV_HEADER,
		},
		.send_config = {
			.header = VISION_SEND_HEADER,
			.detect_color = VISION_DETECT_COLOR_BLUE,
			.reset_tracker = VISION_RESET_TRACKER_NO,
			.is_shoot = VISION_SHOOTING,
			.tail = VISION_SEND_TAIL,
		},
		.usart_config = {
			.recv_buff_size = VISION_RECV_SIZE,
			.usart_handle = &huart1,
		},
	};
	
	vision_recv_data = VisionInit(&config);
	
	//rc_data = RemoteControlInit(&huart3); // �޸�Ϊ��Ӧ����,ע����������а�dbusЭ�鴮����ѡ������˷��������Ǹ�

  video_data = VideoTransmitterControlInit(&huart6); // �޸�Ϊ��Ӧ����


	while (1)
	{
			//����ģʽѡ��
			// ң��+������·
			if (rc_ctrl[TEMP].rc.switch_right)
			{
					switch (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Z] % 2)
					{
					case 0:
						VisionSetEnergy(0);
						break;
					default:
						VisionSetEnergy(1);
						break;
					}
			}
			
			// ͼ����·
			else
			{
					switch (video_ctrl[TEMP].key_count[KEY_PRESS][Key_Z] % 2)
					{
					case 0:
						VisionSetEnergy(0);
						break;
					default:
						VisionSetEnergy(1);
						break;
					}
			}
			
		  //pitchӳ�䣨����ֵ - 3234��*0.0439
		  send_vision_pitch = -(gimbal_Pitch.motor_info.rotor_angle -3234 ) * 0.0439;
      VisionSetAltitude(INS.Yaw, send_vision_pitch, INS.Pitch); // �˴�C�����ڷ���λ�õĹ�ϵ�� Roll �� Pitch �Ե�
  		VisionSend();
		
		  yaw = 100 * INS.Yaw;
	    temp_remote[0] = ((int16_t)yaw >> 8) & 0xff;
	    temp_remote[1] = (int16_t)yaw & 0xff;
      can_remote(temp_remote, 0x35,8);

      osDelay(1);
	}
}