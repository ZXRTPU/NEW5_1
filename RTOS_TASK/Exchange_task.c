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

extern INS_t INS;

static Vision_Recv_s *vision_recv_data;
static RC_ctrl_t *rc_data;       // ң��������,��ʼ��ʱ���ص�ָ��
static Video_ctrl_t *video_data; // ͼ������,��ʼ��ʱ���ص�ָ��

ins_data_t ins_data;

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
	
	rc_data = RemoteControlInit(&huart3); // �޸�Ϊ��Ӧ����,ע����������а�dbusЭ�鴮����ѡ������˷��������Ǹ�

  video_data = VideoTransmitterControlInit(&huart1); // �޸�Ϊ��Ӧ����


	while (1)
	{
      VisionSetAltitude(INS.Yaw, INS.Roll, INS.Pitch); // �˴�C�����ڷ���λ�õĹ�ϵ�� Roll �� Pitch �Ե�
      VisionSend();

      osDelay(1);
	}
}