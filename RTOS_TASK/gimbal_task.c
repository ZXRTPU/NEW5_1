#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "Exchange_task.h"
#include "drv_can.h"
#include "miniPC_process.h"
#include "VideoTransmitter.h"
#include "user_lib.h"
#define MAX_SPEED 200
#define MAX_ANGLE 3900
#define MIN_ANGLE 2200
#define MAX_ANGLE_VISION 29.2374
#define MIN_ANGLE_VISION 45.3926
#define RECV_PITCH_ANGLE 1.84
#define MAP_DATA  22.8

//1405~0 8192~7550  90 6020���
//17~-32  49 ʵ��ת���Ƕ�ֵ 
     
extern INS_t INS;
extern float vision_yaw;
extern float vision_pitch;
extern int32_t vision_is_tracking;
extern Video_ctrl_t video_ctrl[2]; // ͼ����Ϣ�ṹ��

float recv_vision_pitch;

gimbal_t gimbal_Yaw, gimbal_Pitch; // ��̨�����Ϣ�ṹ��
   
static uint8_t Update_yaw_flag = 1;
static float imu_err_yaw = 0; // ��¼yawƮ�Ƶ���ֵ���ڽ���У��

extern RC_ctrl_t rc_ctrl[2]; // ң������Ϣ�ṹ��   

// ��̨����ĳ�ʼ��
static void Gimbal_loop_Init();

// ģʽѡ��
static void mode_select();

// ��̨���������
static void gimbal_current_give();

//�Ӿ�������̨ģʽ
static void yaw_vision_mode();
static void pitch_vision_mode();
static void gimbal_vision_mode();

//����̨ģʽ
static void yaw_lock_mode();
static void pitch_rc_mode();
static void gimbal_lock_mode();

//��ȡYAW��INS����
static void Yaw_read_INS();

static void detel_calc(fp32 *angle);
static void detel_calc2(fp32 *angle);
static void detel_calc3(fp32 *angle);

void Gimbal_task(void const *pvParameters)
{
    Gimbal_loop_Init();
    for (;;)
    {
        mode_select();
        gimbal_current_give();
        osDelay(1);
    }
}

// ��̨����ĳ�ʼ��
static void Gimbal_loop_Init()
{
    // ��ʼ��pid����
    gimbal_Yaw.pid_parameter[0] = 60, gimbal_Yaw.pid_parameter[1] = 0.5, gimbal_Yaw.pid_parameter[2] = 5;
    gimbal_Yaw.pid_angle_parameter[0] = 6, gimbal_Yaw.pid_angle_parameter[1] = 0, gimbal_Yaw.pid_angle_parameter[2] = 10;
    gimbal_Yaw.pid_vision_parameter[0] = 6, gimbal_Yaw.pid_vision_parameter[1] = 0, gimbal_Yaw.pid_vision_parameter[2] = 10;
	  gimbal_Yaw.angle_target = 0;

    gimbal_Pitch.pid_parameter[0] = 30, gimbal_Pitch.pid_parameter[1] = 0.1, gimbal_Pitch.pid_parameter[2] = 0;
    gimbal_Pitch.pid_angle_parameter[0] = 5, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 0.1;
    gimbal_Pitch.pid_vision_parameter[0] = 5, gimbal_Pitch.pid_vision_parameter[1] = 0, gimbal_Pitch.pid_vision_parameter[2] = 0.1;
	  gimbal_Pitch.angle_target = 3234;
  
    // ��ʼ��pid�ṹ��
    pid_init(&gimbal_Yaw.pid, gimbal_Yaw.pid_parameter, 30000, 30000);
    pid_init(&gimbal_Yaw.pid_angle, gimbal_Yaw.pid_angle_parameter, 15000, 15000);
	  pid_init(&gimbal_Yaw.pid_vision, gimbal_Yaw.pid_vision_parameter, 15000, 15000);

    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 30000, 30000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 10000, 1000);
	  pid_init(&gimbal_Pitch.pid_vision, gimbal_Pitch.pid_vision_parameter, 10000, 1000);
}

// ģʽѡ��
static void mode_select()
{
		// ң��+������·
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			// �Ӿ����� - �Ҹ˾��� �� ����Ҽ�����
      //if (rc_ctrl[TEMP].rc.switch_right == 3 || rc_ctrl[TEMP].mouse.press_r == 1) // �Ҳ����� || ��ס�Ҽ�
			if (rc_ctrl[TEMP].mouse.press_r == 3 || rc_ctrl[TEMP].rc.switch_right==3) 
			{
				gimbal_vision_mode();
			}

			// ��yawģʽ - �Ҳ����л���
			else 
			{
				gimbal_lock_mode();
			}
		}
		
		// ͼ����·
		else
		{
			// �Ӿ�����-��ס�Ҽ�����
			if (video_ctrl[TEMP].key_data.right_button_down == 1) 
			{
				gimbal_vision_mode();
			}

			// ��yawģʽ
			else 
			{
				gimbal_lock_mode();
			}
		}
}

// ���͸�����
static void gimbal_current_give()
{   
	  //yaw������������
    gimbal_Yaw.motor_info.set_current = pid_calc(&gimbal_Yaw.pid, 57.3F * INS.Gyro[2], gimbal_Yaw.speed_target); // 57.3F * INS.Gyro[2]
	  gimbal_Yaw.motor_info.set_current += 0.8*gimbal_Yaw.motor_info.torque_current;
    set_motor_current_gimbal(1, gimbal_Yaw.motor_info.set_current, 0, 0, 0);
	  
	  osDelay(1);
	 
	  //pitch������������
	  //gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r || video_ctrl[TEMP].key_data.right_button_down)
	  {
        if (vision_is_tracking)
            gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid,gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
        else
            gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    }
    else
    {
        gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    }
	  set_motor_current_gimbal(0,  gimbal_Pitch.motor_info.set_current, 0, 0, 0);
}

//�Ӿ�������̨ģʽ
static void gimbal_vision_mode()
{
	yaw_vision_mode();
  pitch_vision_mode();
}  

//����̨ģʽ
static void gimbal_lock_mode()
{
	 yaw_lock_mode();
   pitch_rc_mode();
}

static void yaw_vision_mode()
{
	// ����Yaw��imu����
	Yaw_read_INS();

	// ң������·
	if (rc_ctrl[TEMP].rc.switch_right)
	{
		// ���׷�ٵ�Ŀ��
		if (vision_is_tracking)
		{
			// �Ӿ�ģʽ�м����ֶ�΢��
			float normalized_input = (rc_ctrl[TEMP].rc.rocker_r_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f) * 10.0f; // ���΢���Ƕ�����Ϊ10��
			gimbal_Yaw.angle_target = vision_yaw - normalized_input;
		}
		
		else
		{
			// ʹ�÷�����ӳ�亯������������
			float normalized_input = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f * 100.0f;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	// ͼ����·
	else
	{
		// ���׷�ٵ�Ŀ��
		if (vision_is_tracking)
		{
			// �Ӿ�ģʽ�м����ֶ�΢��
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 10.0f; // ���΢���Ƕ�����Ϊ10��
			gimbal_Yaw.angle_target = vision_yaw - normalized_input;
		}
   
		else
		{
			// ʹ�÷�����ӳ�亯������������
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 100.0f;
			gimbal_Yaw.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	detel_calc(&gimbal_Yaw.angle_target);
	gimbal_Yaw.speed_target = gimbal_Yaw_PID_calc(&gimbal_Yaw.pid_vision, INS.yaw_update,gimbal_Yaw.angle_target);

}

static void yaw_lock_mode()
{
		// ����Yaw��imu����
		Yaw_read_INS();

		// ң������·
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			// ʹ�÷�����ӳ�亯������������
			float normalized_input = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f*2 + rc_ctrl[TEMP].mouse.x / 16384.0f * 100;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.97) * sign(normalized_input) * 0.3;
		}
		// ͼ����·
		else
		{
			// ʹ�÷�����ӳ�亯������������
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 50;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}

		detel_calc(&gimbal_Yaw.angle_target);
		gimbal_Yaw.speed_target = gimbal_Yaw_PID_calc(&gimbal_Yaw.pid_angle, INS.yaw_update, gimbal_Yaw.angle_target);
}

static void pitch_vision_mode()
{
     // ң������·
     if (rc_ctrl[TEMP].rc.switch_right)
     {
        // �Ӿ�ʶ�� - �Ҳ�����OR����Ҽ�����
         if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r==1 )
         {
             if (vision_is_tracking)
             {
                 // �Ӿ�ģʽ�µ��ֶ�΢��
                 float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 20.0f;
							 
//							   //�Ӿ�����ӳ��
//							   recv_vision_pitch = vision_pitch*RECV_PITCH_ANGLE;
							   
                 gimbal_Pitch.angle_target = vision_pitch + normalized_input;
							 
							   detel_calc3(&gimbal_Pitch.angle_target);
                 gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_vision,(gimbal_Pitch.motor_info.rotor_angle -3234 ) * 0.0439*MAP_DATA, gimbal_Pitch.angle_target*MAP_DATA);
             }
             else
             {
                 // ʹ�÷�����ӳ�亯������������
                 float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
                 gimbal_Pitch.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
							 
						     detel_calc2(&gimbal_Pitch.angle_target);
                 gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_vision, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);							   
             }
          }
          
          else
          {
             // ʹ�÷�����ӳ�亯������������
             float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
             gimbal_Pitch.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
						
						 detel_calc2(&gimbal_Pitch.angle_target);
             gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_vision, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
          }
      }

      // ͼ����·
      else
      {
         // �Ӿ�ʶ�� - ����Ҽ�����
         if (video_ctrl[TEMP].key_data.right_button_down)
         {
             if (vision_is_tracking)
             {
                float normalized_input = (video_ctrl[TEMP].key_data.mouse_y / 16384.0f) * 20.0f; 
							 
//							//�Ӿ�����ӳ��
//							recv_vision_pitch = vision_pitch;
							 
                gimbal_Pitch.angle_target = -vision_pitch + normalized_input;
							 
							  detel_calc3(&gimbal_Pitch.angle_target);
							 
                gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_vision,(gimbal_Pitch.motor_info.rotor_angle -3234 ) * 0.0439*MAP_DATA, gimbal_Pitch.angle_target*MAP_DATA);
             }
          }

         else
         {
            // ʹ�÷�����ӳ�亯������������
            float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
            gimbal_Pitch.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
						 
						detel_calc2(&gimbal_Pitch.angle_target);
            gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_vision, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
         }
     }

}

static void pitch_rc_mode()
{	
		// ң������·
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			  if (rc_ctrl[TEMP].rc.rocker_r1>= -660 && rc_ctrl[TEMP].rc.rocker_r1 <= 660)
       {
          gimbal_Pitch.angle_target += rc_ctrl[TEMP].rc.rocker_r1/ 660.0;
       }
		}
		
		// ͼ����·
		else
		{
			// ʹ�÷�����ӳ�亯������������
			float normalized_input = video_ctrl[TEMP].key_data.mouse_y / 16384.0f * 50;
			gimbal_Pitch.angle_target += pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 2.0;
		}

     detel_calc2(&gimbal_Pitch.angle_target);
     gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_angle, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
}

static void detel_calc(fp32 *angle)
{
    if (*angle > 360)
    {
        *angle -= 360;
    }

    else if (*angle < 0)
    {
        *angle += 360;
    }
}

static void detel_calc2(fp32 *angle)
{
//    if (*angle > 4096)
//        *angle -= 8192;

//    else if (*angle < -4096)
//         *angle += 8192;

    if (*angle >= MAX_ANGLE)
        *angle = MAX_ANGLE;

    else if (*angle <= MIN_ANGLE)
        *angle = MIN_ANGLE;
}

static void detel_calc3(fp32 *angle)
{
	      if (*angle >= MAX_ANGLE_VISION)
        *angle = MAX_ANGLE_VISION;

    else if (*angle <= -MIN_ANGLE_VISION)
        *angle = -MIN_ANGLE_VISION;
}


/************************************��ȡyaw��INS����**************************************/
static void Yaw_read_INS()
{
	// ��¼��ʼλ��
	if (Update_yaw_flag)
	{
		Update_yaw_flag = 0; // ֻ����һ��
		INS.yaw_init = INS.Yaw - 0.0f;
		gimbal_Yaw.angle_target= INS.yaw_init;
	}

	 // ˳ʱ����ת��������Ʈ -90��/min
	// ���yawƫ�ƣ����У��
	 if (rc_ctrl[TEMP].rc.rocker_l_ > 50 || rc_ctrl[TEMP].mouse.x > 1500)
	    	imu_err_yaw += 0.0015f;
	 if ((rc_ctrl[TEMP].rc.rocker_l_ < -50 || rc_ctrl[TEMP].mouse.x < -1500))
	 	    imu_err_yaw -= 0.0015f;

	// У��
	INS.yaw_update = INS.Yaw - INS.yaw_init + imu_err_yaw;
}


