#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "Exchange_task.h"
#include "drv_can.h"
#include "miniPC_process.h"
#include "VideoTransmitter.h"
#include "user_lib.h"
#define MAX_SPEED 200
#define MAX_ANGLE 1600//3400
#define MIN_ANGLE -720//1400//平的700
#define RECV_PITCH_ANGLE 1.84

//1405~0 8192~7550  90 6020电机
//17~-32  49 实际转过角度值


extern INS_t INS;
extern float vision_yaw;
extern float vision_pitch;
extern int32_t vision_is_tracking;
extern Video_ctrl_t video_ctrl[2]; // 图传信息结构体

float recv_vision_pitch;

gimbal_t gimbal_Yaw, gimbal_Pitch; // 云台电机信息结构体

static uint8_t Update_yaw_flag = 1;
static float imu_err_yaw = 0; // 记录yaw飘移的数值便于进行校正

extern RC_ctrl_t rc_ctrl[2]; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 模式选择
static void mode_select();

// 云台电机的任务
static void gimbal_current_give();

//视觉控制云台模式
static void yaw_vision_mode();
static void pitch_vision_mode();
static void gimbal_vision_mode();

//锁云台模式
static void yaw_lock_mode();
static void pitch_rc_mode();
static void gimbal_lock_mode();

//读取YAW轴INS数据
static void Yaw_read_INS();

static void detel_calc(fp32 *angle);
static void detel_calc2(fp32 *angle);

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

// 云台电机的初始化
static void Gimbal_loop_Init()
{
    // 初始化pid参数
    gimbal_Yaw.pid_parameter[0] = 200, gimbal_Yaw.pid_parameter[1] = 5, gimbal_Yaw.pid_parameter[2] = 0;
    gimbal_Yaw.pid_angle_parameter[0] = 5, gimbal_Yaw.pid_angle_parameter[1] = 0, gimbal_Yaw.pid_angle_parameter[2] = 20;
    gimbal_Yaw.pid_vision_parameter[0] = 6, gimbal_Yaw.pid_vision_parameter[1] = 0, gimbal_Yaw.pid_vision_parameter[2] = 10;
	  gimbal_Yaw.angle_target = 0;

    gimbal_Pitch.pid_parameter[0] = 30, gimbal_Pitch.pid_parameter[1] = 0.1, gimbal_Pitch.pid_parameter[2] = 0;
    gimbal_Pitch.pid_angle_parameter[0] = 5, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 0.1;
    gimbal_Pitch.pid_vision_parameter[0] = 5, gimbal_Pitch.pid_vision_parameter[1] = 0, gimbal_Pitch.pid_vision_parameter[2] = 0.1;
	  gimbal_Pitch.angle_target = 700;

    // 初始化pid结构体
    pid_init(&gimbal_Yaw.pid, gimbal_Yaw.pid_parameter, 30000, 30000);
    pid_init(&gimbal_Yaw.pid_angle, gimbal_Yaw.pid_angle_parameter, 15000, 15000);
	  pid_init(&gimbal_Yaw.pid_vision, gimbal_Yaw.pid_vision_parameter, 15000, 15000);

    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 30000, 30000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 10000, 1000);
	  pid_init(&gimbal_Pitch.pid_vision, gimbal_Pitch.pid_vision_parameter, 1000, 1000);
}

// 模式选择
static void mode_select()
{
		// 遥控+键鼠链路
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			// 视觉控制 - 右杆居中 或 鼠标右键按下
      //if (rc_ctrl[TEMP].rc.switch_right == 3 || rc_ctrl[TEMP].mouse.press_r == 1) // 右拨杆中 || 按住右键
			if (rc_ctrl[TEMP].mouse.press_r == 3 || rc_ctrl[TEMP].rc.switch_right==3) 
			{
				gimbal_vision_mode();
			}

			// 锁yaw模式 - 右拨杆中或下
			else 
			{
				gimbal_lock_mode();
			}
		}
		
		// 图传链路
		else
		{
			// 视觉控制-按住右键开启
			if (video_ctrl[TEMP].key_data.right_button_down == 1) 
			{
				gimbal_vision_mode();
			}

			// 锁yaw模式
			else 
			{
				gimbal_lock_mode();
			}
		}
}

// 发送给电流
static void gimbal_current_give()
{   
	  //yaw轴电机电流发送
    gimbal_Yaw.motor_info.set_current = pid_calc(&gimbal_Yaw.pid, 57.3F * INS.Gyro[2], gimbal_Yaw.speed_target); // 57.3F * INS.Gyro[2]
	gimbal_Yaw.motor_info.set_current += 0.8*gimbal_Yaw.motor_info.torque_current;
    set_motor_current_gimbal(1, gimbal_Yaw.motor_info.set_current, 0, 0, 0);
	  
	  osDelay(1);
	 
	  //pitch轴电机电流发送
	  //gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r || video_ctrl[TEMP].key_data.right_button_down)
	  {
        if (vision_is_tracking)
            gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid_vision, 57.3F * INS.Gyro[1], gimbal_Pitch.speed_target);
        else
            gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    }
    else
    {
        gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    }
	  //set_motor_current_gimbal(0, gimbal_Pitch.motor_info.set_current, 0, 0, 0);
}

//视觉控制云台模式
static void gimbal_vision_mode()
{
	yaw_vision_mode();
  pitch_vision_mode();
}

//锁云台模式
static void gimbal_lock_mode()
{
	 yaw_lock_mode();
   pitch_rc_mode();
}

static void yaw_vision_mode()
{
	// 接收Yaw轴imu数据
	Yaw_read_INS();

	// 遥控器链路
	if (rc_ctrl[TEMP].rc.switch_right)
	{
		// 如果追踪到目标
		if (vision_is_tracking)
		{
			// 视觉模式中加入手动微调
			float normalized_input = (rc_ctrl[TEMP].rc.rocker_r_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f) * 10.0f; // 最大微调角度限制为10°
			gimbal_Yaw.angle_target = vision_yaw - normalized_input;
		}
		
		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f + rc_ctrl[TEMP].mouse.x / 16384.0f * 100.0f;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	// 图传链路
	else
	{
		// 如果追踪到目标
		if (vision_is_tracking)
		{
			// 视觉模式中加入手动微调
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 10.0f; // 最大微调角度限制为10°
			gimbal_Yaw.angle_target = vision_yaw - normalized_input;
		}
   
		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 100.0f;
			gimbal_Yaw.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}
	}

	detel_calc(&gimbal_Yaw.angle_target);
	gimbal_Yaw.speed_target = gimbal_Yaw_PID_calc(&gimbal_Yaw.pid_vision, INS.yaw_update,gimbal_Yaw.angle_target);

}

static void yaw_lock_mode()
{
		// 接收Yaw轴imu数据
		Yaw_read_INS();

		// 遥控器链路
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f*2 + rc_ctrl[TEMP].mouse.x / 16384.0f * 100;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.97) * sign(normalized_input) * 0.3;
		}
		// 图传链路
		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 50;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
		}

		detel_calc(&gimbal_Yaw.angle_target);
		gimbal_Yaw.speed_target = gimbal_Yaw_PID_calc(&gimbal_Yaw.pid_angle, INS.yaw_update, gimbal_Yaw.angle_target);
}

static void pitch_vision_mode()
{
     // 遥控器链路
     if (rc_ctrl[TEMP].rc.switch_right)
     {
        // 视觉识别 - 右拨杆上OR鼠标右键按下
         if (switch_is_up(rc_ctrl[TEMP].rc.switch_right) || rc_ctrl[TEMP].mouse.press_r==1 )
         {
             if (vision_is_tracking)
             {
                 // 视觉模式下的手动微调
                 float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
							 
							   //视觉数据映射
							   recv_vision_pitch = vision_pitch*RECV_PITCH_ANGLE;
							   
                 gimbal_Pitch.angle_target = recv_vision_pitch + normalized_input;
             }
             else
             {
                 // 使用非线性映射函数调整灵敏度
                 float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
                 gimbal_Pitch.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
             }
          }
          
          else
          {
             // 使用非线性映射函数调整灵敏度
             float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
             gimbal_Pitch.angle_target  -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
          }
      }

      // 图传链路
      else
      {
         // 视觉识别 - 鼠标右键按下
         if (video_ctrl[TEMP].key_data.right_button_down == 1)
         {
             if (vision_is_tracking)
             {
                // 视觉模式下的手动微调
                float normalized_input = (video_ctrl[TEMP].key_data.mouse_y / 16384.0f) * 100.0f;
							 
							  //视觉数据映射
							  recv_vision_pitch = vision_pitch*RECV_PITCH_ANGLE;
							 
                gimbal_Pitch.angle_target = recv_vision_pitch + normalized_input;
             }
          }

          else
          {
             // 使用非线性映射函数调整灵敏度
             float normalized_input = (rc_ctrl[TEMP].rc.rocker_r1 / 660.0f - rc_ctrl[TEMP].mouse.y / 16384.0f) * 100.0f;
             gimbal_Pitch.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
          }
      }

      detel_calc2(&gimbal_Pitch.angle_target);
      gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_angle, gimbal_Pitch.motor_info.rotor_angle, gimbal_Pitch.angle_target);
}

static void pitch_rc_mode()
{	
		// 遥控器链路
		if (rc_ctrl[TEMP].rc.switch_right)
		{
			  if (rc_ctrl[TEMP].rc.rocker_r1>= -660 && rc_ctrl[TEMP].rc.rocker_r1 <= 660)
       {
          gimbal_Pitch.angle_target += rc_ctrl[TEMP].rc.rocker_r1/ 660.0;
       }
		}
		
		// 图传链路
		else
		{
			// 使用非线性映射函数调整灵敏度
			float normalized_input = video_ctrl[TEMP].key_data.mouse_x / 16384.0f * 50;
			gimbal_Yaw.angle_target -= pow(fabs(normalized_input), 0.98) * sign(normalized_input) * 0.3;
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
    if (*angle > 4096)
        *angle -= 8192;

    else if (*angle < -4096)
         *angle += 8192;

//    if (*angle >= MAX_ANGLE)
//        *angle = MAX_ANGLE;

//    else if (*angle <= MIN_ANGLE)
//        *angle = MIN_ANGLE;
}


/************************************读取yaw轴INS数据**************************************/
static void Yaw_read_INS()
{
	// 记录初始位置
	if (Update_yaw_flag)
	{
		Update_yaw_flag = 0; // 只进入一次
		INS.yaw_init = INS.Yaw - 0.0f;
		gimbal_Yaw.angle_target= INS.yaw_init;
	}

	 // 顺时针旋转，陀螺仪飘 -90°/min
	// 解决yaw偏移，完成校正
	 if (rc_ctrl[TEMP].rc.rocker_l_ > 50 || rc_ctrl[TEMP].mouse.x > 1500)
	    	imu_err_yaw += 0.0015f;
	 if ((rc_ctrl[TEMP].rc.rocker_l_ < -50 || rc_ctrl[TEMP].mouse.x < -1500))
	 	    imu_err_yaw -= 0.0015f;

	// 校正
	INS.yaw_update = INS.Yaw - INS.yaw_init + imu_err_yaw;
}


