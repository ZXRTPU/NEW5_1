#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "Exchange_task.h"
#include "drv_can.h"
#include "bsp_dwt.h"
#include <stdbool.h>
#include <pid.h>
#include "VideoTransmitter.h"
#define TRIGGER_SINGLE_ANGLE 1620 // 36*360/8

// 电机0为拨盘电机，电机1为弹舱盖电机，电机2、3为摩擦轮电机
shooter_t shooter; // 发射机构信息结构体

extern RC_ctrl_t rc_ctrl[2]; // 遥控器信息结构体
extern Video_ctrl_t video_ctrl[2];
extern int32_t vision_is_tracking;
extern int32_t vision_is_shooting;

bool is_angle_control = false;//单发
float current_time = 0;
float last_time = 0;
uint8_t flag_single=1;

uint8_t friction_flag = 0; // 摩擦轮转速标志位，012分别为low, normal, high, 默认为normal

static void Shooter_Inint();         // 发射机构的初始化
static void model_choice();          // 模式选择
static void dial_control();          // 拨盘电机控制
static void friction_control();      // 摩擦轮电机控制
static void bay_control();           // 弹舱电机控制
static void shooter_current_given(); // 给电流

// 单发
static void trigger_single_angle_move();
//堵转时拨盘反转
static void shoot_reverse();

void Shoot_task(void const * argument)
{
    Shooter_Inint();
    for (;;)
    {
        model_choice();
        shooter_current_given();
        osDelay(1);
    }
}

// 发射机构的初始化
static void Shooter_Inint(void)
{
    // 初始化pid参数
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    shooter.pid_angle_value[0] = 10;
    shooter.pid_angle_value[1] = 0.05;
    shooter.pid_angle_value[2] = 500;
    // 初始化pid结构体
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 10000, 10000);
    pid_init(&shooter.pid_angle, shooter.pid_angle_value, 20000, 30000); // trigger_angle

    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 20000, 20000);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 10000, 10000);

    // 初始化速度目标
    shooter.dial_speed_target = 0;
    shooter.target_angle = shooter.motor_info[0].total_angle;

    shooter.friction_speed_target[0] = 0, shooter.friction_speed_target[1] = 0;//两个摩擦轮速度
    shooter.bay_speed_target = 0;
}

// 模式选择
static void model_choice(void)
{
    //弹仓电机
    bay_control();

    if (rc_ctrl[TEMP].rc.switch_left == 3 || rc_ctrl[TEMP].rc.switch_left == 1 || video_ctrl[TEMP].key_count[KEY_PRESS][Key_F]%2 == 1)
    {
        // 发射摩擦轮- F键
        friction_control();
			
			  friction_flag = 1;
        
			  //拨盘电机 - E键为单发 R键为连发
        dial_control();
    }
		
    else
    {
			  friction_flag = 0;
			
        shooter.dial_speed_target = 0;
        shooter.motor_info[0].set_current=0;
        shooter.bay_speed_target = 0;
        // 停止
        shooter.friction_speed_target[0] = 0;
        shooter.friction_speed_target[1] = 0;
    }
}


// 摩擦轮电机控制
static void friction_control(void)
{
    shooter.friction_speed_target[0] = -6900;
    shooter.friction_speed_target[1] = 6900;
}

// 拨盘电机控制
static void dial_control()
{
		//拨盘电机
	  //单发
    if (rc_ctrl[TEMP].rc.switch_right == 3 || video_ctrl[TEMP].key[KEY_PRESS].e)
    {
       if (flag_single)
       {
          trigger_single_angle_move();
					is_angle_control = true;
          //flag_single=0;
       }
    }
		//连发
    else if (rc_ctrl[TEMP].rc.switch_left ==1 || video_ctrl[TEMP].key_count[KEY_PRESS][Key_R]%2 == 1 || video_ctrl[TEMP].key_data.left_button_down == 1)
    {
        shooter.dial_speed_target = 2100;
        is_angle_control = false;
    }
		//单发重置
    //else if (rc_ctrl[TEMP].rc.switch_right == 2 || video_ctrl[TEMP].key_count[KEY_PRESS][Key_Q])
    else
		{
       //flag_single=1;
			 is_angle_control = false;
			 if (!is_angle_control)
          shooter.dial_speed_target= 0;
    }
		
		// 最后判断，当按下右键使用视觉，且视觉控制不能发射时，拨盘电机停转
    if (video_ctrl[TEMP].key_data.right_button_down && vision_is_shooting == 0 && vision_is_tracking == 1)
    {
        shooter.dial_speed_target = 0;
    }
}

// 弹舱电机控制
static void bay_control(void)
{
    if (!friction_flag && video_ctrl[TEMP].key_count[KEY_PRESS][Key_Q]%2 == 1) // 摩擦轮电机停转或者发射
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);               // 500 开
    else
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2100); // 2100 关
}

// 给电流
static void shooter_current_given(void)
{
    // shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机
    if (is_angle_control)
        shooter.motor_info[0].set_current = pid_calc_trigger(&shooter.pid_angle, shooter.target_angle, shooter.motor_info[0].total_angle); // 拨盘电机
    else
        shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机

    shooter.motor_info[1].set_current = pid_calc(&shooter.pid_bay, shooter.motor_info[1].rotor_speed, shooter.bay_speed_target);          // 弹舱电机
    shooter.motor_info[2].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[2].rotor_speed, shooter.friction_speed_target[0]); // 摩擦轮电机
    shooter.motor_info[3].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[3].rotor_speed, shooter.friction_speed_target[1]); // 摩擦轮电机
    set_motor_current_shoot(1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
    // set_curruent(MOTOR_3508_1, hcan1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
}

/*************拨盘旋转固定角度***********/
static void trigger_single_angle_move()
{
    current_time = DWT_GetTimeline_ms();
    // 判断两次发射时间间隔，避免双发
    if (current_time - last_time > 1000)
    {
        last_time = DWT_GetTimeline_ms();
        shooter.target_angle = shooter.motor_info[0].total_angle + TRIGGER_SINGLE_ANGLE;
    }
}
/*****************反转******************/
static void shoot_reverse()
{
    shooter.dial_speed_target = 250;
}
