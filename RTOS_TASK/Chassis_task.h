//#ifndef CHASSIS_TASK_H
//#define  CHASSIS_TASK_H

//#include "pid.h"
//#include  "drv_can.h"
//#include "rc_potocal.h"
//#include "main.h"
//#include "encoder_map.h"
//#include "rc_map.h"

//typedef struct
//{
//    uint16_t can_id;		//ID��
//    int16_t  set_current;		//������Ϣ
//    uint16_t rotor_angle;		//���ڵĽǶ�
//    int16_t  rotor_speed;		//���ڵ�ת��
//    int16_t  torque_current;		//ʵ��ת�ص���
//    uint8_t  temp;		//����¶�
//	
//	  int16_t target_speed; 
//	
//	  pid_struct_t motor_pid;
//	
//}motor_info_t;


//typedef struct
//{
//	  float gyro_angle;//�����ǽ�����ĵ�ǰ�Ƕ�
//	  int16_t  gyro_omega;	//�����ǽ�����ĵ�ǰ���ٶ�
//	
//	  float target_angle;//Ŀ��Ƕ�
//	  int16_t target_speed;  //Ŀ���ٶ�
//	
//	  fp32 ZERO_gyro;

//}chassis_direct_t;


//typedef enum {
//    CHAS_LF,
//    CHAS_RF,
//    CHAS_RB,
//    CHAS_LB,
//} chassis_motor_cnt_t;

////=================================���������������======================================
//extern int16_t Drifting_yaw;
//extern uint16_t Down_ins_yaw;

////motor_info_t  motor_info_chassis[4];   //�����Ϣ�ṹ��

////================================��������������========================================
//void  Chassis_task(void const * argument);

//void Chassis_Init();
//void chassis_loop();
//void chassis_mode_choice();
//void chassis_mode1();
//void chassis_mode2();
//void chassis_mode3();

//void chassis_motol_speed_calculate(void);
//void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);

//void chassis_current_give();

//void RC_to_Vector(void);

//void lock_chassis_direct();

//void chassis_current_give_RC_6020();

//fp32 Get_err();

//#endif