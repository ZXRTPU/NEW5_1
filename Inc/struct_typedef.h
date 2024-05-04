#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct
{
    uint16_t can_id;        // ID��
    int16_t set_current;    // ������Ϣ
    uint16_t rotor_angle;   // ���ڵĽǶ�
    int16_t rotor_speed;    // ���ڵ�ת��
    int16_t torque_current; // ʵ��ת�ص���
    uint8_t temp;           // ����¶�

    float total_angle;   // �ܽǶ�,ע�ⷽ��
    int32_t total_round; // ��Ȧ��,ע�ⷽ��
    uint16_t last_ecd;   // ��һ�ζ�ȡ�ı�����ֵ
    uint16_t ecd;        // 0-8191,�̶��ܹ���8192��
    float angle_single_round; // ��Ȧ�Ƕ�
} motor_info_t;

typedef struct
{

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;
    fp32 ref;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];

} pid_struct_t;

typedef struct
{
    motor_info_t motor_info;     // �����Ϣ�ṹ��
	
    fp32 pid_parameter[3];       // ����ٶȵ�pid����
    fp32 pid_angle_parameter[3]; // ����Ƕȵ�pid����
	  fp32 pid_vision_parameter[3]; //����Ӿ���pid����
	
    pid_struct_t pid;            // ��̨�����pid�ṹ��
    pid_struct_t pid_angle;      // ��̨�����pid�ṹ��
	  pid_struct_t pid_vision;
	
    fp32 speed_target;           // ��̨�����Ŀ���ٶ�
    fp32 angle_target;           // ��̨�����Ŀ��Ƕ�
    fp32 err_angle;              // ��̨�����Ŀ��Ƕ�
} gimbal_t;

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // �����Ϣ�ṹ��
    fp32 pid_parameter[3];      // ���̵����pid����
    pid_struct_t pid[4];        // ���̵����pid�ṹ��
    int16_t speed_target[4];    // ���̵����Ŀ���ٶ�
    int16_t Vx, Vy, Wz;         // ���̵����Ŀ���ٶ�
} chassis_t;

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // �����Ϣ�ṹ��

    fp32 pid_dial_para[3];     // ���̵����pid����
    fp32 pid_angle_value[3];  // ����angle��pid����

    fp32 pid_friction_para[3]; // Ħ���ֵ����pid����
    fp32 pid_bay_para[3];      // ���յ����pid����

    pid_struct_t pid_angle;    // ����angle��pid�ṹ��

    pid_struct_t pid_dial;     // ���̵����pid�ṹ��
    pid_struct_t pid_friction; // Ħ���ֵ����pid�ṹ��
    pid_struct_t pid_bay;      // ���յ����pid�ṹ��

    int16_t dial_speed_target;     // ���̵����Ŀ���ٶ�

    int16_t friction_speed_target[2]; // Ħ���ֵ����Ŀ���ٶ�
    int16_t bay_speed_target;      // ���յ����Ŀ���ٶ�
    float target_angle;            // ���̵�Ŀ��Ƕ�

    // uint16_t last_ecd;             // ��һ�ζ�ȡ�ı�����ֵ
    // uint16_t ecd;                  // 0-8191,�̶��ܹ���8192��
    // float angle_single_round;      // ��Ȧ�Ƕ�

    // float total_angle;   // �ܽǶ�,ע�ⷽ��
    // int32_t total_round; // ��Ȧ��,ע�ⷽ��
} shooter_t;

#endif
