#ifndef BSP_INIT_h
#define BSP_INIT_h

#include "bsp_dwt.h"

/**
 * @brief bsp���ʼ��ͳһ���,�������ʼ�������bsp���,��������ĳ�ʼ���ڸ��Ե�ģ���н���
 *        ����ʵʱϵͳ����ǰ����,Ŀǰ��RobotoInit()����
 *
 * @note ����ʵ���͵�������CAN�ʹ��ڻ���ע��ʵ����ʱ���Զ���ʼ��,��ע�᲻��ʼ��
 */
//
void BSPInit()
{
    DWT_Init(168);
}

#endif // BSP_INIT_h