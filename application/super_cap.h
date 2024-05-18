#ifndef SUP_CAP_H
#define SUP_CAP_H

#include "drv_can.h"

typedef struct
{
    uint16_t voltage; // 电压
    uint16_t power;   // 功率
    uint8_t state;   // 状态
} SuperCapRx_t;

typedef struct {
    uint16_t buffer; // 缓冲能量
    uint16_t power;  // 底盘功率
    uint8_t state;   // 状态
} SuperCapTx_t;

//超级电容控制模式选择
typedef enum
{
    SUPERCAP_AUTO = 1,
    SUPERCAP_OFF,
} supercap_state;

void Supercap_task(void const *argument);

/**
 * @brief 设置超级电容数据
 *
 * @param buffer 缓冲能量
 * @param power 底盘功率
 * @param state 状态
 */
void SuperCapSet(uint16_t buffer, uint16_t power, uint8_t state);

/**
 * @brief 发送超级电容数据
 *
 *
 */
void SuperCapSend(SuperCapTx_t TxData);
#endif // !SUP_CAP_H