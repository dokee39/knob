/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include <stdint.h>

#define GM6020_CAN  hcan1

/* CAN send and receive ID */
typedef enum {
    // CAN2
    CAN_ALL_ID = 0x1FF,
    CAN_GM6020_ID  = 0x205,
} can_msg_id_e;

typedef enum {
    MOTOR_GM6020_ID,
    MOTOR_NUM, // 电机总个数，也是电机报文数组的长度
} motor_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern motor_measure_t motor_measure[];

extern void CAN_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
