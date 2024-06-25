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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// motor data read
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd      = (ptr)->ecd;                             \
        (ptr)->ecd           = (uint16_t)((data)[0] << 8 | (data)[1]); \
        (ptr)->speed_rpm     = (uint16_t)((data)[2] << 8 | (data)[3]); \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate     = (data)[6];                              \
    }

// 电机报文数据，数组元素对应的电机见枚举 motor_id_e
motor_measure_t motor_measure[MOTOR_NUM];

// 电机发送数据的 buffer
static CAN_TxHeaderTypeDef gm6020_tx_message;
static uint8_t gm6020_can_send_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId) {
        case CAN_GM6020_ID: {
            get_motor_measure(&motor_measure[MOTOR_GM6020_ID], rx_data);
            detect_hook(GM6020_TOE);
        }

        default:
            break;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    switch (rx_header.StdId) {
        
        default:
            break;
    }
}

void CAN_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    gm6020_tx_message.StdId = CAN_ALL_ID;
    gm6020_tx_message.IDE   = CAN_ID_STD;
    gm6020_tx_message.RTR   = CAN_RTR_DATA;
    gm6020_tx_message.DLC   = 0x08;
    gm6020_can_send_data[0] = (motor1 >> 8);
    gm6020_can_send_data[1] = motor1;
    gm6020_can_send_data[2] = (motor2 >> 8);
    gm6020_can_send_data[3] = motor2;
    gm6020_can_send_data[4] = (motor3 >> 8);
    gm6020_can_send_data[5] = motor3;
    gm6020_can_send_data[6] = (motor4 >> 8);
    gm6020_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&GM6020_CAN, &gm6020_tx_message, gm6020_can_send_data, &send_mail_box);
}

