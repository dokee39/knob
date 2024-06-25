/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef PID_H
#define PID_H

#include <stdint.h>

// PID 模式
typedef enum _pid_mode {
    PID_POSITION = 0, // 位置式
    PID_DELTA         // 增量式
} pid_mode_e;

// PID 相关参数
typedef struct _pid_typedef
{
    pid_mode_e mode;
    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float error[3];  // 误差项 0最新 1上一次 2上上次
    float last_diff; // 记录上次的微分值，用于指定微分的增量式 PID 计算
    float dead_band; // 误差死区 (绝对值)

    float out;
    float Pout;
    float Iout;
    float Dout;

    float (*compensate)(struct _pid_typedef *); // 前馈控制的函数指针
    float Cout;

} pid_typedef;

extern void PID_init(pid_typedef *pid, pid_mode_e mode, const float PID[3], float max_out, float max_iout, float dead_band);
extern void PID_add_compensate(pid_typedef *pid, float (*compensate)(pid_typedef *));
extern float PID_calc(pid_typedef *pid, float ref, float set);
extern float PID_calc_specifyD(pid_typedef *pid, float ref, float set, float diff);
extern void PID_clear(pid_typedef *pid);

#endif
