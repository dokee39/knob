#include "knob_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "CAN_receive.h"
#include "pid.h"
#include "detect_task.h"
#include "bsp_usart.h"

#define PI 3.1416f

#define KNOB_POS_KP 10.0f
#define KNOB_POS_KI 0.0f
#define KNOB_POS_KD 0.1f
#define KNOB_POS_MAX_IOUT 0.0f
#define KNOB_POS_MAX_OUT 5.0f
#define KNOB_POS_DEAD_BAND 0.0f

#define KNOB_SPEED_KP 500.0f
#define KNOB_SPEED_KI 10.0f
#define KNOB_SPEED_KD 0.0f
#define KNOB_SPEED_MAX_IOUT 10000.0f
#define KNOB_SPEED_MAX_OUT 30000.0f
#define KNOB_SPEED_DEAD_BAND 0.0f

#define KNOB_SMOOTH_RATIO 700

#define KNOB_POS_LIMIT (2 * PI / 4.0f)
#define KNOB_SMOOTH_DEAD_BAND 0

#define TIMER_DUTY_MIN 1000
#define TIMER_DUTY_LEN 1000

typedef struct {
    motor_measure_t *motor;
    pid_typedef pid_pos;
    pid_typedef pid_speed;
    int16_t ctrl_val;
    float pos_offset;
    float pos;
    float pos_set;
    float speed;
    float speed_set;
    uint16_t duty;
} knob_t;

knob_t knob;

static float rad_format(float rad);

void knob_task(const void *arg)
{
    TickType_t xLastWakeTime;
    float knob_pid_pos[3] = { KNOB_POS_KP, KNOB_POS_KI, KNOB_POS_KD };
    float knob_pid_speed[3] = { KNOB_SPEED_KP, KNOB_SPEED_KI, KNOB_SPEED_KD };
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    knob.motor = &motor_measure[MOTOR_GM6020_ID];
    PID_init(&knob.pid_pos, PID_POSITION, knob_pid_pos, KNOB_POS_MAX_OUT, KNOB_POS_MAX_IOUT, KNOB_POS_DEAD_BAND);
    PID_init(&knob.pid_speed, PID_POSITION, knob_pid_speed, KNOB_SPEED_MAX_OUT, KNOB_SPEED_MAX_IOUT, KNOB_SPEED_DEAD_BAND);

    while (toe_is_error(GM6020_TOE)) { }
        
    knob.pos_offset = (float)(knob.motor->ecd) / 8192 * 2 * 3.1416f;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        knob.pos = rad_format((float)(knob.motor->ecd) / 8192 * 2 * 3.1416f - knob.pos_offset);
        knob.speed = (float)(knob.motor->speed_rpm) / 60 * 2 * PI;

        if (knob.pos < -KNOB_SMOOTH_DEAD_BAND && knob.pos > -KNOB_POS_LIMIT / 2) {
            knob.pos_set = 0;
            knob.speed_set = PID_calc(&knob.pid_pos, 0, rad_format(knob.pos_set - knob.pos));
            knob.ctrl_val = PID_calc(&knob.pid_speed, knob.speed, knob.speed_set);
            knob.duty = TIMER_DUTY_MIN;
        } else if (knob.pos < -KNOB_POS_LIMIT / 2 && knob.pos > -KNOB_POS_LIMIT + KNOB_SMOOTH_DEAD_BAND) {
            knob.pos_set = -KNOB_POS_LIMIT;
            knob.speed_set = PID_calc(&knob.pid_pos, 0, rad_format(knob.pos_set - knob.pos));
            knob.ctrl_val = PID_calc(&knob.pid_speed, knob.speed, knob.speed_set);
            knob.duty = TIMER_DUTY_MIN + TIMER_DUTY_LEN;
        } else {
            PID_clear(&knob.pid_pos);
            PID_clear(&knob.pid_speed);
            if ((knob.pos < 0 && knob.pos > -KNOB_SMOOTH_DEAD_BAND) || (knob.pos > -KNOB_POS_LIMIT && knob.pos < -KNOB_POS_LIMIT + KNOB_SMOOTH_DEAD_BAND)) {
                knob.ctrl_val = 0;
            } else {
                knob.ctrl_val = KNOB_SMOOTH_RATIO * knob.speed;
            }
            knob.duty = TIMER_DUTY_MIN + TIMER_DUTY_LEN * ((knob.pos > 0 ? knob.pos : (2 * PI + knob.pos)) / (2 * PI - KNOB_POS_LIMIT));
        }

        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, knob.duty);
        
    
        CAN_cmd(knob.ctrl_val, 0, 0, 0);

        // usart1_printf("%05d,%.3f,%.3f,%.3f,%.3f\r\n", knob.ctrl_val, knob.pos, knob.pos_set, knob.speed, knob.speed_set);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(KNOB_TASK_INTERVAL));
    }
}

static float rad_format(float rad)
{
    if (rad > PI) {
        while (rad > PI) {
            rad -= 2 * PI;
        }
    } else if (rad < -PI) {
        while (rad < -PI) {
            rad += 2 * PI;
        }
    }
    return rad;
}
