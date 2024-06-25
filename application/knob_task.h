#ifndef _KNOB_TASK_H_
#define _KNOB_TASK_H_

#include <stdint.h>
#include "CAN_receive.h"

typedef enum {
    INIT,
    FORWARD,
    KNOB,
    BACKWARD,
} knob_mode_e;

typedef struct {
    knob_mode_e mode;
    motor_measure_t *motor;
} knob_t;

#endif
