
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <driver/mcpwm.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


void motordriver_init(int pin_in1, int pin_in2, int pin_in3, int pin_in4);
void motordriver_set_speed(int speed);



#endif