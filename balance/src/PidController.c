#include "PidController.h"


void init_pid(pid_control_t *pid, double kp, double ki, double kd)
{
    pid->prev_val = 0;
    pid->sum_error = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kp = kd;
    pid->max_error = MAX_DOUBLE / (pid->kp + 1);
    pid->max_sum_error = MAX_DOUBLE / 2;
}

double calculate_pid(pid_control_t *pid, double set_val, double curr_val)
{
    double error = set_val - curr_val; // calculate error between desired and current
    // Calculate P term
    double p_term = pid->kp * error;    
    // Calculate I term 
    double diff_temp = pid->sum_error + error;
    pid->sum_error = diff_temp;
    double i_term = pid->ki * pid->sum_error;
    // Calculate D term
    double d_term = pid->kd * (pid->prev_val - curr_val); 
    // Sum up and return 
    return p_term + i_term + d_term;
}