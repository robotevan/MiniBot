#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#define MAX_DOUBLE 32767.0
#define MIN_DOUBLE -32768.0


typedef struct pid_controller {
    double kp;
    double ki;
    double kd;
    double prev_val;
    double sum_error;
    double max_error;
    double max_sum_error;
} pid_control_t;

void init_pid(pid_control_t *pid, double kp, double ki, double kd);
double calculate_pid(pid_control_t *pid, double set_val, double curr_val);
#endif