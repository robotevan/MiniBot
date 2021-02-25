#include <MotorDriver.h>


void init_mcpwm(int pin_in1, int pin_in2, int pin_in3, int pin_in4){
    // init motor A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin_in1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pin_in2);
    // init motor B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin_in3);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pin_in4);
}

void set_speed(int speed){
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;
    if(speed > 0){
        pwm_config.cmpr_a = speed;
    }else{
        pwm_config.cmpr_b = speed;
    }
    
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}