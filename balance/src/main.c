#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "MPU.h"
#include "MotorDriver.h"
#include  "PidController.h"

//pid_t pid_controller;
//
//void balance_task(void *pvParameter)
//{
//    
//}

void app_main() 
{
    mpu_init(21 ,22);
    //mcpwm_init(4, 5, 18, 19);
   // init_pid(&pid_controller, 1.0, 1.0, 1.0);

    //xTaskCreate(&balance_task, "balance_task", 2048, NULL, 5, NULL);
    mpu_calibrate();
    while(1)
    {
        get_x_angle_accel();
        
    }

}

