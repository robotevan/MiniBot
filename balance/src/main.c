#include "MotorDriver.h"
#include "MPU.h"
#include <unistd.h>
#include <esp_log.h>

void app_main() {
    init_mcpwm(4,5,18,19);
    mpu_init(21, 22);
    
    while (1){
        vTaskDelay(100);
        //ESP_LOGI("Output", "x: %f, y:%f, z:%f \n", accel.x, accel.y, accel.z);
        
        //ESP_LOGI("Output", "Angle: %f\n", read_angle_xaxis());
    }
}

