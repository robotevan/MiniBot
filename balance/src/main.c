#include "MotorDriver.h"
#include "MPU.h"
#include <unistd.h>
#include <esp_log.h>

void app_main() {
    init_mpu_6050();
    init_mcpwm(4,5,18,19);
    while (1){
        vTaskDelay(10);
        //mpu_accel_t accel = read_accel();
        //ESP_LOGI("Output", "x: %f, y:%f, z:%f \n", accel.x, accel.y, accel.z);
        
        ESP_LOGI("Output", "Angle: %f\n", read_angle_xaxis());
    }
}

