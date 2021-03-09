#include "MotorDriver.h"
#include "MPU.h"
#include <unistd.h>
#include <esp_log.h>

void app_main() {
    init_mcpwm(4,5,18,19);
    mpu_init(21, 22);
    uint16_t x, y, z = 0;
    while (1){
        vTaskDelay(10);
        mpu_read_gyro_raw(&x, &y, &z);
        printf("X:%d  Y:%d  Z:%d\n", x, y, z);
        //ESP_LOGI("Output", "x: %f, y:%f, z:%f \n", accel.x, accel.y, accel.z);
        
        //ESP_LOGI("Output", "Angle: %f\n", read_angle_xaxis());
    }
}

