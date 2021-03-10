#include "MotorDriver.h"
#include "MPU.h"
#include <unistd.h>
#include <esp_log.h>

void app_main() {
    init_mcpwm(4,5,18,19);
    mpu_init(21, 22);
    mpu_calibrate();
    while (1){
        float x = 0, y = 0, z = 0;
        vTaskDelay(10);
        mpu_read_accel(&x, &y, &z);
        printf("X:%f  Y:%f Z:%f\n", x, y, z);
    }
}

