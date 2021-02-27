#include "MotorDriver.h"
#include "MPU.h"
#include <unistd.h>


void app_main() {
    float temp;
    init_mpu_6050();
    init_mcpwm(4,5,18,19);
    calibrate_offsets();
    while (1){
        read_temperature(&temp);
    }
}

