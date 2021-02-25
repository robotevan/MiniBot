#include "MotorDriver.h"
#include <unistd.h>


void app_main() {

    init_mcpwm(4,5,18,19);
    while (1){
        for (int i = -100; i <= 100; i++){
            set_speed(i);
            usleep(100000);
        }
    }
}

