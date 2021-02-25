#include "MotorDriver.h"
#include <unistd.h>


void app_main() {

    init_mcpwm(4,5,18,19);
    while (1){
        printf("hello\n");
    }
}

