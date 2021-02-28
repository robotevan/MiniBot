#ifndef MPU_H
#define MPU_H
#include <stdint.h>
#include <esp_system.h>
#include <esp_log.h>
#include <math.h>
#include "i2c.h"

#define MPU_SLAVE_ADDR          0x68
#define MPU_ACCEL_XOUT_H        0x3B
#define MPU_ACCEL_XOUT_L        0x3C
#define MPU_ACCEL_YOUT_H        0x3D
#define MPU_ACCEL_YOUT_L        0x3E
#define MPU_ACCEL_ZOUT_H        0x3F
#define MPU_ACCEL_ZOUT_L        0x40
#define MPU_TEMP_OUT_H          0x41
#define MPU_TEMP_OUT_L          0x42
#define MPU_GYRO_XOUT_H         0x43
#define MPU_GYRO_XOUT_L         0x44
#define MPU_GYRO_YOUT_H         0x45
#define MPU_GYRO_YOUT_L         0x46
#define MPU_GYRO_ZOUT_H         0x47
#define MPU_GYRO_ZOUT_L         0x48
#define MPU_PWR_MGMT_1          0x6B
#define MPU_ACCEL_CONFIG        0x1C
#define MPU_GYRO_CONFIG         0x1B
#define MPU_SMPLRT_DIV          0x19
#define MPU_CONFIG              0x1A

#define ACCEL_RANGE_2G 0x00
#define ACCEL_RANGE_4G 0x08
#define ACCEL_RANGE_8G 0x10
#define ACCEL_RANGE_16G 0x18

#define GYRO_RANGE_250 0x00
#define GYRO_RANGE_500 0x08
#define GYRO_RANGE_1000 0x10
#define GYRO_RANGE_2000 0x18

#define ACCEL_SENS_2G 16384.0
#define ACCEL_SENS_4G 8192.0
#define ACCEL_SENS_8G 4096.0
#define ACCEL_SENS_16G 2048.0

#define GYRO_SENS_250 131.0
#define GYRO_SENS_500 65.5
#define GYRO_SENS_1000 32.8
#define GYRO_SENS_2000 16.4

#define CALIBRATION_SAMPLES 10000
 
void mpu_init();
void mpu_set_gyro_range();
void mpu_set_accel_range();

#endif