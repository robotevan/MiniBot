#ifndef MPU_H
#define MPU_H
#include <stdint.h>
#include <esp_system.h>
#include <esp_log.h>
#include <math.h>
#include "i2c.h"
#include <assert.h>

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


#define MPU_SLEEP_BIT 6

typedef enum {
    INTERNAL_8MHZ,
    PLL_X_GYRO,
    PLL_Y_GYRO,
    PLL_Z_GYRO,
    PLL_EXTERNAL_32MHZ,
    PLL_EXTERNAL_19MHZ,
    RESERVED,
    CLOCK_STOP
} clock_source_t;


typedef enum {
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
} accel_range_t;

// accel sensitivity values
#define ACCEL_SENS_2G 16384.0
#define ACCEL_SENS_4G 8192.0
#define ACCEL_SENS_8G 4096.0
#define ACCEL_SENS_16G 2048.0


typedef enum {
    GYRO_RANGE_250 = 0x00,
    GYRO_RANGE_500 = 0x08,
    GYRO_RANGE_1000 = 0x10,
    GYRO_RANGE_2000 = 0x18
} gyro_range_t;

// gyro sensitivity values
#define GYRO_SENS_250 131.0
#define GYRO_SENS_500 65.5
#define GYRO_SENS_1000 32.8
#define GYRO_SENS_2000 16.4

#define CALIBRATION_SAMPLES 10000
 
void mpu_init(uint8_t sda_pin, uint8_t scl_pin);
int mpu_set_clock(clock_source_t clock_source);
int mpu_set_rate(uint8_t rate);
int mpu_set_gyro_range(gyro_range_t gyro_range);
int mpu_set_accel_range(accel_range_t accel_range);
int mpu_set_sleep(int enabled);
#endif