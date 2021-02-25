#ifndef MPUREGISTERS_H
#define MPUREGISTERS_H

#define MPU_DEFAULT_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_CONFIG 0x1A

#define MPU_GYRO_CONFIG 0x1B
typedef enum {
    MPU_250_GYRO_MODE,  // (uint8_t)0
    MPU_500_GYRO_MODE,  // (uint8_t)1
    MPU_1000_GYRO_MODE, // (uint8_t)2
    MPU_2000_GYRO_MODE  // (uint8_t)3
} mpu_gyro_range_t;


#define MPU_ACCEL_CONFIG 0x1C
typedef enum {
    MPU_2G_ACCEL_MODE,  // (uint8_t)0
    MPU_4G_ACCEL_MODE,  // (uint8_t)1
    MPU_8G_ACCEL_MODE, // (uint8_t)2
    MPU_16G_ACCEL_MODE  // (uint8_t)3
} mpu_accel_range_t;


#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#endif