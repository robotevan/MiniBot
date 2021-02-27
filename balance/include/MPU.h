#ifndef MPU_H
#define MPU_H
#include <stdint.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <math.h>

// i2c macros
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0  // ENABLE FOR SLAVE ONLY
#define I2C_MASTER_RX_BUF_DISABLE 0  // ENABLE FOR SLAVE ONLY
#define ACK_CHECK_EN 0x1 // Acknowledge from slave
#define NACK_VAL 0x1 
#define WRITE_BIT I2C_MASTER_WRITE 
#define READ_BIT I2C_MASTER_READ

// MPU registers
#define MPU_DEFAULT_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
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

// MPU accel/gyro types and values
#define ACCEL_2G_SENS 16384.0
#define ACCEL_4G_SENS 8192.0
#define ACCEL_8G_SENS 4096.0
#define ACCEL_16G_SENS 2048.0

#define RAD_T_DEG 57.29577951308
 
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


// MPU output types
typedef struct mpu_accel{
    float x;
    float y;
    float z;
}mpu_accel_t; 

typedef struct mpu_gyro{
    float roll;
    float pitch;
    float yaw;
}mpu_gyro_t; 

#define CALIBRATION_SAMPLES 10000 // number of samples taken to calculate offsets


// i2c initialization
void init_i2c_master_();    
void write_byte_(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t read_byte_(uint8_t addr, uint8_t reg);


// mpu functions
void init_mpu_6050();
void set_mpu_sleep(uint8_t state);
void set_mpu_accel_scale(mpu_accel_range_t scale);
void set_mpu_gyro_scale(mpu_gyro_range_t scale);
void calibrate_offsets_(); // gets called once by init_mpu_6050();

// mpu output functions
mpu_gyro_t read_gyro_raw();
mpu_gyro_t read_gyro();
mpu_accel_t read_accel_raw();
mpu_accel_t read_accel();
float read_angle_xaxis();
void read_temperature(float *temperature);

#endif