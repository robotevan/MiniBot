#ifndef MPU_H
#define MPU_H
#include <stdint.h>
#include <MPURegisters.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/i2c.h>

#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0  // ENABLE FOR SLAVE ONLY
#define I2C_MASTER_RX_BUF_DISABLE 0  // ENABLE FOR SLAVE ONLY
#define ACK_CHECK_EN 0x1 // Acknowledge from slave
#define NACK_VAL 0x1 
#define WRITE_BIT I2C_MASTER_WRITE 
#define READ_BIT I2C_MASTER_READ

#define CALIBRATION_SAMPLES 10000
#define ACCEL_2G_Z_OFF 16384.0
#define ACCEL_4G_Z_OFF 8192.0
#define ACCEL_8G_Z_OFF 4096.0
#define ACCEL_16G_Z_OFF 2048.0

void init_i2c_master_();    
void write_byte_(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t read_byte_(uint8_t addr, uint8_t reg);
void set_mpu_sleep_(uint8_t state);
void set_mpu_accel_scale_(mpu_accel_range_t scale);
void set_mpu_gyro_scale_(mpu_gyro_range_t scale);

void init_mpu_6050();
void calibrate_offsets();
void read_gyro_raw(float *pitch, float *roll, float *yaw);
void read_gyro(float *pitch, float *roll, float *yaw);
void read_accel_raw(float *x, float *y, float *z);
void read_accel(float *x, float *y, float *z);
void read_temperature(float *temperature);

#endif