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


void _init_i2c_master();    
void _write_byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t _read_byte(uint8_t addr, uint8_t reg);
void _set_mpu_sleep(uint8_t state);
void _set_mpu_accel_scale(mpu_accel_range_t scale);
void _set_mpu_gyro_scale(mpu_gyro_range_t scale);

void init_mpu_6050();

void read_gyro_raw(float *pitch, float *roll, float *yaw);
void read_gyro(float *pitch, float *roll, float *yaw);
void calibrate_gyro(float *pitch_offset, float *roll_offset, float *raw_offset);

void read_accel_raw(float *x, float *y, float *z);
void read_accel(float *x, float *y, float *z);
void calibrate_accel(float *x_off, float *y_off, float *z_off);

void read_temperature(float *temperature);

#endif