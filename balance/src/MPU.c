#include "MPU.h"

float accel_sensivity = 0;
float gyro_sensivity = 0;

double x_accel_offset = 0, y_accel_offset = 0, z_accel_offset = 0;
double x_gyro_offset = 0, y_gyro_offset = 0, z_gyro_offset = 0;

void mpu_init(uint8_t sda_pin, uint8_t scl_pin)
{
    i2c_init(sda_pin, scl_pin);
    mpu_reset();
    mpu_set_clock(PLL_X_GYRO);
    mpu_set_gyro_range(GYRO_RANGE_250);
    mpu_set_accel_range(ACCEL_RANGE_2G);
    mpu_set_sleep(0);
}

int mpu_reset()
{
    esp_err_t ret = i2c_write_bit(MPU_SLAVE_ADDR, MPU_PWR_MGMT_1, MPU_RESET_BIT, 1);
    if (ret == ESP_FAIL){
        return -1;
    }
    return 1;
    vTaskDelay(100);
}

int mpu_set_clock(clock_source_t clock_source)
{
    i2c_write_slave(MPU_SLAVE_ADDR, MPU_PWR_MGMT_1, clock_source);
    return 1;
}

int mpu_set_rate(uint8_t rate)
{
    esp_err_t ret = i2c_write_slave(MPU_SLAVE_ADDR, MPU_SMPLRT_DIV, rate);
    if (ret == ESP_FAIL){
        return -1;
    }
    return 1;
}

int mpu_set_gyro_range(gyro_range_t gyro_range)
{
    esp_err_t ret = i2c_write_slave(MPU_SLAVE_ADDR, MPU_GYRO_CONFIG, gyro_range);
    if (ret == ESP_FAIL){
        return -1;
    }
    if (gyro_range == GYRO_RANGE_250){
        gyro_sensivity = GYRO_SENS_250;
    }else if (gyro_range == GYRO_RANGE_500){
        gyro_sensivity = GYRO_SENS_500;
    }else if (gyro_range == GYRO_SENS_1000){
        gyro_sensivity = GYRO_SENS_1000;
    }else if (gyro_range == GYRO_SENS_2000){
        gyro_sensivity = GYRO_SENS_2000;
    }
    return 1;
}

int mpu_set_accel_range(accel_range_t accel_range)
{
    esp_err_t ret = i2c_write_slave(MPU_SLAVE_ADDR, MPU_ACCEL_CONFIG, accel_range);
    if (ret == ESP_FAIL){
        return -1;
    }
        if (accel_range == ACCEL_RANGE_2G){
        accel_sensivity = ACCEL_SENS_2G;
    }else if (accel_range == ACCEL_RANGE_4G){
        accel_sensivity = ACCEL_SENS_4G;
    }else if (accel_range == ACCEL_RANGE_8G){
        accel_sensivity = ACCEL_SENS_8G;
    }else if (accel_range == ACCEL_SENS_16G){
        accel_sensivity = ACCEL_SENS_16G;
    }
    return 1;
}

int mpu_set_sleep(int enabled)
{
    esp_err_t ret = i2c_write_bit(MPU_SLAVE_ADDR, MPU_PWR_MGMT_1, MPU_SLEEP_BIT, enabled);
    if (ret == ESP_FAIL){
        return -1;
    }
    return 1;
}

int mpu_set_lp_filter(lp_filter_t lp_filter)
{
    esp_err_t ret = i2c_write_slave(MPU_SLAVE_ADDR, MPU_CONFIG, lp_filter);
    if (ret == ESP_FAIL) {
        return -1;
    }
    return 1;
}

void mpu_read_gyro_raw(float *x_axis, float *y_axis, float *z_axis)
{
    uint8_t x[2];
    uint8_t y[2]; 
    uint8_t z[2];
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_GYRO_XOUT_H, x, 2);
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_GYRO_YOUT_H, y, 2);
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_GYRO_XOUT_H, z, 2);
    *x_axis = (float)((int16_t)(x[0] << 8) | x[1]) / gyro_sensivity;
    *y_axis = (float)((int16_t)(y[0] << 8) | y[1]) / gyro_sensivity;
    *z_axis = (float)((int16_t)(z[0] << 8) | z[1]) / gyro_sensivity;
}

void mpu_read_accel_raw(float *x_axis, float *y_axis, float *z_axis)
{
    uint8_t x[2];
    uint8_t y[2]; 
    uint8_t z[2];
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_ACCEL_XOUT_H, x, 2);
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_ACCEL_YOUT_H, y, 2);
    i2c_read_slave(MPU_SLAVE_ADDR, MPU_ACCEL_ZOUT_H, z, 2);
    *x_axis = (float)((int16_t)(x[0] << 8) | x[1]) / accel_sensivity;
    *y_axis = (float)((int16_t)(y[0] << 8) | y[1]) / accel_sensivity;
    *z_axis = (float)((int16_t)(z[0] << 8) | z[1]) / accel_sensivity;
}


int mpu_calibrate()
{
    float x,y,z = 0;
    double accel_x_off = 0, accel_y_off = 0, accel_z_off = 0;
    double gyro_x_off = 0, gyro_y_off = 0, gyro_z_off = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++){
        mpu_read_accel_raw(&x, &y, &z); // sum up accel vals
        accel_x_off += x;
        accel_y_off += y;
        accel_z_off += z;
        mpu_read_gyro_raw(&x, &y, &z); // sum up gyro vals
        gyro_x_off += x;
        gyro_y_off += y;
        gyro_z_off += z;
    }
    // get average for each and store in offsets
    x_accel_offset = accel_x_off / CALIBRATION_SAMPLES;
    y_accel_offset = accel_y_off / CALIBRATION_SAMPLES;
    z_accel_offset = ACCEL_Z_GRAV_G - accel_z_off / CALIBRATION_SAMPLES;
    x_gyro_offset = gyro_x_off / CALIBRATION_SAMPLES;
    y_gyro_offset = gyro_y_off / CALIBRATION_SAMPLES;
    z_gyro_offset = gyro_y_off / CALIBRATION_SAMPLES;
    return 1;
}

void mpu_read_gyro(float *x_axis, float *y_axis, float *z_axis)
{
    float x, y, z;
    mpu_read_gyro_raw(&x, &y, &z);
    *x_axis -= x - x_gyro_offset;
    *y_axis -= y - y_gyro_offset;
    *z_axis -= z - z_gyro_offset;
}

void mpu_read_accel(float *x_axis, float *y_axis, float *z_axis)
{
    float x, y, z;
    mpu_read_accel_raw(&x, &y, &z);
    *x_axis = x - x_accel_offset;
    *y_axis = y - y_accel_offset;
    *z_axis = z + z_accel_offset;
}