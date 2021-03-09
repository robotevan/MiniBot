#include "MPU.h"

float accel_sensivity = 0;
float gyro_sensivity = 0;

void mpu_init(uint8_t sda_pin, uint8_t scl_pin)
{
    i2c_init(sda_pin, scl_pin);
    mpu_set_clock(PLL_X_GYRO);
    mpu_set_gyro_range(GYRO_RANGE_250);
    mpu_set_accel_range(ACCEL_RANGE_2G);
    print_bits(MPU_SLAVE_ADDR, MPU_PWR_MGMT_1);
    print_bits(MPU_SLAVE_ADDR, MPU_GYRO_CONFIG);
    print_bits(MPU_SLAVE_ADDR, MPU_ACCEL_CONFIG);
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

int mpu_set_sleep(int enabled){
    int ret = i2c_write_bit(MPU_SLAVE_ADDR, MPU_PWR_MGMT_1, MPU_SLEEP_BIT, enabled);
    return ret;
}
