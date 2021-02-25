#include "MPU.h"

// initialize device as MASTER I2C Device
void _init_i2c_master(){
    int i2c_master_port = I2C_MODE_MASTER;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(i2c_master_port, &conf);
    esp_err_t ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void _write_byte(uint8_t addr, uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN); // device address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // access internal register
    i2c_master_write(cmd, &data, 1, ACK_CHECK_EN); // write byte to register
    i2c_master_stop(cmd); // send stop bit
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); // send the i2c command over the bus
    i2c_cmd_link_delete(cmd);
}

uint8_t _read_byte(uint8_t addr, uint8_t reg){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN); // device address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // access internal register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    uint8_t data;
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

void _set_mpu_sleep(uint8_t state){
    if (state <= 1){
        uint8_t data = (state << 6);
        _write_byte(MPU_DEFAULT_ADDR, MPU_PWR_MGMT_1, data);
    }    
}

void _set_mpu_accel_scale(mpu_accel_range_t scale){
    _write_byte(MPU_DEFAULT_ADDR, MPU_ACCEL_CONFIG, scale);
}

void _set_mpu_gyro_scale(mpu_gyro_range_t scale){
    _write_byte(MPU_DEFAULT_ADDR, MPU_GYRO_CONFIG, scale);
}

void init_mpu_6050(){
    // initialize the i2c driver
    _init_i2c_master();
    _set_mpu_sleep(0);  
    _write_byte(MPU_DEFAULT_ADDR, MPU_CONFIG, (uint8_t)3); // 44hz low pass filter
    _set_mpu_accel_scale(MPU_2G_ACCEL_MODE);
    _set_mpu_gyro_scale(MPU_250_GYRO_MODE);
}

void read_gyro_raw(float *pitch, float *roll, float *yaw){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t pitch_val = _read_byte(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, ACCEL_XOUT_L);
    uint16_t roll_val = _read_byte(MPU_DEFAULT_ADDR, ACCEL_YOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, ACCEL_YOUT_L);
    uint16_t yaw_val = _read_byte(MPU_DEFAULT_ADDR, ACCEL_ZOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, ACCEL_ZOUT_L);
    *pitch = (float)pitch_val;
    *roll = (float)roll_val;
    *yaw = (float)yaw_val;
}

void read_accel_raw(float *x, float *y, float *z){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t x_val = _read_byte(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, GYRO_XOUT_L);
    uint16_t y_val = _read_byte(MPU_DEFAULT_ADDR, GYRO_YOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, GYRO_YOUT_L  );
    uint16_t z_val = _read_byte(MPU_DEFAULT_ADDR, GYRO_ZOUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, GYRO_ZOUT_L);
    *x = (float)x_val;
    *y = (float)y_val;
    *z = (float)z_val;
}

void read_temperature(float *temperature){
    uint16_t t_val = _read_byte(MPU_DEFAULT_ADDR, TEMP_OUT_H) << 8 | _read_byte(MPU_DEFAULT_ADDR, TEMP_OUT_L);
    *temperature = (float)t_val;
}