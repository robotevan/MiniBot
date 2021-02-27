#include "MPU.h"

// offsets for the accel/gyro
float ax_offset, ay_offset, az_offset = 0;
float gx_offset, gy_offset, gz_offset = 0;
mpu_accel_range_t accel_range = MPU_2G_ACCEL_MODE;
mpu_gyro_range_t gyro_range = MPU_250_GYRO_MODE;
float mpu_accel_z_offset = ACCEL_2G_Z_OFF;



// initialize device as MASTER I2C Device
void init_i2c_master_(){
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


void write_byte_(uint8_t addr, uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN); // device address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // access internal register
    i2c_master_write(cmd, &data, 1, ACK_CHECK_EN); // write byte to register
    i2c_master_stop(cmd); // send stop bit
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); // send the i2c command over the bus
    i2c_cmd_link_delete(cmd);
}

uint8_t read_byte_(uint8_t addr, uint8_t reg){
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

void set_mpu_sleep_(uint8_t state){
    if (state <= 1){
        uint8_t data = (state << 6);
        write_byte_(MPU_DEFAULT_ADDR, MPU_PWR_MGMT_1, data);
    }    
}

void set_mpu_accel_scale_(mpu_accel_range_t scale){
    write_byte_(MPU_DEFAULT_ADDR, MPU_ACCEL_CONFIG, scale);
    accel_range = scale;
    if(scale == MPU_2G_ACCEL_MODE){
        mpu_accel_z_offset = ACCEL_2G_Z_OFF;
    } else if(scale == MPU_4G_ACCEL_MODE){
        mpu_accel_z_offset = ACCEL_4G_Z_OFF;
    } else if(scale == MPU_8G_ACCEL_MODE){
        mpu_accel_z_offset = ACCEL_8G_Z_OFF;
    } else if(scale == MPU_4G_ACCEL_MODE){
        mpu_accel_z_offset = ACCEL_16G_Z_OFF;
    }
}

void set_mpu_gyro_scale_(mpu_gyro_range_t scale){
    write_byte_(MPU_DEFAULT_ADDR, MPU_GYRO_CONFIG, scale);
    gyro_range = scale;
}

void init_mpu_6050(){

    // initialize the i2c driver
    init_i2c_master_();
    set_mpu_sleep_(0);  
    write_byte_(MPU_DEFAULT_ADDR, MPU_CONFIG, (uint8_t)3); // 44hz low pass filter
    set_mpu_accel_scale_(MPU_2G_ACCEL_MODE);
    set_mpu_gyro_scale_(MPU_250_GYRO_MODE);
}

void calibrate_offsets(){
    ESP_LOGE("Calibration" ,"Starting calibration, leave on flat surface");
    float ax_off, ay_off, az_off = 0;
    float gx_off, gy_off, gz_off = 0;
    ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
    // sum up offsets
    for (int i = 0; i < CALIBRATION_SAMPLES;i++){ 
        read_gyro_raw(&gx_off, &gy_off, &gz_off);
        gx_offset += gx_off;
        gy_offset += gy_off;
        gz_offset += gz_off;

        read_accel_raw(&ax_off, &ay_off, &az_off);
        ax_offset += ax_off;
        ay_offset += ay_off;
        ax_offset += az_off;
    }
    // calculate the average offset
    gx_offset /= CALIBRATION_SAMPLES;
    gy_offset /= CALIBRATION_SAMPLES;
    gz_offset /= CALIBRATION_SAMPLES;

    ax_offset /= CALIBRATION_SAMPLES;
    ay_offset /= CALIBRATION_SAMPLES;
    az_offset /= CALIBRATION_SAMPLES;
    az_offset -= mpu_accel_z_offset; // subtract effects of gravity from z axis
    ESP_LOGE("Calibration" ,"Done Dalibration");
}



void read_gyro_raw(float *pitch, float *roll, float *yaw){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t pitch_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_XOUT_L);
    uint16_t roll_val = read_byte_(MPU_DEFAULT_ADDR, ACCEL_YOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_YOUT_L);
    uint16_t yaw_val = read_byte_(MPU_DEFAULT_ADDR, ACCEL_ZOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_ZOUT_L);
    *pitch = (float)pitch_val;
    *roll = (float)roll_val;
    *yaw = (float)yaw_val;
}

void read_accel_raw(float *x, float *y, float *z){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t x_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_L);
    uint16_t y_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_YOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_YOUT_L  );
    uint16_t z_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_ZOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_ZOUT_L);
    *x = (float)x_val;
    *y = (float)y_val;
    *z = (float)z_val;
}

void read_temperature(float *temperature){
    uint16_t t_val = read_byte_(MPU_DEFAULT_ADDR, TEMP_OUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, TEMP_OUT_L);
    *temperature = (float)t_val;
}
