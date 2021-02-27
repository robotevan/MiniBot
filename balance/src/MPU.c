#include "MPU.h"

// offsets for the accel/gyro
float ax_offset, ay_offset, az_offset = 0;
float gr_offset, gp_offset, gy_offset = 0;
mpu_accel_range_t accel_range = MPU_2G_ACCEL_MODE;
mpu_gyro_range_t gyro_range = MPU_250_GYRO_MODE;
float mpu_sensitivity = ACCEL_2G_SENS;



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

void set_mpu_sleep(uint8_t state){
    if (state <= 1){
        uint8_t data = (state << 6);
        write_byte_(MPU_DEFAULT_ADDR, MPU_PWR_MGMT_1, data);
    }    
}

void set_mpu_accel_scale(mpu_accel_range_t scale){
    write_byte_(MPU_DEFAULT_ADDR, MPU_ACCEL_CONFIG, scale);
    accel_range = scale;
    if(scale == MPU_2G_ACCEL_MODE){
        mpu_sensitivity = ACCEL_2G_SENS;
    } else if(scale == MPU_4G_ACCEL_MODE){
        mpu_sensitivity = ACCEL_4G_SENS;
    } else if(scale == MPU_8G_ACCEL_MODE){
        mpu_sensitivity = ACCEL_8G_SENS;
    } else if(scale == MPU_4G_ACCEL_MODE){
        mpu_sensitivity = ACCEL_16G_SENS;
    }
}

void set_mpu_gyro_scale(mpu_gyro_range_t scale){
    write_byte_(MPU_DEFAULT_ADDR, MPU_GYRO_CONFIG, scale);
    gyro_range = scale;
}

void init_mpu_6050(){
    // initialize the i2c driver
    init_i2c_master_();
    set_mpu_sleep(0);  
    write_byte_(MPU_DEFAULT_ADDR, MPU_CONFIG, (uint8_t)3); // 44hz low pass filter
    set_mpu_accel_scale(MPU_4G_ACCEL_MODE);
    set_mpu_gyro_scale(MPU_500_GYRO_MODE);
    calibrate_offsets_();
}

void calibrate_offsets_(){
    ESP_LOGI("Calibration" ,"Starting calibration, leave on flat surface");
    mpu_accel_t accel;
    mpu_gyro_t gyro;
    ax_offset = 0, ay_offset = 0, az_offset = 0, gr_offset = 0, gp_offset = 0, gy_offset = 0;
    // sum up offsets
    for (int i = 0; i < CALIBRATION_SAMPLES;i++){ 
        gyro = read_gyro_raw();
        gr_offset += gyro.roll;
        gp_offset += gyro.pitch;
        gy_offset += gyro.yaw;

        accel = read_accel_raw();
        ax_offset += accel.x;
        ay_offset += accel.y;
        ax_offset += accel.z;
    }
    // calculate the average offset
    gr_offset /= CALIBRATION_SAMPLES;
    gp_offset /= CALIBRATION_SAMPLES;
    gy_offset /= CALIBRATION_SAMPLES;

    ax_offset /= CALIBRATION_SAMPLES;
    ay_offset /= CALIBRATION_SAMPLES;
    az_offset /= CALIBRATION_SAMPLES;
    az_offset -= mpu_sensitivity; // subtract effects of gravity from z axis
    ESP_LOGI("Calibration" ,"Done Dalibration");
}


mpu_gyro_t read_gyro_raw(){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t roll_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_XOUT_L);
    uint16_t pitch_val = read_byte_(MPU_DEFAULT_ADDR, ACCEL_YOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_YOUT_L);
    uint16_t yaw_val = read_byte_(MPU_DEFAULT_ADDR, ACCEL_ZOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, ACCEL_ZOUT_L);
    mpu_gyro_t gyro_out = {
        .roll = (float)roll_val,
        .pitch = (float)pitch_val,
        .yaw = (float)yaw_val
    };
    return gyro_out;
}

mpu_gyro_t read_gyro(){
    mpu_gyro_t gyro_raw = read_gyro_raw();
    // apply offsets
    gyro_raw.roll = round((gyro_raw.roll - gr_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
    gyro_raw.pitch = round((gyro_raw.pitch - gp_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
    gyro_raw.yaw = round((gyro_raw.yaw - gy_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
    return gyro_raw;
}

mpu_accel_t read_accel_raw(){
    // Each value is 16 bits (two 8 bit registers)
    uint16_t x_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_XOUT_L);
    uint16_t y_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_YOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_YOUT_L);
    uint16_t z_val = read_byte_(MPU_DEFAULT_ADDR, GYRO_ZOUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, GYRO_ZOUT_L);
    mpu_accel_t accel_out = {
        .x = x_val,
        .y = y_val,
        .z = z_val
    };
    return accel_out;
}

mpu_accel_t read_accel(){
    mpu_accel_t accel_raw = read_accel_raw();
    accel_raw.x = round((accel_raw.x - ax_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
	accel_raw.y = round((accel_raw.y - ay_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
	accel_raw.z = round((accel_raw.z - az_offset) * 1000.0 / mpu_sensitivity) / 1000.0;
    return accel_raw;
}

float read_angle_xaxis(){
    mpu_accel_t accel = read_accel();
    return atan2(accel.z, accel.y) * RAD_T_DEG - 90.0;
}

void read_temperature(float *temperature){
    uint16_t t_val = read_byte_(MPU_DEFAULT_ADDR, TEMP_OUT_H) << 8 | read_byte_(MPU_DEFAULT_ADDR, TEMP_OUT_L);
    *temperature = (float)t_val;
}
