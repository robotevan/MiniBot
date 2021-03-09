#include "i2c.h"

void i2c_init(uint8_t sda_pin, uint8_t scl_pin)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                        I2C_MASTER_RX_BUF_DISABLE,
                        I2C_MASTER_TX_BUF_DISABLE, 0);
}

void i2c_destroy()
{
    i2c_driver_delete(I2C_MASTER_NUM);
}

esp_err_t i2c_write_slave(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create cmd link
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1, ACK_CHECK_EN); // write device address to high 8 bits
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN); // write internal register to low 8 bits
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN); // write data to reg
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // send command
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_slave(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint32_t read_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // access the location of the internal register to be read
    i2c_master_write_byte(cmd, slave_addr << 1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL){ // if failed to access internal register
        return ret;
    }
    // start reading bytes from device
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | 1, ACK_CHECK_EN);
    while (read_len) {  // read bytes and store in read_buf
        i2c_master_read_byte(cmd, read_buf, ACK_CHECK_EN);
        read_buf++; // move to next byte in read_buf
        read_len--; // decrement read by one byte, continue until len == 0
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


esp_err_t i2c_write_bit(uint8_t slave_addr, uint8_t reg_addr, uint8_t bit, uint8_t bit_val)
{
    uint8_t register_contents;
    esp_err_t ret;
    ret = i2c_read_slave(slave_addr, reg_addr, &register_contents, 1); // read contents of single register
    if (ret == ESP_FAIL){
        return -1;
    }
    if (bit_val){
        register_contents |= (uint8_t)(1<<bit); // set bit
    }else{
        register_contents &= ~(uint8_t)(1<<bit); // clear bit
    }
    ret = i2c_write_slave(slave_addr, reg_addr, register_contents);
    if (ret == ESP_FAIL){
        return -1;
    }
    return 1;
}


int print_bits(uint8_t slave_addr, uint8_t reg_addr)
{
    uint8_t register_contents = 0xFF;
    esp_err_t ret;
    ret = i2c_read_slave(slave_addr, reg_addr, &register_contents, 1); // read contents of single register
    if (ret == ESP_FAIL) {
        return -1;
    }
    for (int i = 7; i >= 0; i--){
        (register_contents & (1<<i)) ? putchar('1') : putchar('0');
    }
    printf("\n");
    return -1;
}