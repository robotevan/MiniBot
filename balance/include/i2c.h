#ifndef I2C_H
#define I2C_H
#include <driver/i2c.h>
#include <stdint.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000  
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0   
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0  
#define NACK_VAL 0x1

/*
    Initialize i2c, ueses port 0, choose pins for scl and sda
*/
void i2c_init(uint8_t sda_pin, uint8_t scl_pin);

/*
    write a single byte to an internal register of the slav address
    uint8_t slave_addr, address of the device
    uint8_t reg_addr, address of the internal register of a slave device
    uint8_t data, data being writen to the slave's register
*/
esp_err_t i2c_write_slave(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/*
    Read N bytes from an i2c device
    uint8_t slave_addr, address of the device
    uint8_t reg_addr, address of the internal register of a slave device
    uint8_t *read_buf, buffer of bytes to store data
    uint32_t read_len, number of bytes to read to read_buf
*/
esp_err_t i2c_read_slave(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint32_t read_len);


#endif