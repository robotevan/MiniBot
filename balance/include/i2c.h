#ifndef I2C_H
#define I2C_H
#include <driver/i2c.h>
#include <stdint.h>

#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SCL_PIN 19  
#define I2C_MASTER_SDA_PIN 18 
#define I2C_MASTER_FREQ_HZ 100000  
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0   
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0  
#define NACK_VAL 0x1

void i2c_init();
esp_err_t i2c_read_slave(uint8_t* data_rd, size_t size, uint8_t slave_address);
esp_err_t i2c_write_slave(uint8_t* data_rd, size_t size, uint8_t slave_address);

#endif