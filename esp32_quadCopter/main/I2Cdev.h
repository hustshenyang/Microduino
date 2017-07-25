#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include "driver/i2c.h"

#define I2C_NUM			1
#define I2C_SDA_IO		33
#define I2C_SCL_IO		32
#define I2C_FREQ_HZ		300000
#define I2C_TX_BUF_DISABLE	0
#define I2C_RX_BUF_DISABLE	0

#define WRITE_BIT  		I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   		I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   	0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  	0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    		0x0         /*!< I2C ack value */
#define NACK_VAL   		0x1         /*!< I2C nack value */

void I2C_init(void);
esp_err_t I2C_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
esp_err_t I2C_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
esp_err_t I2C_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
esp_err_t I2C_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
esp_err_t I2C_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
esp_err_t I2C_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
esp_err_t I2C_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
esp_err_t I2C_writeCmd(uint8_t devAddr, uint8_t command);
esp_err_t I2C_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
esp_err_t I2C_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
esp_err_t I2C_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
esp_err_t I2C_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
esp_err_t I2C_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
esp_err_t I2C_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
esp_err_t I2C_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
esp_err_t I2C_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
esp_err_t I2C_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

#endif /* _I2CDEV_H_ */
