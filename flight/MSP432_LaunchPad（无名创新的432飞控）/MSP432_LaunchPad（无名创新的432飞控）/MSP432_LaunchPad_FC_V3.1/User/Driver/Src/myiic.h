#ifndef __MYIIC_H
#define __MYIIC_H

#define SCL_H         GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN5)
#define SCL_L         GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN5)
#define SDA_H         GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN4)
#define SDA_L         GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN4)
#define SDA_read      GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN4)

void Init_I2C(void);
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data) ;
uint8_t i2cRead(uint8_t addr, uint8_t regAddr);

void Init_I2C0(void);
void i2c0Write(uint8_t addr, uint8_t regAddr, uint8_t data) ;
uint8_t i2c0Read(uint8_t addr, uint8_t regAddr);


void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);
int16_t Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);
void Nbyte_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address,uint16_t N,uint8_t *buf);


#endif
















