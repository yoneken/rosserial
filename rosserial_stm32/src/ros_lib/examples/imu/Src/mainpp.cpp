/*
 * main.cpp

 *
 *  Created on: 2018/01/02
 *      Author: yoneken
 */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "mpu9250.h"
#include <mainpp.h>
#include <queue>
#include <memory>
#include <ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

class SPI_Mem{
public:
  unsigned short int chip;
  short data_num;
  std::shared_ptr<uint8_t>data;
  void (*callback)(uint8_t dat[]);

  // send message
  SPI_Mem(unsigned short int chip_, uint8_t address_, short num_, const uint8_t data_[])
    : chip(chip_), data_num(num_), data(std::make_shared<uint8_t>(1+num_)), callback(NULL)
  {
    uint8_t *dat = data.get();
    dat[0] = address_;
    memcpy(&(dat[1]), data_, num_);
  }

  // receive message
  SPI_Mem(unsigned short int chip_, uint8_t address_, void (*callback_)(uint8_t dat[]), short num_)
    : chip(chip_), data_num(num_), data(std::make_shared<uint8_t>(1+num_)), callback(callback_)
  {
    uint8_t *dat = data.get();
    dat[0] = address_;
    memset(&(dat[1]), 0xff, num_+1);
  }

  virtual ~SPI_Mem(){ }
  SPI_Mem(const SPI_Mem& mem)
    : chip(mem.chip), data_num(mem.data_num), data(std::make_shared<uint8_t>(1+mem.data_num)), callback(mem.callback)
  { data = mem.data; }

  SPI_Mem& Copy(const SPI_Mem& mem)
  {
    if(this == &mem){ return *this; }
    data = mem.data;
    return *this;
  }

  SPI_Mem& operator =(const SPI_Mem& mem){ return Copy(mem); }
};

std::queue<std::shared_ptr<SPI_Mem>>spiq;
uint8_t rbuf[64];


void SPI_Mem_Transmit(void){
  HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&hspi1);

  volatile static bool mutex = false;
  if((!mutex)&&(state == HAL_SPI_STATE_READY)){	// Only one transmit process is allowed
    mutex = true;
    SPI_Mem *m = spiq.front().get();
    HAL_GPIO_WritePin(GPIOA, m->chip, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, m->data.get(), rbuf, 1+m->data_num);
    mutex = false;
  }
}

void SPI_Mem_Write(unsigned short int chip, uint8_t address, short num, const uint8_t data[]){
  spiq.push(std::make_shared<SPI_Mem>(chip, address, num, data));
  SPI_Mem_Transmit();
}

void SPI_Mem_Read(unsigned short int chip, uint8_t address, void (*callback)(uint8_t dat[]), short num){
  spiq.push(std::make_shared<SPI_Mem>(chip, address | 0x80, callback, num));
  SPI_Mem_Transmit();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  SPI_Mem *m = spiq.front().get();	// This instance must be the same with one which was gotten in SPI_Mem_Trransmit()
  HAL_GPIO_WritePin(GPIOA, m->chip, GPIO_PIN_SET);
  if(m->callback != NULL) m->callback(&(rbuf[1]));
  uint8_t s = spiq.size();
  spiq.pop();
  if(!spiq.empty()) SPI_Mem_Transmit();
}

void cb_whoami(uint8_t dat[]){
  uint8_t tmp = dat[0];	// expect MPU9250-0x71(113), MPU9255-0x73(115)
}

void cb_sense(uint8_t dat[]){
  short acc[3], temp, gyro[3];
  acc[0] = dat[0] << 8 | dat[1];
  acc[1] = dat[2] << 8 | dat[3];
  acc[2] = dat[4] << 8 | dat[5];
  temp = dat[6] << 8 | dat[7];
  gyro[0] = dat[8] << 8 | dat[9];
  gyro[1] = dat[10] << 8 | dat[11];
  gyro[2] = dat[12] << 8 | dat[13];
}

void cb_mag_sense(uint8_t dat[]){
  short magnet[3];
  magnet[0] = dat[0] | dat[1] << 8;
  magnet[1] = dat[2] | dat[3] << 8;
  magnet[2] = dat[4] | dat[5] << 8;
}

void setup(void)
{
  nh.initNode();
  //nh.advertise(thermo);

  // Initialize I2C Master mode of mpu9250
  USER_CTRL_REG uctr[1] = {0};
  uctr[0].I2C_MST_EN = 1;	// Enable the I2C Master I/F module
  uctr[0].I2C_IF_DIS = 1;	// Disable I2C Slave module and put the serial interface in SPI mode only
  //uctr[0].BYTE = 0x30;
  SPI_Mem_Write(CS_IMU_Pin, USER_CTRL, 1, (uint8_t*)uctr);	// expect 0x30(48)

  I2C_MST_CTRL_REG i2cmctr[1] = {0};
  i2cmctr[0].I2C_MST_CLK = _400kHz;	// Set I2c Master clock to 400kHz
  SPI_Mem_Write(CS_IMU_Pin, I2C_MST_CTRL, 1, (uint8_t*)i2cmctr);		// expect 0x0D(13)

  // Reset AK8963 (Magnetometer implemented on mpu9250) through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);	// expect 0x0C(12)

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = CNTL2;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x0B(11)

  AK8963_CNTL2_REG cntl2[1] = {0};
  cntl2[0].SRST = 1;	// Enable reset
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl2);	// expect 0x01(1)

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 1;	// Write 1 byte
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x81(129)

  // Initialize AK8963 for continuous sensing mode through I2C
  i2creg[0].BYTE = CNTL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x0A(10)

  AK8963_CNTL1_REG cntl1[1] = {0};
  cntl1[0].MODE = CONT_MES_MODE2;	// continuous measuring mode
  cntl1[0].BIT = OUT_16BIT;			// measure 16 bit
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl1);	// expect 0x16(22)

  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x81(129)

}

void loop(void)
{
  //SPI_Mem_Read(CS_IMU_Pin, WHO_AM_I, cb_whoami, 1);
  SPI_Mem_Read(CS_IMU_Pin, ACCEL_XOUT_H, cb_sense, 14);

  // Read magnetometer through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  i2caddr[0].I2C_SLV0_RNW = 1;	// read flag
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);	// expect 0x8C(140)

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = HXL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x03(3)

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 7;	// Read 7 byte from HXL to ST2
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x87(135)

  SPI_Mem_Read(CS_IMU_Pin, EXT_SENS_DATA, cb_mag_sense, 7);
  /*
  if(!flag){
    flag = true;

    tbuf[0] = WHO_AM_I | 0x80;
    tbuf[1] = 0xff;
    tbuf[2] = 0xff;
    tbuf[3] = 0xff;
    tbuf[4] = 0xff;
    tbuf[5] = 0xff;
    tbuf[6] = 0xff;
    tbuf[7] = 0xff;
    //tbuf[0] = ACCEL_XOUT_H | 0x80;

    HAL_GPIO_WritePin(GPIOA, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tbuf, 1, 1000);
    HAL_SPI_Receive(&hspi1, rbuf, 8, 1000);	// expect MPU9250-0x71(113), MPU9255-0x73(115)
    //HAL_SPI_TransmitReceive(&hspi1, tbuf, rbuf, 8, 1000);
    HAL_GPIO_WritePin(GPIOA, CS_IMU_Pin, GPIO_PIN_SET);
  }
  */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

  nh.spinOnce();
  HAL_Delay(1000);
}

