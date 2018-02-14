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
  uint8_t *data;
  void (*callback)(uint8_t dat[]);

  SPI_Mem(unsigned short int chip_, uint8_t address_, void (*callback_)(uint8_t dat[]), short num_, const uint8_t data_[])
    : chip(chip_), data_num(num_), data(new uint8_t[1+num_]), callback(callback_)
  {
    data[0] = address_;
    if(data_ != NULL){
      memcpy(&(data[1]), data_, num_);
    }else{
      memset(&(data[1]), 0xff, num_+1);
    }
  }
  virtual ~SPI_Mem(){ delete[] data; }

  SPI_Mem& Copy(const SPI_Mem& mem)
  {
    if(this == &mem){ return *this; }
    delete[] data;
    data = new uint8_t[1+mem.data_num];
    memcpy((void *)data, mem.data, 1+mem.data_num);
    return *this;
  }
  SPI_Mem& operator =(const SPI_Mem& mem){ return Copy(mem); }
};

volatile bool flag_transmit = false;
std::queue<SPI_Mem>spiq;
uint8_t rbuf[64];


void SPI_Mem_Transmit(void){
  if((!flag_transmit) && (!spiq.empty())){
    flag_transmit = true;
    SPI_Mem m = spiq.front();
    HAL_GPIO_WritePin(GPIOA, m.chip, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, m.data, rbuf, 1+m.data_num);
  }
}

void SPI_Mem_Write(unsigned short int chip, uint8_t address, short num, const uint8_t data[]){
  spiq.push(SPI_Mem(chip, address, NULL, num, data));
  SPI_Mem_Transmit();
}

void SPI_Mem_Read(unsigned short int chip, uint8_t address, void (*callback)(uint8_t dat[]), short num){
  spiq.push(SPI_Mem(chip, address | 0x80, callback, num, NULL));
  SPI_Mem_Transmit();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  SPI_Mem m = spiq.front();
  HAL_GPIO_WritePin(GPIOA, m.chip, GPIO_PIN_SET);
  if(m.callback != NULL) m.callback(&(rbuf[1]));
  spiq.pop();
  flag_transmit = false;
  SPI_Mem_Transmit();
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

void setup(void)
{
  nh.initNode();
  //nh.advertise(thermo);
}

void loop(void)
{
  SPI_Mem_Read(CS_IMU_Pin, WHO_AM_I, cb_whoami, 1);
  SPI_Mem_Read(CS_IMU_Pin, ACCEL_XOUT_H, cb_sense, 14);
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

