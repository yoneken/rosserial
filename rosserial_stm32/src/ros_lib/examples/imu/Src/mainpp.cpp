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
  SPI_Mem(const SPI_Mem& mem)
    : chip(mem.chip), data_num(mem.data_num), data(new uint8_t[1+mem.data_num]), callback(mem.callback)
  { memcpy(data, mem.data, data_num + 1); }

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
  SPI_Mem_Write(CS_IMU_Pin, USER_CTRL, 1, (uint8_t*)uctr);

  I2C_MST_CTRL_REG i2cmctr[1] = {0};
  i2cmctr[0].I2C_MST_CLK = _400kHz;	// Set I2c Master clock to 400kHz
  SPI_Mem_Write(CS_IMU_Pin, I2C_MST_CTRL, 1, (uint8_t*)i2cmctr);

  // Reset AK8963 (Magnetometer implemented on mpu9250) through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = CNTL2;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);

  AK8963_CNTL2_REG cntl2[1] = {0};
  cntl2[0].SRST = 1;	// Enable reset
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl2);

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 1;	// Write 1 byte
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);

  // Initialize AK8963 for continuous sensing mode through I2C
  i2creg[0].BYTE = CNTL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);

  AK8963_CNTL1_REG cntl1[1] = {0};
  cntl1[0].MODE = CONT_MES_MODE2;	// continuous measuring mode
  cntl1[0].BIT = OUT_16BIT;			// measure 16 bit
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl1);

  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);

}

void loop(void)
{
  //SPI_Mem_Read(CS_IMU_Pin, WHO_AM_I, cb_whoami, 1);
  SPI_Mem_Read(CS_IMU_Pin, ACCEL_XOUT_H, cb_sense, 14);

  // Read magnetometer through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  i2caddr[0].I2C_SLV0_RNW = 1;	// read flag
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = HXL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 7;	// Read 7 byte from HXL to ST2
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

