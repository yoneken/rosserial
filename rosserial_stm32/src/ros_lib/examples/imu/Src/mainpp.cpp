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
#include <ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

ros::NodeHandle nh;

volatile bool flag = false;
uint8_t tbuf[64];
uint8_t rbuf[64];

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
  //thermo.publish(&img);
  flag = false;
}

void setup(void)
{
  nh.initNode();
  //nh.advertise(thermo);
}

void loop(void)
{
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

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
  }

  nh.spinOnce();
  HAL_Delay(100);
}

