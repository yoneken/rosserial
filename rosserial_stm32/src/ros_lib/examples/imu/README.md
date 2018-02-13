# IMU (an example for rosserial_stm32)

## HAL
- [STM32CubeMX](http://www.st.com/ja/development-tools/stm32cubemx.html)

## Target board
- [Nucleo-F303K8(STM32F303)](http://www.st.com/ja/evaluation-tools/nucleo-f303k8.html)

## Device
- [MPU9250/9255 Breakout](http://tinkersphere.com/sensors/1875-9dof-accelerometer-gyro-magnetometer-breakout-mpu-9250.html)

## Using Peripherals
- USART2 (through DMA)
- Timer2
- SPI1 (through DMA)

## Wire
|Name    |Nucreo board|STM32F303(LQFP32)|MPU9250 board|MPU9250    |
|--------|------------|-----------------|-------------|-----------|
|Vin (5V)|CN4-P4      |                 |VIN          |           |
|GND     |CN4-P2      |VSS(16, 32)      |GND          |GND(18)    |
|MOSI    |CN4-P6      |PA7(20)          |SDA          |SDI(24)    |
|MISO    |CN4-P7      |PA6(21)          |AD0          |SDO(9)     |
|SCLK    |CN4-P8      |PA5(22)          |SCL          |SCLK(23)   |
|CS      |CN4-P9      |PA4(23)          |NCS          |nCS(22)    |
