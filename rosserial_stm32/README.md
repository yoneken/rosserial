# rosserial_stm32

## Limitation
Currently, this code is focused on STM32F3xx series.
If you use the package for other series, please edit rosserial_stm32/src/ros_lib/STM32Hardware.h .
After that, we appreciate you sharing your code :)

## Generate code
$ cd _target_sw4stm32_workspace_
$ rosrun rosserial_stm32 make_libraries.py .
*Never forget to change the project type to cpp project in SW4STM32!!*

## examples
See rosserial_stm32/src/ros_lib_exaples
