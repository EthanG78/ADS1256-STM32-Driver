## ADS1256 STM32 Driver
A serial peripheral interface (SPI) driver for the [ADS1256 low noise, 24-bit analog-to-digital converter](https://www.ti.com/lit/ds/symlink/ads1256.pdf?ts=1715603140406&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1256) written in C for the ARM-based Cortex-M7 MCU running on STM32F7 devices. This driver has been tested using the STM NUCLEO-F746ZG board.

### Including the driver in your STM32 project
If you are building your STM32 project using the STM32CubeIDE, simply place the ADS1256.c and ADS1256.h files within the Src and Inc directories of your project, respectively.

In the future, time permitting, I will look into building this driver as a static library through the use of the [stm32-cmake project](https://github.com/ObKo/stm32-cmake/tree/master).

### Notice on STM32F7 HAL
This driver was originally written using the STM32F7 (hardware abstraction layer) HAL functions for communicating over SPI. After some of my own testing, with confirmation from the [community](https://community.st.com/t5/stm32-mcus-products/spi-too-slow/m-p/251638), it has been made clear that the STM32F7 HAL is insufficient for high-speed applications. Thus, I have replaced all HAL function calls with their equivalent register calls.

#### Author: Ethan Garnier