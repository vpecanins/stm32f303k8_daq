# stm32f303k8_daq
Data acquisition, ADC, DAC with the NUCLEO-F303K8 mini board

This is just a write down so I can remember the steps later.

# Developed with
- STM32CubeIDE v.1.1.0 build 4551_20191014-1140
- Host OS Ubuntu 18.04.3 LTS
- Board: NUCLEO-F303K8 (Arduino Nano form factor) 
- SB16 & SB18 ON, as shipped from factory
- MCU: STM32F303K8 from STMicroelectronics
- Low Level LL libraries for all STM32 peripherals
- STM32Cube FW_F3 v.1.11.0

# Peripherals used
- TIM6 basic timer generates trigger for DAC1
- DAC1 Ch1 on STM32 _PA4_ pin, NUCLEO label _A3_.
- DAC1 receives trigger from TIM6 and generates DMA request
- DMA1 Ch3 gets request from DAC1 each sample
- DMA1 fetches data from SRAM and delivers to DAC1

