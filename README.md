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

# CubeMX configuration

- Init all peripherals in their default mode? YES
- Project Manager -> Driver Selector -> All drivers must be LL not HAL (do this at the end)

## Clocks

- SYSCLK soure must be PLLCLK = 64MHz
- PLLCLK multiplier x16 = 64MHz --> This is the base clock for CPU & TIM6
- PLLCLK source HSI/2 = 4MHz
- APB1 prescaler /2 = PCLK1 = 32MHz (must be less than 36MHz)

## TIM6

- Activated: YES
- Prescaler: 1 (means divide frequency by 2)
- Counter mode: UP
- Counter Period (Autoreaload reg): 49 (means divide frequency by 50)
- Auto reload preload: Enable
- Trigger event selection: Update Event
- Update event is generated every time the counter reaches the autoreload 
  value and goes back to 0. This generates the trigger output for DAC1
- NVIC, DMA settings: NONE

## DAC1

- OUT1 configuration: YES
- Pin PA4 function: DAC1_OUT1
- Output buffer: Enable
- Trigger: Timer 6 trigger out event
- Wave generation mode: Disabled
- DMA Settings: DAC1 CH1 / DMA1 CH3 / Memory to periph / Priority Low
- DMA Mode: Circular. Data width: Half word both periph & men

## DMA1

- No need to change settings on CubeMX

