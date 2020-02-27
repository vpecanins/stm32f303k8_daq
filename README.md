# stm32f303k8_daq
Data acquisition, ADC, DAC with the NUCLEO-F303K8 mini board

This is just a write down so I can remember the steps later.

# NOTE

STM32F303x8 has only 1 DMA controller: ADC & DAC share the same bus. So when ADC & DAC
are used at the same time, sample rates are lower, incorrect. Especially in the ADC, even if the priority of the ADC ISR is higher than the DAC request.

Bigger micros of the STM32F303 series have two independent DMAs to use ADC & DAC
at the same time without penalty.

- Toggle ADC active / inactive: send 'a' to UART
- Toggle DAC active / inactive: send 'd' to UART

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

- ADC1 Ch4 on STM32 _PA3_ pin, NUCLEO label _A2_
- DMA1 Ch1 gets samples from ADC as they are available

- OPAMP2 mode FOLLOWER as driver for the ADC input.
- OPAMP2 input STM32 pin _PA7_ NUCLEO label _A6_
- OPAMP2 output STM32 pin _PA6_ NUCLEO label _A5_
- You should REMOVE SB16 & SB18 from NUCLEO-F303K8 to allow use of _PA6_(_A5_)
- You should CONNECT MANUALLY NUCLEO label _A5_ (opamp out) with NUCLEO label _A2_ (ADC in)

# CubeMX configuration

- Init all peripherals in their default mode? YES
- Project Manager -> Driver Selector -> All drivers must be LL not HAL (do this at the end)

## Clocks

- SYSCLK soure must be PLLCLK = 64MHz
- PLLCLK multiplier x16 = 64MHz --> This is the base clock for CPU & TIM6
- PLLCLK source HSI/2 = 4MHz
- APB1 prescaler /2 = PCLK1 = 32MHz (must be less than 36MHz)
- ADC1,2 prescaler = 16 -> ADC Clock = 4 MHz

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

## ADC1

- IN4 Single-ended: Enabled
- Mode: Independent mode
- Clock prescaler: ADC asynchronous clock
- Resolution 12 bits
- Right alignment
- Scan: Disabled
- Continuous conversion mode: ENABLE
- Discontinuous mode: Disabled
- DMA continuous requests: DISABLED
- Regular conversion: ENABLE
- No. conversions 1 (more channels can be easily added here)
- Rank 1: Channel 4
- Sampling time 19.5 cycles

Note: Sample rate = Clock / ADCPrescaler / (TSampl+TSar) = 64MHz / 16 / (19.5+12.5) = 125KHz

- Injected conversion: DISABLE

- DMA Settings: ADC1 / DMA1 CH1 / Periph to mem / Priority Low
- DMA Mode: Normal, Half width both periph & mem

## OPAMP2

- Mode FOLLOWER: PA7 = in, PA6 = out

# Configuration of peripherals NOT done by CubeMX

Add user code between USER CODE BEGIN / USER CODE END tags

## Define DAC & ADC buffer

Declare in main.h so it can be accessed everywhere

```c
/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ADC_BUF_LEN 512
#define DAC_BUF_LEN 16
/* USER CODE END EM */

```

Define in main.c

Not sure if alignment is necessary

```c
/* USER CODE BEGIN PV */
__attribute__((aligned (16))) uint16_t dac_buf [DAC_BUF_LEN] = {
		0x0400, 0x0200, 0x0400, 0x0600, 0x0800, 0x0A00, 0x0C00, 0x0E00,
		0x0F00, 0x0E00, 0x0C00, 0x0A00, 0x0800, 0x0600, 0x0400, 0x0200} ;
		
__attribute__((aligned (16))) uint16_t adc_buf [ADC_BUF_LEN] = {0};
/* USER CODE END PV */
```

## TIM6

This is enough to start generating the trigger

```c
  /* USER CODE BEGIN TIM6_Init 2 */
  LL_TIM_EnableCounter(TIM6);
  /* USER CODE END TIM6_Init 2 */
```

## DAC

DMA initialization for DAC

```c
  /* USER CODE BEGIN DAC1_Init 1 */

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
  		(uint32_t)dac_buf,
			LL_DAC_DMA_GetRegAddr(DAC1,LL_DAC_CHANNEL_1, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED),
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DAC_BUF_LEN);

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  /* USER CODE END DAC1_Init 1 */
  
```

Enable DAC

```c
  /* USER CODE BEGIN DAC1_Init 2 */

	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_1);

  /* USER CODE END DAC1_Init 2 */
```

## ADC

DMA Initialization for ADC

```c
  /* USER CODE BEGIN ADC1_Init 1 */

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
  			LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
    		(uint32_t)adc_buf,
  			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_BUF_LEN);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  /* USER CODE END ADC1_Init 1 */
```

ADC calibration using busy waits, and enable

```c
/* USER CODE BEGIN ADC1_Init 2 */

  LL_ADC_EnableInternalRegulator(ADC1);

  uint32_t n=0;
  n=1000000;
  while (n--);

  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);

  n=1000000;
  while (n--);

  while (LL_ADC_IsCalibrationOnGoing(ADC1)) {

  };

  n=2000000;
  while (n--);

  LL_ADC_Enable(ADC1);

  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
  {
  	n=200;
  	while (n--);
  };

  // Ready to start conversion
  // LL_ADC_REG_StartConversion(ADC1);

  /* USER CODE END ADC1_Init 2 */
```

## OPAMP2

Yes, you have to ENABLE opamp manually:

```c
  /* USER CODE BEGIN OPAMP2_Init 2 */
  LL_OPAMP_Enable(OPAMP2);
  /* USER CODE END OPAMP2_Init 2 */
```

# Important code generator issues

Make sure when CubeMX generates initialization code, the DMA is initialized 
before DAC1:

```c
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
```

The `MX_DAC1_Init();` does some changes on DMA registers too so DMA clocks must
be initialized before.

## Notice to ST developers

In one ocasion CubeMX generated the following initialization code (this is outside
USER CODE section)

```c
/* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
```

There is a BUG: DAC1 is initialized BEFORE DMA clocks are enabled, 
so DMA configuration inside `MX_DAC1_Init();` does not work. 

Workaround: Initialize DMA clocks two times:

```c
/* USER CODE BEGIN SysInit */
  // FIXME: DMA must be initialized before DAC1_INIT because it writes DMA registers!!!
  MX_DMA_Init(); 
  /* USER CODE END SysInit */

/* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
```

Sometimes CubeMX produces the correct order, sometimes not.

