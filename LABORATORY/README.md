# COPY PROJECT
- in CubeIDE fai New > New stm project from an existing .ioc file 
- selezioni il file .ioc del progetto da copiare
- dai il nome che vuoi al progetto (obv diverso da prima)
- copi-incolli il main file

# ENABLE FLOATING POINT PRINTFS
properties > C/C++ build > Settings > MCU settings > check printf_float

# NOTABLE PINS 
PINS:
- green led: PA5
- user button: PC13
- microphone: PA8
- potentiometer: PA1
- encoder: PC7 (TIM3_CH2), PC6 (TIM3_CH1)
- keyboard: PC8-11 OUTPUT (column driver), PC12,13,2,3 INPUT (row readout)
- LCD:
  - PB12-15
	- PB1-2
	- PA4
- USART2:
	- rx: PA3
	- tx: PA2
- I2C: PB9->SDA, PB8->SCL
	- temperature addr: 0b1001000<<1 (watch out to the version of the sensor)
	- accelerator addr: 0b0101000<<1 (watch out to the version of the sensor)
- SPI: (1,4 up to 42Mbps; 2,3 up to 21Mbps)
	- led matrix (daisy-chain configuration, MSB first) update 4ms
	- PA5 (SCK), PA6(MISO), PA7(MOSI), PB6 (FOR LEDMATRIX: after transfer complete, SET and then RESET)
- IR: PB10 (Led attached to TIM2_CH3) and PA10 (receiver USART1RX)


# CHEATSHEET LAB

## GPIO:
- HAL_GPIO_ReadPin(GPIOC_BASE, GPIO_PIN_13)
- HAL_GPIO_WritePin(GPIOA_BASE, GPIO_PIN_5, GPIO_PIN_RESET | GPIO_PIN_SET)
- HAL_GPIO_TogglePin(GPIOA_BASE, GPIO_PIN_5)
INTERRUPT MODE:
  - set the pin to be in GPIO_EXTIx
  - enable in NVIC the EXTI interrupts that include x
  - write function void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  - switch (GPIO_Pin) {case GPIO_PIN_13:}

## TIMERS:
PWM:
- set pin to be active in mode TIMx_CHy
- set TIMx clock source
- set TIMx channel y to be in mode "PWM Generation CHy"
- set prescaler and counterperiod -1
- set pulse of the PWM
- HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_y);

## UART:
DMA: 
- add Tx DMA entry on DMA settings
- enable interrupts
- void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

## ADC:
- (rank: sampling 480 cycles; HAL_ADC_GetValue(jadc))
- IF USING DMA: MX_DMA_Init(); in [USER CODE Init]
- set in the cube window the pin to convert with the adc in ADCx_INy mode
- modify the settings in analog>ADCx: ADC_Regular_ConversionMode: 
  - external trigger conversion source: [software/Timer x Trigger Out event]
  - rank > sampling time: 480 Cycles
- if software trigger: HAL_ADC_Start_IT(&hadc1);
- if not DMA mode: HAL_ADC_GetValue(&hadc1) with values from 0 to 4096 (2^12)
  - if not polling: void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  - if polling: while(HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK) ;
- if DMA mode: 
  - DMA settings > add > Mode: Circular
  - DMA continuous requests: enabled
  - HAL_ADC_Start_DMA(&hadc1, (uint16_t*) data, bufflen); (
  - void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
  - void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
- if time triggered:
  - TIMx set internal clock source and Trigger Event Selection = Update Event
  - HAL_TIM_Base_Start(&htim);
  - HAL_ADC_Start_IT(&hadc1);
  - void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)	
- if more values to be taken: 
  - ADC_Regular_ConversionMode > NumberOfConversions = #values (or of different sensors to be sampled)
  - set the correct Channels in every "Rank" appeared (and 480 cycles)

## I2C: 
half-duplex, only 2 wires for all the connected devices (SCL clock and SDA data) but it's slow (max 400kbps in fast mode). wires are open drain, so we have a pull up that brings them at Vdd if nobody sets nothing
- start condition: SCL=1 and SDA=1->0
- stop condition: SCL=1 and SDA= 0->1
- communication when SCL=0, made of 8bits from master and 1 bit from slave
- address:7 bit+W/Rbit -> ACK or not by the correspondant slave
- RegAddr: 8bit -> ACK or not by the correspondant slave
- Data: 8bit -> ACK
functions:
- HAL_I2C_Master_Transmit(&hi2c1, accAddress, dataBuf, 1, 1000); dataBuf(buffer of uint8_t data to be transmitted)
- HAL_I2C_Master_Receive(&hi2c1, accAddress, accBuf, 2, 1000); accBuf(buffer of uint8_t data that will contain data received)
- HAL_I2C_Master_Receive_DMA(&hi2c1, accAddress, accBuf, 2); accBuf(buffer of uint8_t data that will contain data received)
- void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)	

## SPI: 
(only led matrix) full-duplex, short distance communication, higher speed (42 Mbps SPI1,SPI4, 21 Mbps SPI2,SPI3)
wirings: 
- SCLK: serial clock; 
  CPOL = at what value the clock idles (default low)
  CPHA = 0 if data changes on the trailing edge of prev clock, 1 if data changes on leading edge of current clock (default: 1 Edge)
- MISO: Master in Slave out,
- MOSI: Master out Slave in,
- CS: chip select, must be pulled low to enable communication with selected device
Virtual ring topology -> master sends a bit to the slave, that reads it; at the same time slave sends a bit to master and master reads it
Two main configurations:
- independent slaves: only one MISO and one MOSI for all the system, but each slave has his CS direct to Master
- Daisy chain: it's like a shift register It only needs one CS line for all:
  . Master MOSI -> FIRST slave MOSI
  . N slave MISO -> N+1 slave MOSI
  . Master MISO <- LAST slave MISO
Note: 
- use prescaler > 8, CPOL = low, CPHA = 1Edge
- LedMatrix: Daisy chain configuration
  GPIOB, GPIO_PIN_6 for the OE, pulse to effectively set the column
- PA5 (SCK), PA6(MISO), PA7(MOSI), PB6 (FOR LEDMATRIX: after transfer of single column complete, SET and then RESET)
- {dataValue, colValue} MSB top-left
- PRESCALER = 8
- timer: quick enought to see continuous light (update all the led matrix in less than 4ms -> frequency > 1250Hz)

## ENCODER: 
TIM3 combined channels=Encoder Mode
- __HAL_TIM_GET_COUNTER(&htim3)
- HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

## Keyboard: 
PC8-11 OUTPUT (column driver), PC12,13,2,3 INPUT (row readout)
row at 0 if button at that row (in the column selected before) is pressed, 1 otherwise.
debounce: register more presses of the button to validate it
- xx 8 9 10 11
- 12
- 13
- 2
- 3

# PROJECTS

## ADC
	1. ADC-polling: ADC start + poll for conversion
	2a. ADC-adcInterrupt: ADC interrupt + software start in while loop
	2b. ADC-interrupt: ADC interrupt + Tim2 trigger out event (in TIM2: Trigger output=Update Event) when conversion ends callback called
	2c. ADC-LCD: ADC interrupt + writing on LCD + drawbar
	3a. ADC-DMA_software: in while(1) HAL_ADC_Start_DMA with buffer of uint16_t. reads potentiometer, temperature of MCU, Vref
	3b. ADC-DMA_timer: reading LDR 1000 a second and sending average every second. Using DMA with length 2000 and reading both in half and full complete

## I2C
	1a. I2C-TemperatureSensor: (temp addr = 0b1001000<<1), reading only the MSB
	1b. I2C-TemperatureSensorFull: selfTest, repBinary, conversion to float, receiving more bytes
	2a. I2C-Accelerometer: (accel addr = 0b0101000<<1), reading three different registers (sending subaddr, receiving data)
	2b. I2C-Accelerometer-Interrupts-UART_DMA: using also UART DMA and a timer to trigger the sampling
	2c. I2C-Accelerometer-I2C_DMA-Interrupts-UART_DMA: using also I2C DMA and autoincrement feature
	
## SPI
	1a. SPI-LedMatrix: use SPI in order to write a letter on led matrix
	1b. SPI-DMA-LedMatrix: two or more letters changing
	
## Encoder
	1a. Encoder: overflow and underflow
	1b. Encoder-DMA:

## Keyboard
	1a. keyboard: polling read of keyboard
	1b. keyboard-DMA: usage of timer interrupt
	
## IR
    1. IR-Receiver: infrared communication receiver. receives data in UART protocol (from UART1) modulated at 38kHz and sends every byte received to the default serial port (UART2).
    2. IR-Transceiver: both ends of the infrared communication, the transmitting part (that implements literally the uart protocol with the modulation of 38kHz) and the receiving part (that implements the already present receiving part)
    3. IR-Transceiver-Accelerometer: both ends of the infrared communication, transmitting the three accelerations on the three axis as values (not as formatted strings) and printing them on the LCD. usage of libraries

## Exams
    1. Exam22-12: scanning the LDR 1000 times and return the average. Stopping the conversion when pressed a corner keybutton of the keyboard
    2. exam-24-02: both ends of the infrared communication, transmitting the temperature of the I2C sensor (not as formatted strings) and printing it on the LCD