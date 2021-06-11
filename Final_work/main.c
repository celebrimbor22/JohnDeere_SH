/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t boolean_T;

typedef struct {
    float  Gain;                         /* '<S1>/Gain' */
    float  Vin;                          /* '<S1>/Divide' */
    float Subtract;                     /* '<S3>/Subtract' */
    uint16_t adcSat;                     /* '<Root>/Saturation' */
    uint16_t currentHigh;                /* '<S2>/Cast' */
    uint16_t Cast1;                      /* '<S2>/Cast1' */
    uint16_t courrentLow;                /* '<S2>/Gain' */
    boolean_T Compare;                   /* '<S4>/Compare' */
    boolean_T Compare_m;                 /* '<S5>/Compare' */
} B_adc2current_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
    uint16_t adcOut;                     /* '<Root>/Input' */
} ExtU_adc2current_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
    float  Iout;                         /* '<Root>/Iout' */
    uint16_t CurrentState;              /* '<Root>/CurrentState' */
} ExtY_adc2current_T;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ID_EXT 		0x18FEEE97 //ID HOT-END ID ident 97
#define RX_ID_EXT  	0x04F03700
#define DTC_ID_EXT 	0x18FECA97 //ID DM1

//Constantes de Formula Steinhart-Hart
#define c1  2.114990448e-03
#define c2 0.3832381228e-04
#define c3 5.228061052e-07
#define ADC_RESOLUTION 4096.0 //ADC resolution value

//Consta voltajeRead
#define ADCResolution 4096.0F
#define Vref 3.3F        //voltage ref
#define MAX_VOL 2.19395 //max voltage 2.19395 2.126448 13
#define MIN_VOL 1.90705 //min voltage 1.907 11.3 1.923929 11.4
#define OFF_SET 40       //OFF SET of data in SPN 110 and SPN 5825

//ADC CONSTANTS
#define MAX_ADC_CONV 16 //ADC DMA buffer size
#define ADC_TEMP 12     //ADC buffer size for temp
#define MAX_CURRENT  3.3F //MAX CURRENT CIRCUIT

#define EMA_a 0.5 //Low pass filter alpha
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart3;

osThreadId adcReadHandle;
osThreadId hotEndHandle;
osThreadId CAN_sendDiagnosHandle;
osThreadId efuseHandle;
osMutexId mutexCANHandle;
/* USER CODE BEGIN PV */


CAN_RxHeaderTypeDef rxHeader; //CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txDTC; //CAN Bus Transmit DTC Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t canTxTemp[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; //CAN Bus Transmit Temp Buffer
uint8_t canTxDTC[8]  = {0xFF,0xFF,0,0,0,0,0xFF,0xFF}; //CAN Bus Transmit DTC Buffer no errors
//							      SPN  0, FMI,CC
uint8_t canTxDM1[8]  = {0xFF,0xFF,0x6E,0 ,0 , 0,0xFF,0xFF}; //CAN Bus Transmit DM1 Buffer report
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

//uint32_t adcBuffer [MAX_ADC_CONV] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t adcBuffer [MAX_ADC_CONV] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t raw[ADC_TEMP] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t datUart[] = "Hello \r\n";

uint8_t errorEst = 0; //error flag
uint16_t temp=0; //global temp

uint32_t delayFuse = 10;

//Temperatur values
int Vo;
float R1 = 100000;
float logR2, R2, T;
float Tc; //Temp in celsius

uint8_t setPoint =0;

//diagnostics variables
uint8_t data, dataStatus; //temp status


uint8_t volState =0; //voltage state

uint8_t eFuseState=0; //software fuse state 0 is ok, 1 blaw
//end diagnostics variables

enum FMI_No{
	FMI0, //Data Valid but Above Normal Operational Range
	FMI1, //Data Valid but Below Normal Operational Range
	FMI2,
	FMI3,
	FMI4,
	FMI5,
	FMI6
}; //FMI No.

uint8_t countError[8] = {0,0,0,0,0,0,0,0};
uint16_t countTimH=0; //occurrence count

/* Block signals (default storage) */
B_adc2current_T adc2current_B;

/* External inputs (root inport signals with default storage) */
ExtU_adc2current_T adc2current_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_adc2current_T adc2current_Y;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
void StartadcRead(void const * argument);
void StarthotEnd(void const * argument);
void StartCAN_sendDiagnostics(void const * argument);
void Startefuse(void const * argument);

/* USER CODE BEGIN PFP */
uint32_t Linear_Interpolation(float x0,float y0,float x1, float y1, float current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  	canfil.FilterBank = 0;
  	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh = 0;
    canfil.FilterIdLow = 0;
    canfil.FilterMaskIdHigh = 0;
    canfil.FilterMaskIdLow = 0;
    canfil.FilterActivation = ENABLE;
    canfil.SlaveStartFilterBank = 14;

    txHeader.DLC = 8;
    txHeader.IDE = CAN_ID_EXT;//Select the ext version
    txHeader.RTR = CAN_RTR_DATA;
    //txHeader.StdId = 0x030;
    txHeader.ExtId = ID_EXT;//ID extendend
    txHeader.TransmitGlobalTime = DISABLE;

    //DTC configuration
    txDTC.DLC = 8;
    txDTC.IDE = CAN_ID_EXT; //Select the ext version
    txDTC.RTR = CAN_RTR_DATA;
     //txHeader.StdId = 0x030;
    txDTC.ExtId = DTC_ID_EXT;//ID extendend
    txDTC.TransmitGlobalTime = DISABLE;

    HAL_CAN_ConfigFilter(&hcan,&canfil);//Config CAN bus
    HAL_CAN_Start(&hcan);//star CAN bus


    //ADC config.
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, MAX_ADC_CONV);

    //Interrup
    //HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//callback

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mutexCAN */
  osMutexDef(mutexCAN);
  mutexCANHandle = osMutexCreate(osMutex(mutexCAN));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of adcRead */
  osThreadDef(adcRead, StartadcRead, osPriorityNormal, 0, 128);
  adcReadHandle = osThreadCreate(osThread(adcRead), NULL);

  /* definition and creation of hotEnd */
  osThreadDef(hotEnd, StarthotEnd, osPriorityHigh, 0, 128);
  hotEndHandle = osThreadCreate(osThread(hotEnd), NULL);

  /* definition and creation of CAN_sendDiagnos */
  osThreadDef(CAN_sendDiagnos, StartCAN_sendDiagnostics, osPriorityNormal, 0, 128);
  CAN_sendDiagnosHandle = osThreadCreate(osThread(CAN_sendDiagnos), NULL);

  /* definition and creation of efuse */
  osThreadDef(efuse, Startefuse, osPriorityNormal, 0, 128);
  efuseHandle = osThreadCreate(osThread(efuse), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ADC_LED_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC_LED_Pin PC15 */
  GPIO_InitStruct.Pin = ADC_LED_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : hotend_out_Pin */
  GPIO_InitStruct.Pin = hotend_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hotend_out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Function implementing the Callback RX CAN.
  * @param  argument: Not used
  * @retval Global variable the CAN Rx Buffer
  */
/* USER CODE END Header_StartadcRead */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	//static uint8_t txMessage[8] = { 0x8C, 0, 0, 0, 0, 0, 0, 0 };
//	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);
//	if(rxHeader.ExtId == RX_ID_EXT){
		// HAL_CAN_AddTxMessage(&hcan, &txHeader, canTxTemp, &canMailbox);
//	}
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	//osMutexWait(mutexCANHandle, 20);
	//HAL_GPIO_TogglePin(GPIOC, ADC_LED_Pin);

	//osMutexRelease(mutexCANHandle);

}

void ADC2Celsius(uint16_t ADC) {
    //funcion de par obtner temperatura a base de lectura ADC
    if (ADC <= 180) {
        Vo = 50;
    }
    else {
        Vo = ADC;
    }
    R2 = R1 * (ADC_RESOLUTION / (float)Vo - 1.0);
    logR2 = log(R2);

    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;


    if (Tc <= -40 || Tc >= 240) {
        data = 1;// dato invalido
        dataStatus = 3;//invalido
    }
    else {
        if (Tc > 0 && Tc < 100) {
            data = 0;//dato valido
            dataStatus = 0;//Normal
        }
        else if (Tc <= 0) {
            data = 0;
            dataStatus = 2;//Low
        }
        else if (Tc >= 100) {
            data = 0;
            dataStatus = 1;//High
        }
    }
}

void adc2current_step()
{
  uint16_t u0;

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/Input'
   */
  u0 = adc2current_U.adcOut;
  if (u0 > 4095) {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = 4095U;
  } else if (u0 < 1985) {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = 1985U;
  } else {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = u0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Gain: '<S1>/Gain' */
  adc2current_B.Gain = 3.3 * (float)adc2current_B.adcSat;

  /* Product: '<S1>/Divide' incorporates:
   *  Constant: '<Root>/Constant'
   */
  adc2current_B.Vin = (float)adc2current_B.Gain / 4096.0;

  /* Sum: '<S3>/Subtract' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  adc2current_B.Subtract = adc2current_B.Vin - 2.5;

  /* Outport: '<Root>/Iout' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Product: '<S3>/Divide1'
   */
  adc2current_Y.Iout = adc2current_B.Subtract / 0.18;

  /* RelationalOperator: '<S4>/Compare' incorporates:
   *  Constant: '<S4>/Constant'
   *  Outport: '<Root>/Iout'
   */
  adc2current_B.Compare = (adc2current_Y.Iout >= MAX_CURRENT);

  /* DataTypeConversion: '<S2>/Cast' */
  adc2current_B.currentHigh = adc2current_B.Compare;

  /* RelationalOperator: '<S5>/Compare' incorporates:
   *  Constant: '<S5>/Constant'
   *  Outport: '<Root>/Iout'
   */
  adc2current_B.Compare_m = (adc2current_Y.Iout <= 0.1);

  /* DataTypeConversion: '<S2>/Cast1' */
  adc2current_B.Cast1 = adc2current_B.Compare_m;

  /* Gain: '<S2>/Gain' */
  adc2current_B.courrentLow = (uint16_t)(adc2current_B.Cast1 << 1);

  /* Outport: '<Root>/CurrentState' incorporates:
   *  Sum: '<S2>/Add'
   */
  adc2current_Y.CurrentState = (uint16_t)((uint32_t)adc2current_B.currentHigh +adc2current_B.courrentLow);

 // return 0;
}

void DiagnosticCheck(){
	//para esta parte todos tienen que tener su estado calculado
		  //Diagnostics
		  if(dataStatus != 0 || volState !=0 || adc2current_Y.CurrentState!=0 || eFuseState){
		      //hay error tenemos que cheecar
		      if(dataStatus == 1){
		  		  //temp >= 100
		  		  countTimH++;
		  		  if(countTimH>=7){
		  			  //Han pasaso 7s tem >=7
		  			  //prepare the DM1
		  			  countError[FMI0]++;
		  			  errorEst =1; //dar de alta la banderea de error
		  			  canTxDM1[4] =   FMI0;
		  			  canTxDM1[5] =   countError[FMI0];
		  		  }
		  	  }else if(dataStatus == 2){
		  		  //low temp
		  		  errorEst =1;
		  		  countError[FMI1]++;
		  		  // DM1 FM1
		  		  canTxDM1[4] =   FMI1;
		  		  canTxDM1[5] =   countError[FMI1];
		  	  }else if(dataStatus ==3){
		  		  //error data incorrect
		  		  errorEst =1;
		  		  countError[FMI2]++;
		  		  // DM1 FM2
		  		  canTxDM1[4] =   FMI2;
		  		  canTxDM1[5] =   countError[FMI2];
		  	  }else if(volState == 1){
		  		  //High voltage
		  		  errorEst =1;
		  		  countError[FMI3]++;
		  		  // DM1 FM3
		  		  canTxDM1[4] =   FMI3;
		  		  canTxDM1[5] =   countError[FMI3];
		  	  }else if(volState == 2){
		  		  //Low voltage
		  		  errorEst =1;
		  		  countError[FMI4]++;
		  		  // DM1 FM4
		  		  canTxDM1[4] =   FMI4;
		  		  canTxDM1[5] =   countError[FMI4];
		  	  }else if(adc2current_Y.CurrentState == 2){
		  		  //Low Current
		  		  errorEst =1;
		  		  countError[FMI5]++;
		  		  canTxDM1[4] =   FMI5;
		  		  canTxDM1[5] =   countError[FMI5];
		  	  }else if(!eFuseState){
		  		  //the fuse blown graounded circuit
		  		  errorEst =1;
		  		countError[FMI6]++;
		  		canTxDM1[4] =   FMI6;
		  		canTxDM1[5] =   countError[FMI6];
		  	  }
		  }else{
		      //no hay erroes
			  for(uint8_t i=0;i<8;i++){
				  countError[i]=0;
			  }
		      countTimH =0;
		      errorEst  =0;
		  }

		  //send cand msg
		  if(errorEst){
			  //send DM1 and temp == FF

			  HAL_CAN_AddTxMessage(&hcan, &txDTC, canTxDM1, &canMailbox);

			  osDelay(100);

			  canTxTemp[0] = 0xFF;
			  HAL_CAN_AddTxMessage(&hcan, &txHeader, canTxTemp, &canMailbox);

		  }else{
			  //send DTC zeros and temp
			  //HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			  HAL_CAN_AddTxMessage(&hcan, &txDTC, canTxDTC, &canMailbox);
			  osDelay(15);
			  //HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			  osDelay(85);
			  //HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			  canTxTemp[0] = temp;
			  HAL_CAN_AddTxMessage(&hcan, &txHeader, canTxTemp, &canMailbox);
			 // HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		  }
}

uint8_t voltageErrorDet(uint16_t ADCvalue){
    uint8_t state = 0;
    float voltage =0.0F;

    if(ADCvalue >= ADCResolution){
        voltage = Vref;
    }else{
        voltage = ((float)(ADCvalue * Vref) )/ ADCResolution;
    }

    if(voltage >= MAX_VOL){
        state = 1;
    }else if(voltage <= MIN_VOL){
        state = 2;
    }else{
        state = 0;
    }

    return state;
}
uint32_t Linear_Interpolation(float x0,float y0,float x1, float y1, float current){
	uint32_t yp;
	yp = (uint32_t)((y0 + ((y1-y0)/(x1-x0)) * (current - x0))*100);
	return yp;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartadcRead */
/**
  * @brief  Function implementing the adcRead thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartadcRead */
void StartadcRead(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//uint16_t raw[2] ={1950,0};
	uint16_t prom =0;
	float EMA_S = 1.0F;
	float EMA_ant =0.0F;
	//uint16_t adcVal[8];
  /* Infinite loop */
  for(;;)
  {
	  if(!eFuseState){
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
		  for(uint8_t i=0; i<ADC_TEMP;i++){
			  prom += adcBuffer[i];
		  }
		  prom =prom/ADC_TEMP;
		  EMA_S = (EMA_a * prom) + ((1-EMA_a)*EMA_ant);//Low pass filter
		  EMA_ant = EMA_S;
		  ADC2Celsius((uint16_t)EMA_S);//Temp calculation
		  temp = (uint16_t)Tc + 40; //temp tiene la temp a enviar

		  if(setPoint >= (temp-OFF_SET)){
			  HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_RESET);
		  }else{
			  HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_SET);
		  }
	  }

    osDelay(200);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StarthotEnd */
/**
* @brief Function implementing the hotEnd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StarthotEnd */
void StarthotEnd(void const * argument)
{
  /* USER CODE BEGIN StarthotEnd */
	//float auxTemp;
	uint32_t canNum=0;
	uint8_t i;
  /* Infinite loop */
  for(;;)
  {
	  canNum = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
	  	  if(canNum>=1){
	  		  for(i=0; i<canNum;i++){
	  			  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, canRX);
	  			  if(rxHeader.ExtId >= RX_ID_EXT && rxHeader.ExtId <= (RX_ID_EXT+0xFF)){
	  			  		//ya tenemos la informacion
	  				  setPoint = canRX[0] - OFF_SET;
	  				  HAL_GPIO_TogglePin(GPIOC, ADC_LED_Pin);

	  			  }
	  		  }
	  	  }
	  	  //HAL_GPIO_TogglePin(GPIOC, ADC_LED_Pin);
	      osDelay(20);
	    }

  /* USER CODE END StarthotEnd */
}

/* USER CODE BEGIN Header_StartCAN_sendDiagnostics */
/**
* @brief Function implementing the CAN_sendDiagnostic and temp msg thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_sendDiagnostics */
void StartCAN_sendDiagnostics(void const * argument)
{
  /* USER CODE BEGIN StartCAN_sendDiagnostics */

	adc2current_Y.CurrentState=0;
	uint16_t volProm =0; //Average voltage read
  /* Infinite loop */
  for(;;)
  {
	  //calcular voltaje
	 volProm = (adcBuffer[12]+adcBuffer[13])/2;
	 volState = voltageErrorDet(volProm);  //voltage state
	 DiagnosticCheck();
    osDelay(900);
  }
  /* USER CODE END StartCAN_sendDiagnostics */
}

/* USER CODE BEGIN Header_Startefuse */
/**
* @brief Function implementing the efuse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startefuse */
void Startefuse(void const * argument)
{
  /* USER CODE BEGIN Startefuse */
	uint16_t currentRaw [2];
	float currentP; //corriente en porcentaje
	uint32_t globalTime = 0;
	uint32_t auxTime; //time to blow
	uint8_t flag=0;
	uint32_t count2blow=0;
	//uint32_t aux=0;
  /* Infinite loop */
  for(;;)
  {

	  if(!eFuseState){
		  currentRaw[0] = adcBuffer[MAX_ADC_CONV-2];
		  currentRaw[1] = adcBuffer[MAX_ADC_CONV-1];
		  adc2current_U.adcOut =  (currentRaw[0]+currentRaw[1])/2;
		  adc2current_step();
		  //The fuse is ok
		  currentP = (adc2current_Y.Iout*100)/MAX_CURRENT;
//		  if(currentP >= 4.5){
//			  //protect the power supplay
//			  eFuseState =1;
//			  //we need to switch on to turn off the system
//			  HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_SET);
//		  }else if(currentP>=100 && currentP <110 ){
//			 auxTime = Linear_Interpolation(100, 2000, 110, 200, currentP);
//		  }
		  if(currentP >= 100){

			 if(currentP >= 135){
				 //protect the power supplay
				 eFuseState =1;
				 //we need to switch on to turn off the system
				 HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_SET);
			 }else if(currentP>=100 && currentP <110 ){
				 auxTime = Linear_Interpolation(100, 2000, 110, 200, currentP);
			 }else if(currentP>=110 && currentP <135 ){
				 auxTime = Linear_Interpolation(100, 2000, 110, 200, currentP);
			 }
			 if(!flag){
				 globalTime = auxTime;
				 flag =1;
			 }else{
				 if(globalTime>auxTime){
					 globalTime = globalTime - auxTime;
				 }
			 }
			 count2blow++;
			 //to blow
			 if(count2blow >= globalTime){
				 eFuseState =1;
				 //we need to switch on to turn off the system
				 HAL_GPIO_WritePin(hotend_out_GPIO_Port, hotend_out_Pin, GPIO_PIN_SET);
			 }

		  }
	  }
    osDelay(delayFuse);

  }
  /* USER CODE END Startefuse */
}


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
