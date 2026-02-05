/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "dht11.h"
#include "i2c_LCD.h"
#include "time_delay.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_RX_Task02 */
osThreadId_t UART_RX_Task02Handle;
const osThreadAttr_t UART_RX_Task02_attributes = {
  .name = "UART_RX_Task02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Warning_Task03 */
osThreadId_t Warning_Task03Handle;
const osThreadAttr_t Warning_Task03_attributes = {
  .name = "Warning_Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for UART_TX_Task04 */
osThreadId_t UART_TX_Task04Handle;
const osThreadAttr_t UART_TX_Task04_attributes = {
  .name = "UART_TX_Task04",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for LCD_Task05 */
osThreadId_t LCD_Task05Handle;
const osThreadAttr_t LCD_Task05_attributes = {
  .name = "LCD_Task05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for DHT11_Task06 */
osThreadId_t DHT11_Task06Handle;
const osThreadAttr_t DHT11_Task06_attributes = {
  .name = "DHT11_Task06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for sensor_Queue01 */
osMessageQueueId_t sensor_Queue01Handle;
const osMessageQueueAttr_t sensor_Queue01_attributes = {
  .name = "sensor_Queue01"
};
/* Definitions for rxUart_Queue02 */
osMessageQueueId_t rxUart_Queue02Handle;
const osMessageQueueAttr_t rxUart_Queue02_attributes = {
  .name = "rxUart_Queue02"
};
/* Definitions for uart_tx_Mutex01 */
osMutexId_t uart_tx_Mutex01Handle;
const osMutexAttr_t uart_tx_Mutex01_attributes = {
  .name = "uart_tx_Mutex01"
};
/* Definitions for rxUart_BinarySem01 */
osSemaphoreId_t rxUart_BinarySem01Handle;
const osSemaphoreAttr_t rxUart_BinarySem01_attributes = {
  .name = "rxUart_BinarySem01"
};
/* USER CODE BEGIN PV */
volatile uint8_t g_temp = 0;
volatile uint8_t g_hum  = 0;

volatile uint8_t th_temp = 20;
volatile uint8_t th_hum  = 60;

volatile uint32_t T1 = 2000; // DHT11
volatile uint32_t T2 = 500;  // Warning
volatile uint32_t T3 = 2000; // LCD
volatile uint32_t T4 = 1000; // UART TX

uint8_t rx_byte;
char rx_buf[64];
volatile uint8_t rx_idx = 0;
// Driver handles
LCD1602_I2C_t lcd;
DHT11_HandleTypeDef hdht11;
DHT11_Data_t dht_out;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void rxStartTask02(void *argument);
void waStartTask03(void *argument);
void txStartTask04(void *argument);
void lcdStartTask05(void *argument);
void dht11StartTask06(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf(const char *fmt, ...)
{
  char buf[128];
  va_list args;

  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len <= 0) return;
  if (len > sizeof(buf)) len = sizeof(buf);

  osMutexAcquire(uart_tx_Mutex01Handle, osWaitForever);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
  osMutexRelease(uart_tx_Mutex01Handle);
}

/* USER CODE BEGIN 0 */
static void Exec_DHT11(void)
{
  DHT11_Data_t d;
  int err = DHT11_Read(&hdht11, &d);
  if (err == 0)
  {
    g_temp = d.temperature_int;
    g_hum  = d.humidity_int;
  }
  // n?u l?i: gi? giá tr? cu (không gán)
}

//static void Exec_WarningLED(void)
//{
//  if (g_temp > th_temp || g_hum > th_hum)
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ON (active-low)
//  else
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // OFF
//}
static void Exec_WarningLED(void)
{
  static uint32_t last_toggle = 0;
  const uint32_t blink_ms = 50; // t?c d? nháy khi c?nh báo

  if (g_temp > th_temp || g_hum > th_hum)
  {
    uint32_t now = osKernelGetTickCount();
    if ((now - last_toggle) >= blink_ms)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // nháy
      last_toggle = now;
    }
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // OFF (PC13 active-low)
    last_toggle = osKernelGetTickCount(); // reset nh?p nháy
  }
}


static void Exec_LCD(void)
{
  char l1[17], l2[17];
  snprintf(l1, sizeof(l1), "Tem:%2uC Hum:%2u%%", g_temp, g_hum);
  snprintf(l2, sizeof(l2), "SET:%2uC %2u%% ", th_temp, th_hum);

  LCD1602_SetCursor(&lcd, 0, 0);
  LCD1602_Print(&lcd, "                ");
  LCD1602_SetCursor(&lcd, 0, 0);
  LCD1602_Print(&lcd, l1);

  LCD1602_SetCursor(&lcd, 1, 0);
  LCD1602_Print(&lcd, "                ");
  LCD1602_SetCursor(&lcd, 1, 0);
  LCD1602_Print(&lcd, l2);
}

static void Exec_UARTTX(void)
{
  myPrintf("[TX] tick=%lu T=%u H=%u | P=%lu,%lu,%lu,%lu | SET: %u,%u\r\n",
           osKernelGetTickCount(), g_temp, g_hum,
           T1, T2, T3, T4, th_temp, th_hum);
}

static void Exec_UARTRX_Parse(void)
{
  uint32_t p1,p2,p3,p4,tt,hh;
  if (sscanf(rx_buf, "SET P=%lu,%lu,%lu,%lu TEMP=%lu HUM=%lu",
             &p1,&p2,&p3,&p4,&tt,&hh) == 6)
  {
    osKernelLock();
    T1 = p1; T2 = p2; T3 = p3; T4 = p4;
    th_temp = (uint8_t)tt;
    th_hum  = (uint8_t)hh;
    osKernelUnlock();

    myPrintf("[RX] OK P=%lu,%lu,%lu,%lu TH=%u,%u\r\n",
             T1,T2,T3,T4,th_temp,th_hum);
  }
  else
  {
    myPrintf("[RX] BAD CMD: %s\r\n", rx_buf);
    myPrintf("[RX] EX: SET P=2000,500,1000,1000 TEMP=30 HUM=60.\r\n");
  }
}
/* USER CODE END 0 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  TIM2_Delay_Init(&htim2);

  // init dht11 (PA5 + TIM2)
  hdht11.GPIOx = GPIOA;
  hdht11.GPIO_Pin = GPIO_PIN_5;
  hdht11.htim = &htim2;
  DHT11_Init(&hdht11);

  // LCD address: 0x27 ho?c 0x3F
  LCD1602_Init(&lcd, &hi2c1, 0x27);
  LCD1602_Clear(&lcd);
  LCD1602_SetCursor(&lcd, 0, 0);
  LCD1602_Print(&lcd, "DHT11 + LCD");
  HAL_Delay(1000);
  LCD1602_Clear(&lcd);

  // set default global

//  T1 = 2000; // DHT11
//  T2 = 500;  // Warning
//  T3 = 1000; // LCD
//  T4 = 1000; // UART TX
//
//  th_temp = 20;
//  th_hum  = 70;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF (PC13 active-low)

  // start UART RX interrupt (1 byte)
 HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//  HAL_UART_Transmit(&huart1, (uint8_t*)"BOOT\r\n", 6, 100);
//  HAL_Delay(200);
//  HAL_UART_Transmit(&huart1, (uint8_t*)"BOOT2\r\n", 7, 100);

  /* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uart_tx_Mutex01 */
  uart_tx_Mutex01Handle = osMutexNew(&uart_tx_Mutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of rxUart_BinarySem01 */
//  rxUart_BinarySem01Handle = osSemaphoreNew(1, 1, &rxUart_BinarySem01_attributes);
  rxUart_BinarySem01Handle = osSemaphoreNew(1, 0, &rxUart_BinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensor_Queue01 */
  sensor_Queue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &sensor_Queue01_attributes);

  /* creation of rxUart_Queue02 */
  rxUart_Queue02Handle = osMessageQueueNew (16, sizeof(uint16_t), &rxUart_Queue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_RX_Task02 */
  UART_RX_Task02Handle = osThreadNew(rxStartTask02, NULL, &UART_RX_Task02_attributes);

  /* creation of Warning_Task03 */
  Warning_Task03Handle = osThreadNew(waStartTask03, NULL, &Warning_Task03_attributes);

  /* creation of UART_TX_Task04 */
  UART_TX_Task04Handle = osThreadNew(txStartTask04, NULL, &UART_TX_Task04_attributes);

  /* creation of LCD_Task05 */
  LCD_Task05Handle = osThreadNew(lcdStartTask05, NULL, &LCD_Task05_attributes);

  /* creation of DHT11_Task06 */
  DHT11_Task06Handle = osThreadNew(dht11StartTask06, NULL, &DHT11_Task06_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_idx < sizeof(rx_buf)-1)
      rx_buf[rx_idx++] = (char)rx_byte;

    if (rx_byte == '.')
    {
      rx_buf[rx_idx] = 0;
      rx_idx = 0;

      if (osKernelGetState() == osKernelRunning && rxUart_BinarySem01Handle != NULL)
        osSemaphoreRelease(rxUart_BinarySem01Handle);
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
//	  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	  for(;;) osDelay(10000);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_rxStartTask02 */
/* USER CODE END Header_rxStartTask02 */
void rxStartTask02(void *argument)
{
  /* USER CODE BEGIN rxStartTask02 */
//	for(;;)
//	  {
//	    osSemaphoreAcquire(rxUart_BinarySem01Handle, osWaitForever);
//
//	    uint32_t p1,p2,p3,p4,tt,hh;
//
//	    if (sscanf(rx_buf,"SET P=%lu,%lu,%lu,%lu TEMP=%lu HUM=%lu",
//	               &p1,&p2,&p3,&p4,&tt,&hh)==6)
//	    {
//	      T1=p1; T2=p2; T3=p3; T4=p4;
//	      th_temp=tt; th_hum=hh;
//
//	      myPrintf("[RX] OK P=%lu,%lu,%lu,%lu T=%u H=%u\r\n",
//	               T1,T2,T3,T4,th_temp,th_hum);
//	    }
//	  }
  /* USER CODE END rxStartTask02 */

	  /* USER CODE BEGIN rxStartTask02 */
	  for(;;)
	  {
	    osSemaphoreAcquire(rxUart_BinarySem01Handle, osWaitForever);
	    Exec_UARTRX_Parse();
	  }
	  /* USER CODE END rxStartTask02 */


}

/* USER CODE BEGIN Header_waStartTask03 */
/* USER CODE END Header_waStartTask03 */
void waStartTask03(void *argument)
{
  /* USER CODE BEGIN waStartTask03 */

//	  for(;;)
//	  {
//	    myPrintf("[WARN] START %lu\r\n", osKernelGetTickCount());
//
//	    if(g_temp>th_temp || g_hum>th_hum)
//	      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//	    else
//	      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
//
//	    myPrintf("[WARN] END %lu\r\n", osKernelGetTickCount());
//	    osDelay(T2);
//	  }

	  /* USER CODE BEGIN waStartTask03 */
	  uint32_t last = osKernelGetTickCount();
	  for(;;)
	  {
		  myPrintf("[%lu] RUN warring\r\n",last);
	    last += T2;
	    Exec_WarningLED();
	   osDelayUntil(last);
	    //osDelay(last);
	  }
	  /* USER CODE END waStartTask03 */




  /* USER CODE END waStartTask03 */
}

/* USER CODE BEGIN Header_txStartTask04 */
/* USER CODE END Header_txStartTask04 */
void txStartTask04(void *argument)
{
  /* USER CODE BEGIN txStartTask04 */

//	  for(;;)
//	  {
//	    myPrintf("[TX] tick=%lu T=%d H=%d\r\n",
//	             osKernelGetTickCount(), g_temp, g_hum);
//	    osDelay(T4);
//	  }

	  /* USER CODE BEGIN txStartTask04 */
	//  HAL_UART_Transmit(&huart1, (uint8_t*)"HELLO\r\n", 7, 100);
	  uint32_t last = osKernelGetTickCount();
	  for(;;)
	  {
	    last += T4;
	    Exec_UARTTX();
	    osDelayUntil(last);
	   // osDelay(last);
	  }
	  /* USER CODE END txStartTask04 */



  /* USER CODE END txStartTask04 */
}

/* USER CODE BEGIN Header_lcdStartTask05 */
/* USER CODE END Header_lcdStartTask05 */
void lcdStartTask05(void *argument)
{
  /* USER CODE BEGIN lcdStartTask05 */

//	  for(;;)
//	  {
//	    myPrintf("[LCD] tick=%lu\r\n", osKernelGetTickCount());
//	    osDelay(T3);
//	  }

	  /* USER CODE BEGIN lcdStartTask05 */
	  uint32_t last = osKernelGetTickCount();
	  for(;;)
	  {
	    last += T3;
	    Exec_LCD();
	    osDelayUntil(last);
	   // osDelay(last);
	  }
	  /* USER CODE END lcdStartTask05 */




  /* USER CODE END lcdStartTask05 */
}

/* USER CODE BEGIN Header_dht11StartTask06 */
/* USER CODE END Header_dht11StartTask06 */
void dht11StartTask06(void *argument)
{
  /* USER CODE BEGIN dht11StartTask06 */

//	  for(;;)
//	  {
//	    myPrintf("[DHT11] START %lu\r\n", osKernelGetTickCount());
//
//	    // DHT11_Read(...) -> g_temp, g_hum
//
//	    myPrintf("[DHT11] END %lu\r\n", osKernelGetTickCount());
//	    osDelay(T1);
//	  }

	  /* USER CODE BEGIN dht11StartTask06 */
	  uint32_t last = osKernelGetTickCount();
	  for(;;)
	  {
	    last += T1;
	    Exec_DHT11();
	    osDelayUntil(last);
	 //   osDelay(last);
	  }
	  /* USER CODE END dht11StartTask06 */




  /* USER CODE END dht11StartTask06 */
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
  if (htim->Instance == TIM4)
  {
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
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
