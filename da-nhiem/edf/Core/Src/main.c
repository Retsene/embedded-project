/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // Cho sprintf
#include <string.h> // Cho strlen
#include "CLCD_I2C.h"
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

/* Definitions for edfManager */
osThreadId_t edfManagerHandle;
const osThreadAttr_t edfManager_attributes = {
  .name = "edfManager",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for taskDht11 */
osThreadId_t taskDht11Handle;
const osThreadAttr_t taskDht11_attributes = {
  .name = "taskDht11",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskWarning */
osThreadId_t taskWarningHandle;
const osThreadAttr_t taskWarning_attributes = {
  .name = "taskWarning",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskRxUart */
osThreadId_t taskRxUartHandle;
const osThreadAttr_t taskRxUart_attributes = {
  .name = "taskRxUart",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskTxUart */
osThreadId_t taskTxUartHandle;
const osThreadAttr_t taskTxUart_attributes = {
  .name = "taskTxUart",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskLcd */
osThreadId_t taskLcdHandle;
const osThreadAttr_t taskLcd_attributes = {
  .name = "taskLcd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};

osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};


/* USER CODE BEGIN PV */
CLCD_I2C_Name LCD1;

typedef struct {
  uint8_t temp;
  uint8_t hum;
  uint8_t limit_temp;
  uint8_t limit_hum;
  uint8_t is_warning;
} SystemData_t;

typedef struct {
  osThreadId_t *handle_ptr; // Con trỏ trỏ đến Handle của Task
  uint32_t period;          // Chu kỳ (ms)
  uint32_t absolute_dl;     // Deadline tuyệt đối (thời điểm phải xong)
} EDF_Task_Block;

EDF_Task_Block taskList[] = {
  { &taskDht11Handle,   1000,  0 },
  { &taskWarningHandle, 1000,  0 },
  { &taskRxUartHandle,  1000,  0 },
  { &taskTxUartHandle,  3000,  0 },
  { &taskLcdHandle,     2000,  0 },
};

#define RX_BUFF_SIZE 100
uint8_t rx_byte;               // Nhận từng ký tự
char rx_buffer[RX_BUFF_SIZE];  // Gom thành chuỗi lệnh
uint8_t rx_indx = 0;           // Vị trí con trỏ buffer
uint8_t cmd_received_flag = 0; // Cờ báo có lệnh mới
int numTasks = 5;
volatile SystemData_t sysData = {0, 0, 35, 95, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartEdfManager(void *argument);
void StartTaskDht11(void *argument);
void StartTaskWarning(void *argument);
void StartTaskRxUart(void *argument);
void StartTaskTxUart(void *argument);
void StartTaskLcd(void *argument);

/* USER CODE BEGIN PFP */
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *hum);
void delay_us(uint16_t us);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of dataMutex */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

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
  /* creation of edfManager */
  edfManagerHandle = osThreadNew(StartEdfManager, NULL, &edfManager_attributes);

  /* creation of taskDht11 */
  taskDht11Handle = osThreadNew(StartTaskDht11, NULL, &taskDht11_attributes);

  /* creation of taskWarning */
  taskWarningHandle = osThreadNew(StartTaskWarning, NULL, &taskWarning_attributes);

  /* creation of taskRxUart */
  taskRxUartHandle = osThreadNew(StartTaskRxUart, NULL, &taskRxUart_attributes);

  /* creation of taskTxUart */
  taskTxUartHandle = osThreadNew(StartTaskTxUart, NULL, &taskTxUart_attributes);

  /* creation of taskLcd */
  taskLcdHandle = osThreadNew(StartTaskLcd, NULL, &taskLcd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset bộ đếm TIM2 về 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Chờ đến khi đủ thời gian
}

static void DHT11_SetDir(uint8_t output)
{

  if (output) {
    GPIOB->CRL &= ~(0xF << 0); // Xóa cấu hình cũ của PB0
    GPIOB->CRL |= (1 << 0); // Output Push-Pull 10MHz
  } else {
    GPIOB->CRL &= ~(0xF << 0); // Xóa cấu hình cũ
    GPIOB->CRL |= (8 << 0); // Input Push-Pull/Floating
  }
}

uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *hum) {
  uint8_t data[5] = {0};
  uint32_t wait_cnt = 0;

  /* 1. Gửi tín hiệu Start */
  DHT11_SetDir(1); // Output
  HAL_GPIO_WritePin(GPIOB, DHT11_Pin, GPIO_PIN_RESET);
  HAL_Delay(20); // Kéo thấp 20ms
  HAL_GPIO_WritePin(GPIOB, DHT11_Pin, GPIO_PIN_SET);
  delay_us(30); // Kéo cao 30us

  /* 2. Chuyển sang Input */
  DHT11_SetDir(0);

  taskENTER_CRITICAL();

  wait_cnt = 0;
  while ((GPIOB->IDR & DHT11_Pin)) {
    wait_cnt++;
    if (wait_cnt > 1000) {
      taskEXIT_CRITICAL();
      return 0;
    }
  }
  // Chờ kéo lên cao (80us)
  wait_cnt = 0;
  while (!(GPIOB->IDR & DHT11_Pin)) {
    wait_cnt++;
    if (wait_cnt > 1000) {
      taskEXIT_CRITICAL();
      return 0;
    }
  }
  // Chờ xuống thấp lại để bắt đầu bit đầu tiên
  wait_cnt = 0;
  while ((GPIOB->IDR & DHT11_Pin)) {
    wait_cnt++;
    if (wait_cnt > 1000) {
      taskEXIT_CRITICAL();
      return 0;
    }
  }

  /* 4. Đọc 40 bit */
  for (int i = 0; i < 40; i++) {
    // Chờ bit bắt đầu (đang thấp -> chờ lên cao)
    wait_cnt = 0;
    while (!(GPIOB->IDR & DHT11_Pin)) {
      wait_cnt++;
      if (wait_cnt > 1000) {
        taskEXIT_CRITICAL();
        return 0;
      }
    }

    // Đo độ rộng mức cao để phân biệt 0/1
    delay_us(40);

    if ((GPIOB->IDR & DHT11_Pin)) {
      // Vẫn cao -> Bit 1
      data[i / 8] |= (1 << (7 - (i % 8)));

      // Chờ hết mức cao
      wait_cnt = 0;
      while ((GPIOB->IDR & DHT11_Pin)) {
        wait_cnt++;
        if (wait_cnt > 1000) {
          taskEXIT_CRITICAL();
          return 0;
        }
      }
    }
  }

  /* 5. MỞ NGẮT */
  taskEXIT_CRITICAL();

  if (data[4] == (data[0] + data[1] + data[2] + data[3]))
  {
    *hum  = data[0]; // Byte 0: Độ ẩm phần nguyên
    *temp = data[2]; // Byte 2: Nhiệt độ phần nguyên

    return 1;
  }
  return 0;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEdfManager */
/**
  * @brief  Function implementing the edfManager thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEdfManager */
void StartEdfManager(void *argument)
{
  /* USER CODE BEGIN 5 */
  osThreadSetPriority(edfManagerHandle, osPriorityRealtime);
  uint32_t now = osKernelGetTickCount();
  for(int i=0; i<numTasks; i++) {
    taskList[i].absolute_dl = now + taskList[i].period;
  }

  for(;;)
  {
    now = osKernelGetTickCount();

    for (int i = 0; i < numTasks; i++)
    {
      if (now >= taskList[i].absolute_dl) {
        taskList[i].absolute_dl += taskList[i].period;
      }
    }

    uint32_t minDeadline = 0xFFFFFFFF;
    int winnerIndex = -1;

    for (int i = 0; i < numTasks; i++) {
      if (taskList[i].absolute_dl < minDeadline) {
        minDeadline = taskList[i].absolute_dl;
        winnerIndex = i;
      }
    }

    for (int i = 0; i < numTasks; i++)
    {
      osThreadId_t targetTask = *(taskList[i].handle_ptr);
      if (i == winnerIndex)
        osThreadSetPriority(targetTask, osPriorityAboveNormal);
      else
        osThreadSetPriority(targetTask, osPriorityLow);
    }

    osDelay(10);

  }  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskDht11 */
/**
* @brief Function implementing the taskDht11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDht11 */
void StartTaskDht11(void *argument)
{
  /* USER CODE BEGIN StartTaskDht11 */
  HAL_TIM_Base_Start(&htim2); // Start Timer cho delay_us

  uint8_t t=0, h=0;
  /* Infinite loop */
  for(;;) {
    // 1. Đọc 4 thông số từ cảm biến
  if (DHT11_Read_Data(&t, &h) == 1)
  {
      // 2. Cập nhật vào Struct chung (Có Mutex)
      if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
      {
        sysData.temp = t;
        sysData.hum  = h;
        osMutexRelease(dataMutexHandle);
      }
    }

    // 3. Nghỉ 1000ms
    osDelay(1000);
  }
  /* USER CODE END StartTaskDht11 */
}

/* USER CODE BEGIN Header_StartTaskWarning */
/**
* @brief Function implementing the taskWarning thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskWarning */
void StartTaskWarning(void *argument)
{
  /* USER CODE BEGIN StartTaskWarning */
  uint8_t t=0, h=0, lt=0, lh=0, w=0;
  for(;;)
  {
    if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
    {
      t  = sysData.temp;
      h  = sysData.hum;
      lt = sysData.limit_temp;
      lh = sysData.limit_hum;
      osMutexRelease(dataMutexHandle);
    }

    // So sánh và bật đèn
    if (t > lt || h > lh) {
      w = 1;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Nháy đèn
    } else {
      w = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Tắt đèn
    }

    // Cập nhật lại cờ warning
    if (osMutexWait(dataMutexHandle, osWaitForever) == osOK) {
      sysData.is_warning = w;
      osMutexRelease(dataMutexHandle);
    }
    osDelay(1000);
  }
  /* USER CODE END StartTaskWarning */
}

/* USER CODE BEGIN Header_StartTaskRxUart */
/**
* @brief Function implementing the taskRxUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRxUart */
void StartTaskRxUart(void *argument)
{
  /* USER CODE BEGIN StartTaskRxUart */
  uint8_t rxChar = 0;
  char rxBuffer[50] = {0}; // Increased size for longer commands
  uint8_t index = 0;

  char msg[64]; // Buffer for sending response back

  /* Infinite loop */
  for(;;)
  {
    // Wait for a character (Blocking mode)
    if (HAL_UART_Receive(&huart1, &rxChar, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      // Check for Newline (Enter key)
      if (rxChar == '\r' || rxChar == '\n')
      {
        if (index > 0)
        {
          rxBuffer[index] = '\0'; // Null-terminate the string so sscanf can read it

          int new_t = 0;
          int new_h = 0;

          if (sscanf(rxBuffer, "Set T=%d H=%d", &new_t, &new_h) == 2)
          {
            // Update Global Variables safely with Mutex
            if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
            {
              sysData.limit_temp = (uint8_t)new_t;
              sysData.limit_hum  = (uint8_t)new_h;
              osMutexRelease(dataMutexHandle);
            }

            // Send confirmation
            sprintf(msg, " Limit Updated: T=%d H=%d\r\n", new_t, new_h);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
          }

          if (sscanf(rxBuffer, "Set ID=%d P=%d", &new_t, &new_h) == 2)
          {
            // Update Global Variables safely with Mutex
            if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
            {
              taskList[new_t].period = (uint32_t)new_h;
              osMutexRelease(dataMutexHandle);
            }

            // Send confirmation
            sprintf(msg, " Task %d Period Update: %d\r\n", new_t, new_h);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
          }


          // Reset buffer for next command
          index = 0;
          memset(rxBuffer, 0, sizeof(rxBuffer));
        }
      }
      else
      {
        // Add character to buffer if space allows
        if (index < sizeof(rxBuffer) - 1)
        {
          rxBuffer[index++] = rxChar;
        }
      }
    }
  }
  osDelay(1000);
}

/* USER CODE BEGIN Header_StartTaskTxUart */
/**
* @brief Function implementing the taskTxUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTxUart */
void StartTaskTxUart(void *argument)
{
  char msg[100]; // Biến đệm để chứa chuỗi log
  uint8_t t=0, h=0, w=0;
  uint8_t lt=0, lh=0;

  for(;;)
  {
    // 1. Lấy dữ liệu từ vùng nhớ chung
    if(osMutexWait(dataMutexHandle, osWaitForever) == osOK)
    {
      t  = sysData.temp;
      h  = sysData.hum;
      w  = sysData.is_warning;
      lt = sysData.limit_temp;
      lh = sysData.limit_hum;
      osMutexRelease(dataMutexHandle);
    }

    // 2. Format chuỗi gửi lên máy tính
    // Mẫu: "Temp: 30 | Hum: 70 | Warn: 0 | Limit: 35-95"
    sprintf(msg, "Temp: %d | Hum: %d | Warn: %d | Limit: %d-%d\r\n", t, h, w, lt, lh);

    // 3. Gửi qua UART
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    // 4. Delay theo chu kỳ được cài đặt trong taskList[3]
    // Mặc định là 3000ms, nhưng có thể thay đổi bằng lệnh
    osDelay(taskList[3].period);

  }
}

/* USER CODE BEGIN Header_StartTaskLcd */
/**
* @brief Function implementing the taskLcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLcd */
void StartTaskLcd(void *argument)
{
  /* USER CODE BEGIN StartTaskLcd */
  /* 1. KHOI TAO LCD */
  // Tham so: &BienLCD, &BienI2C, DiaChi, SoCot, SoHang
  // Dia chi thuong la 0x4E hoac 0x7E. Neu khong len hinh thu doi sang 0x7E
  CLCD_I2C_Init(&LCD1, &hi2c1, 0x4E, 16, 2);

  // Hien thi man hinh chao (Option)
  CLCD_I2C_Clear(&LCD1);

  char lcd_line1[17];
  char lcd_line2[17];
  uint8_t t=0, h=0;
  uint8_t lt=0, lh=0, w=0;  /* Infinite loop */
  for(;;) {
    // 2. LAY DU LIEU (Mutex)
    if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
    {
      t  = sysData.temp;
      h  = sysData.hum;
      lt = sysData.limit_temp;
      lh = sysData.limit_hum;
      w  = sysData.is_warning;
      osMutexRelease(dataMutexHandle);
    }

    // 3. HIEN THI
    // Format: "Temp: 28 Hum: 80"
    // Them khoang trang o cuoi de xoa ky tu thua neu so bi giam (vd 100 -> 99)
    sprintf(lcd_line1, "Temp:%d Hum:%d", t, h);

    // Ghi len Hang 1 (Trong thu vien nay quy dinh: X=Cot, Y=Hang)
    CLCD_I2C_SetCursor(&LCD1, 0, 0);
    CLCD_I2C_WriteString(&LCD1, lcd_line1);

    sprintf(lcd_line2, "T=%d H=%d | W=%d", lt, lh, w);
    CLCD_I2C_SetCursor(&LCD1, 0, 1); // Xuống hàng 2
    CLCD_I2C_WriteString(&LCD1, lcd_line2);

    // 4. CHU KY 2000ms
    osDelay(2000);  /* USER CODE END StartTaskLcd */
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

