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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "i2c_LCD.h"
#include "time_delay.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  // sensor
  uint8_t temp_c;
  uint8_t hum_pct;
  uint8_t valid; // valid=1 DHT11 đo OK, còn valid=0 DHT11 hư

  // config (có thể đổi qua UART)
  uint32_t period_ms;     // chu kì đo + gửi + update LCD
  uint8_t  th_temp;       // ngưỡng nhiệt độ
  uint8_t  th_hum;        // ngưỡng độ ẩm

  // status
  uint8_t alarm;          // 1: Cbao, 0: Bth

  // UART RX line (nhận theo dòng)
  uint8_t  rx_byte;
  char     rx_line[64];
  uint8_t  rx_len;
  volatile uint8_t line_ready; // 1: có 1 dòng lệnh hoàn chỉnh

} AppCtx;

typedef enum {
  EVENT_NONE = 0,
  EVENT_TASK_TEMP,  // Sự kiện đọc DHT11
  EVENT_TASK_LCD,   // Sự kiện cập nhật LCD
  EVENT_TASK_UART,  // Sự kiện gửi báo cáo UART
  EVENT_TASK_CMD    // Sự kiện xử lý lệnh từ UART RX
} EventID_t;

#define QUEUE_SIZE 15
EventID_t event_queue[QUEUE_SIZE];
int8_t head = 0, tail = 0, event_count = 0;

// Hàm đẩy sự kiện vào hàng đợi
void push_event(EventID_t id) {
  if (event_count < QUEUE_SIZE) {
    event_queue[tail] = id;
    tail = (tail + 1) % QUEUE_SIZE;
    event_count++;
  }
}

// Hàm lấy sự kiện ra khỏi hàng đợi
EventID_t pop_event(void) {
  if (event_count == 0) return EVENT_NONE;
  EventID_t id = event_queue[head];
  head = (head + 1) % QUEUE_SIZE;
  event_count--;
  return id;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13
#define LED_ON()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LCD1602_I2C_t lcd;

DHT11_HandleTypeDef hdht11;
static AppCtx app;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//TASK1: đo DHT11 và lưu dữ liệu vào ctx
static void TASK_DHT11_Measure(AppCtx *ctx, DHT11_HandleTypeDef *dht)
{
  DHT11_Data_t raw;
  int err = DHT11_Read(dht, &raw);

  if (err == 0) {
    ctx->temp_c = raw.temperature_int;
    ctx->hum_pct = raw.humidity_int;
    ctx->valid = 1;
  } else {
    ctx->valid = 0;
  }
}

// Task2 kiểm tra ngưỡng + nháy LED 1 lần sau khi kiểm tra
static void TASK_CheckThreshold_BlinkOnce(AppCtx *ctx)
{
  // nếu lỗi cảm biến thì coi như cảnh báo
  if (!ctx->valid) {
    ctx->alarm = 1;
  } else {
	  // nháy 1 lần (non-blocking kiểu ngắn dùng HAL_Delay OK vì chỉ 1 nháy)
    ctx->alarm = (ctx->temp_c >= ctx->th_temp) || (ctx->hum_pct >= ctx->th_hum);
  }
  if(ctx->alarm){
	  LED_ON();
  } else {
	  LED_OFF();
  }

  // nháy 1 lần (non-blocking kiểu ngắn dùng HAL_Delay OK vì chỉ 1 nháy)
  //LED_ON();
  //HAL_Delay(100);
  //LED_OFF();
}

// Task3 LCD: dòng 1 = T/H, dòng 2 = chu kì + TT(Bth/Cbao)
static void TASK_LCD_Update(AppCtx *ctx, LCD1602_I2C_t *lcd)
{
  char l1[17], l2[17];

  if (ctx->valid) {
    snprintf(l1, sizeof(l1), "Tem:%2uC Hum:%2u%%", ctx->temp_c, ctx->hum_pct);
  } else {
    snprintf(l1, sizeof(l1), "Tem:--C Hum:--%%");
  }

  // hiển thị chu kì dạng giây hoặc ms cho gọn
  // ví dụ: "Cyc=2000 TT=Bth"
  snprintf(l2, sizeof(l2), "Trangthai: %s",
           ctx->alarm ? "Cbao" : "Bth");

  LCD1602_SetCursor(lcd, 0, 0);
  LCD1602_Print(lcd, l1);


  LCD1602_SetCursor(lcd, 1, 0);
  LCD1602_Print(lcd, "                ");

  LCD1602_SetCursor(lcd, 1, 0);
  LCD1602_Print(lcd, l2);
}

// Task4 UART TX: gửi nhiệt độ, ẩm, chu kì, ngưỡng, TT
static void TASK_UART_SendStatus(AppCtx *ctx, UART_HandleTypeDef *huart)
{
  char tx[128];
  int n;
 //Lấy thời điểm hiện tại để kiểm tra độ chính xác của lập lịch
  uint32_t current_time = HAL_GetTick();

  if (ctx->valid) {
    n = snprintf(tx, sizeof(tx),
      "[%lu ms] T=%u H=%u Tth=%u Hth=%u Status=%s\r\n",
	  current_time,
      ctx->temp_c, ctx->hum_pct,
      ctx->th_temp, ctx->th_hum,
      ctx->alarm ? "Cbao" : "Bth"
    );
  } else {
    n = snprintf(tx, sizeof(tx),
      "[%lu ms] DHT11_ERROR Tth=%u Hth=%u Status=%s\r\n",
	  current_time,
      ctx->th_temp, ctx->th_hum,
      ctx->alarm ? "Cbao" : "Bth"
    );
  }

  HAL_UART_Transmit(huart, (uint8_t*)tx, (uint16_t)n, 100);
}

// Task5 UART RX: parse line để cập nhật chu kì và ngưỡng
// Nhận các token: P=..., T=..., H=...
static void TASK_UART_ParseLine(AppCtx *ctx, UART_HandleTypeDef *huart)
{
  if (!ctx->line_ready) return;
  ctx->line_ready = 0;

  // copy ra buffer tạm để strtok
  char buf[64];
  strncpy(buf, ctx->rx_line, sizeof(buf));
  buf[sizeof(buf)-1] = 0;

  // tách theo space hoặc comma
  char *tok = strtok(buf, " ,");
  uint8_t updated = 0;

  while (tok) {
    if (tok[0] == 'P' || tok[0] == 'p') {
      uint32_t p = 0;
      if (sscanf(tok, "P=%lu", &p) == 1 || sscanf(tok, "p=%lu", &p) == 1 ) {
    	  __HAL_TIM_SET_AUTORELOAD(&htim1, p - 1);
    	          // Reset counter để áp dụng ngay lập tức
    	          __HAL_TIM_SET_COUNTER(&htim1, 0);
        updated = 1;
      }
    } else if (tok[0] == 'T' || tok[0] == 't') {
      unsigned t = 0;
      if (sscanf(tok, "T=%u", &t) == 1 || sscanf(tok, "t=%u", &t) == 1 ) {
        if (t <= 99) { ctx->th_temp = (uint8_t)t; updated = 1; }
      }
    } else if (tok[0] == 'H' || tok[0] == 'h') {
      unsigned h = 0;
      if (sscanf(tok, "H=%u", &h) == 1 || sscanf(tok, "h=%u", &h) == 1 ) {
        if (h <= 99) { ctx->th_hum = (uint8_t)h; updated = 1; }
      }
    }

    tok = strtok(NULL, " ,");
  }

  // phản hồi lại cấu hình hiện tại
  if (updated) {
    char ack[80];
    int n = snprintf(ack, sizeof(ack), "OK P=%lu Tth=%u Hth=%u\r\n",
                     (unsigned long)ctx->period_ms, ctx->th_temp, ctx->th_hum);
    HAL_UART_Transmit(huart, (uint8_t*)ack, (uint16_t)n, 100);
  } else {
    const char *msg = "FORMAT: P=2000 T=20 H=50\r\n";
    HAL_UART_Transmit(huart, (uint8_t*)msg, (uint16_t)strlen(msg), 100);
  }
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
    TIM2_Delay_Init(&htim2);   // nếu time_delay của bạn dùng TIM2 thì cứ giữ
  // init dht11

    // Khởi chạy ngắt cho các timer tạo sự kiện định kỳ
      HAL_TIM_Base_Start_IT(&htim1); // Cho UART TX
      HAL_TIM_Base_Start_IT(&htim3); // Cho DHT11
      HAL_TIM_Base_Start_IT(&htim4); // Cho LCD

    hdht11.GPIOx = GPIOA;
    hdht11.GPIO_Pin = GPIO_PIN_5;
    hdht11.htim = &htim2;
    DHT11_Init(&hdht11);

    // LCD address: 0x27 hoặc 0x3F
    LCD1602_Init(&lcd, &hi2c1, 0x27);
    LCD1602_Clear(&lcd);
    LCD1602_SetCursor(&lcd, 0, 0);
    LCD1602_Print(&lcd, "DHT11 + LCD");
    HAL_Delay(1000);
    LCD1602_Clear(&lcd);

    // app default
    memset(&app, 0, sizeof(app));
    app.period_ms = 4000;  // chu kì mặc định
    app.th_temp   = 29;    // ngưỡng mặc định
    app.th_hum    = 70;    // ngưỡng mặc định
    app.alarm     = 0;
    LED_OFF();

    // start UART RX interrupt (1 byte)
    HAL_UART_Receive_IT(&huart1, &app.rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  EventID_t active_event = pop_event(); // Kiểm tra hàng đợi

	      if (active_event != EVENT_NONE) {
	        // Thực thi nhiệm vụ không ưu tiên (Non-preemptive)
	        switch (active_event) {
	          case EVENT_TASK_TEMP:
	            TASK_DHT11_Measure(&app, &hdht11);
	            TASK_CheckThreshold_BlinkOnce(&app);
	            break;
	          case EVENT_TASK_LCD:
	            TASK_LCD_Update(&app, &lcd);
	            break;
	          case EVENT_TASK_UART:
	            TASK_UART_SendStatus(&app, &huart1);
	            break;
	          case EVENT_TASK_CMD:
	            app.line_ready = 1; // Đánh dấu để hàm parse nhận diện
	            TASK_UART_ParseLine(&app, &huart1);
	            break;
	          default: break;
	        }
	      } else {
	        __NOP();
	      }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 29999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) push_event(EVENT_TASK_TEMP);
  if (htim->Instance == TIM4) push_event(EVENT_TASK_LCD);
  if (htim->Instance == TIM1) push_event(EVENT_TASK_UART);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {

    char c = (char)app.rx_byte;

    // kết thúc dòng
    if (c == '\n' || c == '\r' || c == ';') {
      if (app.rx_len > 0) {
        app.rx_line[app.rx_len] = 0;
        app.rx_len = 0;
        push_event(EVENT_TASK_CMD); // Đẩy sự kiện xử lý lệnh
        app.line_ready = 1;
      }
    } else {
      // tích lũy ký tự
      if (app.rx_len < (sizeof(app.rx_line) - 1)) {
        app.rx_line[app.rx_len++] = c;
      } else {
        // overflow -> reset
        app.rx_len = 0;
      }
    }

    // nhận tiếp byte sau
    HAL_UART_Receive_IT(&huart1, &app.rx_byte, 1);
  }
}

/* USER CODE END 4 */

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
