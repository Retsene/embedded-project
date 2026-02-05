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

#include <stdarg.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct {
  // sensor
  uint8_t temp_c;
  uint8_t hum_pct;
  uint8_t valid;

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



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13
#define LED_ON()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

#define MYPRINTF_BUF_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

//extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//LCD1602_I2C_t lcd;
//DHT11_HandleTypeDef hdht11;
//DHT11_Data_t dht;

LCD1602_I2C_t lcd;

DHT11_HandleTypeDef hdht11;
static AppCtx app;


//char line[17];
//uint32_t t0 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf(const char *fmt, ...)
{
    char buf[MYPRINTF_BUF_SIZE];

    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n < 0) return;

    // Nếu chuỗi bị cắt do buffer nhỏ, đảm bảo vẫn null-terminated
    buf[MYPRINTF_BUF_SIZE - 1] = '\0';

    // vsnprintf trả về số ký tự "muốn in" (không tính '\0')
    // Nếu n >= sizeof(buf) nghĩa là bị cắt, ta chỉ gửi phần có trong buffer
    uint16_t len = (n < (int)sizeof(buf)) ? (uint16_t)n : (uint16_t)(sizeof(buf) - 1);

    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
}

//TASK: đo DHT11 và lưu dữ liệu vào ctx
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

// Task kiểm tra ngưỡng + nháy LED 1 lần sau khi kiểm tra
static void TASK_CheckThreshold_And_Blink(AppCtx *ctx)
{
  // 1) cập nhật trạng thái alarm
  if (!ctx->valid) {
    ctx->alarm = 1;
  } else {
    ctx->alarm = (ctx->temp_c >= ctx->th_temp) || (ctx->hum_pct >= ctx->th_hum);
  }

  // 2) nháy LED khi alarm (không block)
  static uint32_t last_toggle = 0;
  const uint32_t blink_ms = 200;   // <-- tăng/giảm tốc độ nháy ở đây
  uint32_t now = uwTick;

  if (ctx->alarm) {
    if ((now - last_toggle) >= blink_ms) {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);  // PC13 active-low vẫn toggle OK
      last_toggle = now;
    }
  } else {
    LED_OFF();          // bình thường -> tắt hẳn
    last_toggle = now;  // reset nhịp nháy
  }
}


// Task LCD: dòng 1 = T/H, dòng 2 = chu kì + TT(Bth/Cbao)
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
  snprintf(l2, sizeof(l2), "Cyc:%4lu TT:%s",
           (unsigned long)ctx->period_ms,
           ctx->alarm ? "Cbao" : "Bth ");

  LCD1602_SetCursor(lcd, 0, 0);
  LCD1602_Print(lcd, l1);


//  LCD1602_SetCursor(lcd, 1, 0);
//  LCD1602_Print(lcd, "                ");

  LCD1602_SetCursor(lcd, 1, 0);
  LCD1602_Print(lcd, l2);
}

// Task UART TX: gửi nhiệt độ, ẩm, chu kì, ngưỡng, TT
static void TASK_UART_SendStatus(AppCtx *ctx, UART_HandleTypeDef *huart)
{
  char tx[128];
  int n;

  if (ctx->valid) {
    n = snprintf(tx, sizeof(tx),
      "T=%u H=%u P=%lu Tth=%u Hth=%u TT=%s\r\n",
      ctx->temp_c, ctx->hum_pct,
      (unsigned long)ctx->period_ms,
      ctx->th_temp, ctx->th_hum,
      ctx->alarm ? "Cbao" : "Bth"
    );
  } else {
    n = snprintf(tx, sizeof(tx),
      "DHT11_ERR P=%lu Tth=%u Hth=%u TT=%s\r\n",
      (unsigned long)ctx->period_ms,
      ctx->th_temp, ctx->th_hum,
      ctx->alarm ? "Cbao" : "Bth"
    );
  }

  HAL_UART_Transmit(huart, (uint8_t*)tx, (uint16_t)n, 100);
}

// Helper: clamp
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Task UART RX: parse line để cập nhật chu kì và ngưỡng
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
      if (sscanf(tok, "P=%lu", &p) == 1 || sscanf(tok, "p=%lu", &p) == 1 || sscanf(tok, "P%lu", &p) == 1) {
        ctx->period_ms = clamp_u32(p, 500, 60000); // giới hạn 0.5s..60s
        updated = 1;
      }
    } else if (tok[0] == 'T' || tok[0] == 't') {
      unsigned t = 0;
      if (sscanf(tok, "T=%u", &t) == 1 || sscanf(tok, "t=%u", &t) == 1 || sscanf(tok, "T%u", &t) == 1) {
        if (t <= 99) { ctx->th_temp = (uint8_t)t; updated = 1; }
      }
    } else if (tok[0] == 'H' || tok[0] == 'h') {
      unsigned h = 0;
      if (sscanf(tok, "H=%u", &h) == 1 || sscanf(tok, "h=%u", &h) == 1 || sscanf(tok, "H%u", &h) == 1) {
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */



  HAL_TIM_Base_Start(&htim2);
  TIM2_Delay_Init(&htim2);   // nếu time_delay của bạn dùng TIM2 thì cứ giữ
// init dht11
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  uint32_t now = uwTick;
//
//	  static uint32_t t_job = 0;
//
//	  // 1) Task nhận UART line (parse khi có line)
//	  TASK_UART_ParseLine(&app, &huart1);
//
//	  // 2) Task theo chu kì: đo + check ngưỡng + blink + LCD + UART
//	  if (now - t_job >= app.period_ms) {
//	    t_job = now;
//
//	    TASK_DHT11_Measure(&app, &hdht11);
//	    TASK_CheckThreshold_BlinkOnce(&app);
//	    TASK_LCD_Update(&app, &lcd);
//	    TASK_UART_SendStatus(&app, &huart1);
//	  }



	    uint32_t now = uwTick;
	    static uint32_t t_job = 0;

	    // parse UART nếu có line
	    TASK_UART_ParseLine(&app, &huart1);

	    // CHỈ 1 TASK check ngưỡng (kèm nháy LED)
//	    TASK_CheckThreshold_And_Blink(&app);

	    // theo chu kỳ đo / update
	    if (now - t_job >= app.period_ms) {
	      t_job = now;
	      myPrintf("tick_ms=%lu\r\n", (unsigned long)HAL_GetTick());
	      TASK_CheckThreshold_And_Blink(&app);

	      TASK_DHT11_Measure(&app, &hdht11);
	      // KHÔNG cần gọi check ngưỡng ở đây nữa
	      TASK_LCD_Update(&app, &lcd);
	      TASK_UART_SendStatus(&app, &huart1);
	    }

	    // optional: giảm 100% CPU
	    HAL_Delay(1);



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
  htim2.Init.Period = 0xffff;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

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
  if (huart->Instance == USART1) {

    char c = (char)app.rx_byte;

    // kết thúc dòng
    if (c == '\n' || c == '\r' || c == ';') {
      if (app.rx_len > 0) {
        app.rx_line[app.rx_len] = 0;
        app.rx_len = 0;
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
