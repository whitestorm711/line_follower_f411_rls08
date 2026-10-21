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
#include <stdlib.h>
#include <stdint.h>
#include "sensorup.h"
#include "motordrive.h"
#include "config.h"
#include "usb_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

float Kp = 0.1f;
float Ki = 0.0f;
float Kd = 0.2f;

int flag = 0;

float error = 0.0f;
float prev_error = 0.0f;
float integral = 0.0f;

int16_t base_speed = 500;
int16_t max_speed  = 1049;      /* = TIM4 ARR (PSC=3, ARR=1049 -> 20 kHz at 84 MHz) */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT_PERIOD_MS   50U
#define BTN_DEBOUNCE_MS  20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* Returns 1 exactly once per press (on the debounced falling edge of PA0) */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
float calculate_error(void)
{
    int32_t weighted_sum = 0;
    int32_t active_count = 0;

    const int16_t weights[8] = {
        -3500, -2500, -1500, -500,
          500,  1500,  2500, 3500
    };

    for (int i = 0; i < 8; i++)
    {
        if (senval[i])
        {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0)
    {
        return prev_error;  // Line lost: retain last direction
    }

    return (float)weighted_sum / (float)active_count;
}

float PID_Update(void)
{
    error = calculate_error();

    integral += error;

    float derivative = error - prev_error;

    float output = (Kp * error)
                 + (Ki * integral)
                 + (Kd * derivative);

    prev_error = error;

    return output;
}

void line_follow(void)
{
    float correction = PID_Update();

    int32_t left_speed  = base_speed + (int32_t)correction;
    int32_t right_speed = base_speed - (int32_t)correction;

    if (left_speed > max_speed) left_speed = max_speed;
    if (left_speed < 0)         left_speed = 0;

    if (right_speed > max_speed) right_speed = max_speed;
    if (right_speed < 0)         right_speed = 0;

    LF_SetMotorSpeed((uint16_t)right_speed, (uint16_t)left_speed);
}

/* ---- USB debug output ---------------------------------------------------- */
static uint8_t int_to_str(int32_t v, char *out)
{
    char tmp[11];
    uint8_t n = 0, t = 0;

    if (v < 0) { out[n++] = '-'; v = -v; }
    if (v == 0) tmp[t++] = '0';
    while (v > 0) { tmp[t++] = (char)('0' + (v % 10)); v /= 10; }
    while (t) out[n++] = tmp[--t];
    return n;
}

/* Prints one line like:  S:00011000 E:-500 L:650 R:350
 *   S = sensor bits S1..S8 (left to right, 1 = senval[i] true)
 *   E = error from calculate_error()
 *   L / R = TIM4 CCR3 / CCR4 (the PWM duty actually applied) */
static void usb_debug_print(void)
{
    char line[48];
    uint8_t n = 0;

    line[n++] = 'S'; line[n++] = ':';
    for (int i = 0; i < 8; i++) line[n++] = senval[i] ? '1' : '0';

    line[n++] = ' '; line[n++] = 'E'; line[n++] = ':';
    n += int_to_str((int32_t)error, &line[n]);

    line[n++] = ' '; line[n++] = 'L'; line[n++] = ':';
    n += int_to_str((int32_t)TIM4->CCR3, &line[n]);

    line[n++] = ' '; line[n++] = 'R'; line[n++] = ':';
    n += int_to_str((int32_t)TIM4->CCR4, &line[n]);

    line[n++] = '\r'; line[n++] = '\n';

    cdc_write((const uint8_t *)line, n);    /* non-blocking, never stalls the PID loop */
}

/* USER CODE END PFP */
static uint8_t button_pressed(void)
{
    static uint8_t  stable   = 1U;   /* debounced level, 1 = released (pull-up) */
    static uint8_t  last_raw = 1U;
    static uint32_t t_change = 0U;

    uint8_t  raw = (GPIOA->IDR & (1U << 0U)) ? 1U : 0U;
    uint32_t now = HAL_GetTick();

    if (raw != last_raw)             /* input moved: restart the timer */
    {
        last_raw = raw;
        t_change = now;
    }
    else if ((raw != stable) && ((now - t_change) >= BTN_DEBOUNCE_MS))
    {
        stable = raw;                /* held steady long enough: accept it */
        if (stable == 0U) return 1U; /* pressed */
    }
    return 0U;
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GO(void)
{

	if(senval[7] == 1 ){
		LF_SetMotorSpeed(0,600);


	}
	if(senval[0] == 1 ){
			LF_SetMotorSpeed(600,0);

		}
	else
	{
    line_follow();
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
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  initconfig();
  LF_SetMotorDirection(1U, 1U);
  cdc_init();                       /* start USB virtual COM port */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_last = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    senup();

    // Optional start button (PA0), uncomment to hold the robot until pressed:
    if (button_pressed()) flag = !flag;   /* press to start, press again to stop */
    if (flag == 1) GO();

    if ((HAL_GetTick() - t_last) >= DEBUG_PRINT_PERIOD_MS)
    {
        t_last = HAL_GetTick();
        usb_debug_print();
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
