/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht3x.h"
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
sht3x_t sht3x_sensor;

sht3x_result_t result;
sht3x_data_t data;
uint16_t status;
uint32_t measurement_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static sht3x_result_t stm32f103_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len);
static sht3x_result_t stm32f103_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len);
static void stm32f103_delay(sht3x_t* handle, uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
    return len;
}
#endif
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
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  printf("STM32F103C8T6 SHT3x Sensor Example\r\n");
  printf("==================================\r\n");

  /* Test I2C communication first */
  printf("Testing I2C communication...\r\n");

  /* Check if SHT3x device is present on I2C bus */
  HAL_StatusTypeDef i2c_status = HAL_I2C_IsDeviceReady(&hi2c1, SHT3X_ADDR_A << 1, 3, 1000);
  if (i2c_status == HAL_OK) {
      printf("SHT3x device detected at address 0x%02X\r\n", SHT3X_ADDR_A);
  } else {
      printf("SHT3x device NOT detected at address 0x%02X (status: %d)\r\n", SHT3X_ADDR_A, i2c_status);

      /* Try alternative address */
      i2c_status = HAL_I2C_IsDeviceReady(&hi2c1, SHT3X_ADDR_B << 1, 3, 1000);
      if (i2c_status == HAL_OK) {
          printf("SHT3x device detected at address 0x%02X\r\n", SHT3X_ADDR_B);
          sht3x_sensor.addr = SHT3X_ADDR_B;
      } else {
          printf("SHT3x device NOT detected at address 0x%02X either\r\n", SHT3X_ADDR_B);
          printf("Check wiring and power supply!\r\n");
          Error_Handler();
      }
  }

  /* Scan I2C bus for all devices */
  printf("Scanning I2C bus...\r\n");
  uint8_t devices_found = 0;
  for (uint8_t addr = 1; addr < 128; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 100) == HAL_OK) {
          printf("Device found at address: 0x%02X\r\n", addr);
          devices_found++;
      }
  }
  if (devices_found == 0) {
      printf("No I2C devices found! Check pull-up resistors and connections.\r\n");
      Error_Handler();
  } else {
      printf("Total devices found: %d\r\n", devices_found);
  }

  /* Configure SHT3x sensor */
  sht3x_sensor.i2c_write = stm32f103_i2c_write;
  sht3x_sensor.i2c_read = stm32f103_i2c_read;
  sht3x_sensor.delay = stm32f103_delay;
  sht3x_sensor.addr = SHT3X_ADDR_A; /* ADDR pin connected to GND */
  sht3x_sensor.user_data = NULL;

  /* Initialize SHT3x sensor */
  printf("Initializing SHT3x sensor...\r\n");
  result = sht3x_init(&sht3x_sensor);
  if (result != SHT3X_OK) {
      printf("SHT3x initialization failed: %d\r\n", result);

      /* Additional debug information */
      switch (result) {
          case SHT3X_ERR_PARAM:
              printf("Error: Invalid parameters\r\n");
              break;
          case SHT3X_ERR_I2C:
              printf("Error: I2C communication failed\r\n");
              break;
          case SHT3X_ERR:
              printf("Error: Generic error (likely soft reset failed)\r\n");
              break;
          default:
              printf("Error: Unknown error code\r\n");
              break;
      }
      Error_Handler();
  }

  printf("SHT3x initialized successfully\r\n");

  /* Check sensor status */
  result = sht3x_read_status(&sht3x_sensor, &status);
  if (result == SHT3X_OK) {
      printf("Initial status: 0x%04X\r\n", status);

      /* Clear any error flags */
      if (status & 0x8013) {
          sht3x_clear_status(&sht3x_sensor);
          printf("Status cleared\r\n");
      }
  }

  /* Set sensor configuration */
  sht3x_set_repeatability(&sht3x_sensor, SHT3X_REPEATABILITY_HIGH);
  printf("Configuration: High repeatability mode\r\n");
  printf("Starting measurements...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* Turn on LED before measurement */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

      /* Start single shot measurement */
      result = sht3x_start_single_shot(&sht3x_sensor);
      if (result != SHT3X_OK) {
          printf("Failed to start measurement: %d\r\n", result);
          HAL_Delay(1000);
          continue;
      }

      /* Read measurement data */
      result = sht3x_read_data(&sht3x_sensor, &data);
      if (result == SHT3X_OK) {
          measurement_count++;

          /* Validate data range */
          if (data.temperature >= -40.0f && data.temperature <= 125.0f &&
              data.humidity >= 0.0f && data.humidity <= 100.0f) {

              printf("Measurement #%lu: T=%.2f°C, RH=%.2f%%\r\n",
                     measurement_count, data.temperature, data.humidity);

              /* Turn off LED to indicate successful reading */
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

          } else {
              printf("Measurement #%lu: Invalid data received\r\n", measurement_count);
          }
      } else {
          printf("Failed to read data: %d\r\n", result);

          /* Try to recover by resetting sensor */
          printf("Attempting sensor reset...\r\n");
          result = sht3x_reset(&sht3x_sensor);
          if (result == SHT3X_OK) {
              printf("Sensor reset successful\r\n");
          } else {
              printf("Sensor reset failed: %d\r\n", result);
          }
      }

      /* Check status every 10 measurements */
      if (measurement_count % 10 == 0) {
          result = sht3x_read_status(&sht3x_sensor, &status);
          if (result == SHT3X_OK) {
              printf("Status check: 0x%04X", status);
              if (status & 0x0002) {
                  printf(" [CMD_ERR]");
              }
              if (status & 0x0001) {
                  printf(" [CRC_ERR]");
              }
              printf("\r\n");
          }
      }

      /* Wait 2 seconds between measurements */
      HAL_Delay(2000);
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

/* USER CODE BEGIN 4 */
/**
 * \brief           I2C write function for STM32F103C8T6
 */
static sht3x_result_t
stm32f103_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len) {
    HAL_StatusTypeDef status;

    printf("I2C Write: addr=0x%02X, len=%d, data=[", addr, (int)len);
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X", data[i]);
        if (i < len - 1) printf(" ");
    }
    printf("]\r\n");

    status = HAL_I2C_Master_Transmit(&hi2c1, addr << 1, (uint8_t*)data, len, 1000);

    printf("I2C Write result: %s\r\n", (status == HAL_OK) ? "OK" : "FAILED");

    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

/**
 * \brief           I2C read function for STM32F103C8T6
 */
static sht3x_result_t
stm32f103_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len) {
    HAL_StatusTypeDef status;

    printf("I2C Read: addr=0x%02X, len=%d\r\n", addr, (int)len);

    status = HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, len, 1000);

    if (status == HAL_OK) {
        printf("I2C Read result: OK, data=[");
        for (size_t i = 0; i < len; i++) {
            printf("0x%02X", data[i]);
            if (i < len - 1) printf(" ");
        }
        printf("]\r\n");
    } else {
        printf("I2C Read result: FAILED\r\n");
    }

    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

/**
 * \brief           Delay function for STM32F103C8T6
 */
static void
stm32f103_delay(sht3x_t* handle, uint32_t ms) {
    HAL_Delay(ms);
}
/* USER CODE END 4 */

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
