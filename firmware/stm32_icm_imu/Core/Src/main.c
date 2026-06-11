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
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)
typedef struct {
    uint16_t header;       /* 0xAA55 */
    uint32_t timestamp_us;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
    /* temp removed: not needed for VIO/SLAM */
} imu_packet_t; /* total: 18 bytes */
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMI160_REG_CHIP_ID      0x00U
#define BMI160_REG_GYR_DATA     0x0CU
#define BMI160_REG_ACC_DATA     0x12U
/* BMI160_REG_TEMP_DATA removed: not used */
#define BMI160_REG_ACC_CONF     0x40U
#define BMI160_REG_ACC_RANGE    0x41U
#define BMI160_REG_GYR_CONF     0x42U
#define BMI160_REG_GYR_RANGE    0x43U
#define BMI160_REG_CMD          0x7EU

#define BMI160_CHIP_ID          0xD1U
#define BMI160_CMD_SOFT_RESET   0xB6U
#define BMI160_CMD_ACC_NORMAL   0x11U
#define BMI160_CMD_GYR_NORMAL   0x15U

#define BMI160_CS_PORT          GPIOA
#define BMI160_CS_PIN           GPIO_PIN_4

#define BMI160_ACC_LSB_PER_G    8192
#define BMI160_GYR_LSB_PER_DPS_X10 164

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t imu_sample_flag = 0;
static uint8_t bmi160_ready = 0;
static volatile uint32_t imu_timestamp_us = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void bmi160_cs_low(void);
static void bmi160_cs_high(void);
static HAL_StatusTypeDef bmi160_read(uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef bmi160_write(uint8_t reg, uint8_t value);
static uint8_t bmi160_init(void);
static void imu_publish_sample(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void bmi160_cs_low(void)
{
  HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);
}

static void bmi160_cs_high(void)
{
  HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef bmi160_read(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;
  uint8_t addr = reg | 0x80U;

  bmi160_cs_low();
  status = HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, data, len, 10);
  }
  bmi160_cs_high();

  return status;
}

static HAL_StatusTypeDef bmi160_write(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;
  uint8_t tx[2] = {reg & 0x7FU, value};

  bmi160_cs_low();
  status = HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10);
  bmi160_cs_high();

  return status;
}

static uint8_t bmi160_init(void)
{
  uint8_t chip_id = 0;

  bmi160_cs_high();
  HAL_Delay(100);

  /* First SPI transaction wakes/selects SPI mode on BMI160 boards. */
  (void)bmi160_read(BMI160_REG_CHIP_ID, &chip_id, 1);
  HAL_Delay(2);

  if (bmi160_read(BMI160_REG_CHIP_ID, &chip_id, 1) != HAL_OK || chip_id != BMI160_CHIP_ID)
  {
    return 0;
  }

  (void)bmi160_write(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  HAL_Delay(100);
  (void)bmi160_read(BMI160_REG_CHIP_ID, &chip_id, 1);

  (void)bmi160_write(BMI160_REG_ACC_CONF, 0x2CU);  /* ODR 1600 Hz, bwp normal. */
  HAL_Delay(5);
  (void)bmi160_write(BMI160_REG_ACC_RANGE, 0x05U); /* +/- 4 g. */
  HAL_Delay(5);
  (void)bmi160_write(BMI160_REG_GYR_CONF, 0x2CU);  /* ODR 1600 Hz, bwp normal. */
  HAL_Delay(5);
  (void)bmi160_write(BMI160_REG_GYR_RANGE, 0x00U); /* +/- 2000 dps. */
  HAL_Delay(5);

  (void)bmi160_write(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  HAL_Delay(50);
  (void)bmi160_write(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  HAL_Delay(80);

  return 1;
}

/* cdc_write_line removed — not used for binary protocol */

static void imu_publish_sample(void)
{
  /* Single burst read: GYR_DATA(0x0C) + ACC_DATA(0x12) = 12 bytes contiguous */
  uint8_t raw[12];
  static imu_packet_t pkt;

  if (!bmi160_ready)
  {
    return;
  }

  /* Let USB stack decide if TX is ready — do NOT gate on user-level busy flag.
   * CDC_Transmit_FS already checks hcdc->TxState internally and returns USBD_BUSY
   * if the previous transfer is still in progress. Returning USBD_BUSY here is OK:
   * we simply skip this sample (1600 Hz, missing 1 sample is negligible). */
  if (bmi160_read(BMI160_REG_GYR_DATA, raw, sizeof(raw)) != HAL_OK)
  {
    return;
  }

  pkt.header = 0xAA55U;

  __disable_irq();
  pkt.timestamp_us = imu_timestamp_us;
  __enable_irq();

  pkt.gx = (int16_t)((uint16_t)raw[1]  << 8 | raw[0]);
  pkt.gy = (int16_t)((uint16_t)raw[3]  << 8 | raw[2]);
  pkt.gz = (int16_t)((uint16_t)raw[5]  << 8 | raw[4]);
  pkt.ax = (int16_t)((uint16_t)raw[7]  << 8 | raw[6]);
  pkt.ay = (int16_t)((uint16_t)raw[9]  << 8 | raw[8]);
  pkt.az = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);

  /* USBD_BUSY = previous packet still transmitting → skip this sample, try next tick */
  (void)CDC_Transmit_FS((uint8_t *)&pkt, (uint16_t)sizeof(pkt));
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  bmi160_ready = bmi160_init();
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (imu_sample_flag)
    {
      imu_sample_flag = 0;
      imu_publish_sample();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    imu_sample_flag = 1;
    imu_timestamp_us += 625;
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
