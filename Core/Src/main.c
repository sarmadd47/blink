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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME_ADDR (0x77 << 1)
#define LSM_ADDR (0x6A << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#define I2C_TIMEOUT 100000U

// Wait for a status flag in SR1. Returns 0 if it appeared, negative on error.
static int i2c_wait_sr1(uint32_t flag)
{
  uint32_t t = I2C_TIMEOUT;
  while (!(I2C1->SR1 & flag))
  {
    if (I2C1->SR1 & I2C_SR1_AF)        // device answered NACK
    {
      I2C1->SR1 &= ~I2C_SR1_AF;
      I2C1->CR1 |= I2C_CR1_STOP;
      return -1;
    }
    if (--t == 0)                      // gave up waiting
    {
      I2C1->CR1 |= I2C_CR1_STOP;
      return -2;
    }
  }
  return 0;
}

// Write one byte to one sensor register
static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
  I2C1->CR1 |= I2C_CR1_START;                  // 1. START
  if (i2c_wait_sr1(I2C_SR1_SB)) return -1;

  I2C1->DR = addr;                         // 2. address + write bit (0)
  if (i2c_wait_sr1(I2C_SR1_ADDR)) return -2;
  (void)I2C1->SR2;                             //    reading SR1 then SR2 clears ADDR

  I2C1->DR = reg;                              // 3. which register
  if (i2c_wait_sr1(I2C_SR1_TXE)) return -3;

  I2C1->DR = value;                            // 4. the value
  if (i2c_wait_sr1(I2C_SR1_BTF)) return -4;    //    wait until it's fully sent

  I2C1->CR1 |= I2C_CR1_STOP;                   // 5. STOP
  while (I2C1->CR1 & I2C_CR1_STOP);            //    wait for STOP to finish
  return 0;
}

// Read 'len' bytes starting at sensor register 'reg'
static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  // Part 1: tell the sensor which register we want
  I2C1->CR1 |= I2C_CR1_START;
  if (i2c_wait_sr1(I2C_SR1_SB)) return -1;
  I2C1->DR = addr;                         // address + write bit
  if (i2c_wait_sr1(I2C_SR1_ADDR)) return -2;
  (void)I2C1->SR2;
  I2C1->DR = reg;
  if (i2c_wait_sr1(I2C_SR1_BTF)) return -3;

  // Part 2: repeated START, then switch to reading
  I2C1->CR1 |= I2C_CR1_START;
  if (i2c_wait_sr1(I2C_SR1_SB)) return -4;
  I2C1->DR = addr | 1;                     // address + read bit (1)

  if (len == 1)
  {
    if (i2c_wait_sr1(I2C_SR1_ADDR)) return -5;
    I2C1->CR1 &= ~I2C_CR1_ACK;                 // NACK the only byte
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    I2C1->CR1 |= I2C_CR1_ACK;                  // ACK every byte except the last
    if (i2c_wait_sr1(I2C_SR1_ADDR)) return -5;
    (void)I2C1->SR2;
  }

  for (uint8_t i = 0; i < len; i++)
  {
    if (i2c_wait_sr1(I2C_SR1_RXNE)) return -6;
    buf[i] = I2C1->DR;
    if (i == len - 2)                          // the last byte is arriving now:
    {
      I2C1->CR1 &= ~I2C_CR1_ACK;               //   NACK it
      I2C1->CR1 |= I2C_CR1_STOP;               //   and STOP right after
    }
  }
  while (I2C1->CR1 & I2C_CR1_STOP);
  return 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t id = 0;
  int err;

  // LSM6DSOX: check ID, then turn on accel + gyro
  err = i2c_read_regs(LSM_ADDR, 0x0F, &id, 1);
  printf("LSM6DSOX: err=%d, ID=0x%02X (expect 0x6C)\r\n", err, id);
  i2c_write_reg(LSM_ADDR, 0x12, 0x44);  // CTRL3_C
  i2c_write_reg(LSM_ADDR, 0x10, 0x40);  // CTRL1_XL
  i2c_write_reg(LSM_ADDR, 0x11, 0x40);  // CTRL2_G

  // BME280: check ID, start measuring, read calibration
  err = i2c_read_regs(BME_ADDR, 0xD0, &id, 1);
  printf("BME280:   err=%d, ID=0x%02X (expect 0x60)\r\n", err, id);
  i2c_write_reg(BME_ADDR, 0xF4, 0x27);  // ctrl_meas: temp + pressure on, normal mode

  uint8_t cal[6];
  i2c_read_regs(BME_ADDR, 0x88, cal, 6);
  dig_T1 = (uint16_t)(cal[1] << 8 | cal[0]);
  dig_T2 = (int16_t)(cal[3] << 8 | cal[2]);
  dig_T3 = (int16_t)(cal[5] << 8 | cal[4]);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(200);

    uint8_t raw[12];
    if (i2c_read_regs(LSM_ADDR, 0x22, raw, 12) == 0)
    {
      int16_t gx = (int16_t)(raw[1]  << 8 | raw[0]);
      int16_t gy = (int16_t)(raw[3]  << 8 | raw[2]);
      int16_t gz = (int16_t)(raw[5]  << 8 | raw[4]);
      int16_t ax = (int16_t)(raw[7]  << 8 | raw[6]);
      int16_t ay = (int16_t)(raw[9]  << 8 | raw[8]);
      int16_t az = (int16_t)(raw[11] << 8 | raw[10]);

      printf("accel (mg): %6d %6d %6d | gyro (dps): %4d %4d %4d\r\n",
             ax * 61 / 1000, ay * 61 / 1000, az * 61 / 1000,
             gx * 875 / 100000, gy * 875 / 100000, gz * 875 / 100000);
    }
    uint8_t t[3];
    if (i2c_read_regs(BME_ADDR, 0xFA, t, 3) == 0)
    {
      int32_t adc_T = ((int32_t)t[0] << 12) | ((int32_t)t[1] << 4) | (t[2] >> 4);

      // Bosch's official integer formula from the BME280 datasheet
      int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
      int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                        ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
      int32_t temp = ((var1 + var2) * 5 + 128) >> 8;   // in hundredths of a degree C

      printf("temp: %ld.%02ld C\r\n", temp / 100, temp % 100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
