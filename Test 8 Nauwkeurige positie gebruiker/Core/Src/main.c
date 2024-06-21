/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TCA9543A_ADDR 0x70 // TCA9543A I2C-adres
#define ADS1115_BASE_ADDR 0x48 // Basisadres voor ADS1115

#define MOTOR_LEFT_PIN GPIO_PIN_15
#define MOTOR_RIGHT_PIN GPIO_PIN_14
#define MOTOR_GPIO_PORT GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint16_t results[24];
float current_position = 0.0;
float current_percentage = 0.0;
float desired_position = 50.0; // Initialiseer met een standaardwaarde
uint8_t rx_data; // Variabele om ontvangen data op te slaan
uint8_t received_number = 0; // Variabele om het ontvangen getal op te slaan
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void selectI2CChannel(uint8_t channel);
uint16_t readADS1115(uint8_t addr, uint8_t channel);
float adc_to_percentage(uint32_t adc_value);
void set_motor_position(float desired_percentage, float current_percentage);
void update_desired_position_from_uart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void selectI2CChannel(uint8_t channel) {
    uint8_t config = 1 << channel;
    HAL_I2C_Master_Transmit(&hi2c1, TCA9543A_ADDR << 1, &config, 1, HAL_MAX_DELAY);
}

uint16_t readADS1115(uint8_t addr, uint8_t channel) {
    uint8_t config[3];
    uint8_t data[2];

    // Config register to single-shot mode, selecting the correct channel
    config[0] = 0x01; // Config register

    // Configuration settings: Single-shot mode, FSR ±6.144V, 128SPS
    // Different AINx for different channels
    switch(channel) {
        case 0: config[1] = 0xC3; break; // AIN0
        case 1: config[1] = 0xD3; break; // AIN1
        case 2: config[1] = 0xE3; break; // AIN2
        case 3: config[1] = 0xF3; break; // AIN3
    }

    config[2] = 0x83; // LSB: Disable comparator

    HAL_I2C_Master_Transmit(&hi2c1, addr << 1, config, 3, HAL_MAX_DELAY);

    // Wait for the conversion to complete
    HAL_Delay(8); // ADS1115 max conversion time is 8ms

    // Read conversion result
    uint8_t reg = 0x00; // Conversion register
    HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, 2, HAL_MAX_DELAY);

    uint16_t result = (data[0] << 8) | data[1];
    return result;
}

float adc_to_percentage(uint32_t adc_value) {
    // Bereken de overeenkomstige spanning (16-bit ADC en 4.096V referentie)
    float voltage = (adc_value / 32767.0) * 4.096;

    // Lineaire conversie van spanning naar percentage
    // Spanningsbereik 0.95V tot 4.00V komt overeen met 0% tot 100%
    float percentage = (voltage - 0.95) / (4.00 - 0.95) * 100.0;

    // Beperk percentage tussen 0% en 100%
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    return percentage;
}

void set_motor_position(float desired_percentage, float current_percentage) {
    // Tolerantie in percentage
    float tolerance = 3.0;

    if (desired_percentage > (current_percentage + tolerance)) {
        // Draai naar rechts
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_RIGHT_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_LEFT_PIN, GPIO_PIN_RESET);
    } else if (desired_percentage < (current_percentage - tolerance)) {
        // Draai naar links
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_LEFT_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_RIGHT_PIN, GPIO_PIN_RESET);
    } else {
        // Stop de motor
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_LEFT_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_RIGHT_PIN, GPIO_PIN_RESET);
    }
}

void Receive_Number_Gebruiker(void)
{
    // Wacht tot er data ontvangen wordt
    if (HAL_UART_Receive(&huart5, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
    {
        // Controleer of het ontvangen getal binnen het bereik 0-100 ligt
        if (rx_data >= 0 && rx_data <= 100)
        {
            received_number = rx_data;
        }
        else
        {
            // Ongeldige invoer, stel een standaardwaarde in
            received_number = 0;
        }
    }
    else
    {
        // Ontvangst is mislukt, stel een standaardwaarde in
        received_number = 0;
    }
}

void Transmit_Current_Position(uint8_t position)
{
    char buffer[10];
    sprintf(buffer, "%d\r\n", position);
    HAL_UART_Transmit(&huart5, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART5_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart5, pData, Size, HAL_MAX_DELAY);
}

void send_position_to_esp32(uint8_t current_position)
{
  char uart_data[5]; // Buffer om de tekst te formatteren

  // Formateer de huidige positie als een string
  sprintf(uart_data, "%d", current_position);

  // Verstuur via UART5
  UART5_Transmit((uint8_t *)uart_data, strlen(uart_data));
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (received_number >= 0 && received_number <= 100)
	  {
	   desired_position = received_number;
	  }
	  else
	  {
	   desired_position = 0; // Standaardwaarde instellen
	  }

      for (uint8_t channel = 0; channel < 2; channel++) {
          selectI2CChannel(channel);
          uint8_t ads_addresses[3] = {0x4A, 0x49, 0x48}; // ADDR: SDA, 5V, GND
          for (uint8_t i = 0; i < 3; i++) {
              uint8_t addr = ads_addresses[i];
              for (uint8_t j = 0; j < 4; j++) {
                  uint16_t result = readADS1115(addr, j); // Pas de readADS1115 aan om het kanaal door te geven
                  results[channel * 12 + i * 4 + j] = result;
                  HAL_Delay(100); // Eventueel wat vertraging om het uitlezen stabiel te houden
              }
          }
      }

      uint32_t adc_value = results[20];
      current_position = adc_to_percentage(adc_value);
      Transmit_Current_Position(current_position);
      send_position_to_esp32(current_position);
      set_motor_position(desired_position, current_position);

      HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
