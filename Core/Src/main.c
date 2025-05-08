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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
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

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//Velinimo funkcija mikrosekundemis
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}
// Matavimo kintamieji
uint16_t RH1 = 0, RH2 = 0;
float Hum1 = 0, Hum2 = 0, Avg = 0, Avg10 = 0, sum1 = 0, sum2 = 0, sum3 = 0, Diff10 = 0;
uint8_t Diff = 0,sample = 0;


//Funkcija nustatyti pin'a kaip outpout 
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//Funkcija nustatyti pin'a kaip input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP; 
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/***************************DHT11 MATAVIMAS**********************************************/
//DHT11 jutikliu prijungimo pin'ai 
#define DHT11_1_PORT GPIOA
#define DHT11_1_PIN  GPIO_PIN_0
#define DHT11_2_PORT GPIOA
#define DHT11_2_PIN  GPIO_PIN_1

//Funkcija nusiusti pradzios signala DHT11
void DHT11_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	Set_Pin_Output(GPIOx, GPIO_Pin); //nustatom pin'a kaip output
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin,1); //pradiniu atveju laikom auksta
	delay(1000); //laukiame 1 ms
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); //nuleidziam zemyn
	delay(18000); //laukiame 18 ms
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1); //pakeliam pin'a aukstyn
	delay(20); //laukiame 20 탎
	Set_Pin_Input(GPIOx, GPIO_Pin); //nustatome kaip input
}

//DHT11 atsako funkcija 
uint8_t DHT11_Check_Response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Response = 0;
	delay(40); //laukiame 40 탎
	if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) //Patikriname ar pin'as yra zemas
	{
		delay(80); //laukiame 80 탎
		if ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) 
			Response = 1; //Patikriname ar pin'as yra pakeltas aukstyn ir nurodome, kad jutiklis atsake
		else Response = 0; //Jutiklis neatsake
	}
	while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))); //Laukiame kol pin'as vel bus zemas
	return Response;
}

//Funkcija nuskaityti 1 baita is DHT11
uint8_t DHT11_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))); //Laukiame kol pin'as bus aukstas
		delay(40); //laukiame 40 탎
		if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) //Jei zemas - tai 0
		{
			i &= ~(1 << (7 - j)); //Irasome 0
		}
		else i |= (1 << (7 - j)); //Jei aukstas - tai 1
		while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))); //Laukiame kol vel taps zemas
	}
	return i;
}

//Funkcija nuskaityti dregme is DHT11
uint8_t ReadHumidity(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint16_t SUM;

	DHT11_Start(GPIOx, GPIO_Pin);
	if (DHT11_Check_Response(GPIOx, GPIO_Pin))
	{
		Rh_byte1 = DHT11_Read(GPIOx, GPIO_Pin);
		Rh_byte2 = DHT11_Read(GPIOx, GPIO_Pin);
		Temp_byte1 = DHT11_Read(GPIOx, GPIO_Pin);
		Temp_byte2 = DHT11_Read(GPIOx, GPIO_Pin);
		SUM = DHT11_Read(GPIOx, GPIO_Pin);
		return Rh_byte1; //tik pirmas baitas dregmei
	}
	else return 255; // klaidos reiksme
}
// UART klaidu valdymo funkcija
void HandleError(void) {
    uint32_t uart_err;
    uart_err = HAL_UART_GetError(&huart2);
}

int8_t OLED1 = 1; // kintamasis pirmam kartui atnaujint indikatoriu 
char txBuf[64]; 
extern UART_HandleTypeDef huart2; 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim7)
{
	char buf[32];

	RH1 = ReadHumidity(DHT11_1_PORT, DHT11_1_PIN);
	RH2 = ReadHumidity(DHT11_2_PORT, DHT11_2_PIN);
	

	if (RH1 == 255 || RH2 == 255) { // Klaidos tikrinimas
		 const char* errorMsg = "Klaida: DHT11 nepavyko ismatuoti\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);

    // Sustabdom laikmati
    HAL_TIM_Base_Stop_IT(&htim6);

    // OLED klaidos prane쉏mas
    ssd1306_Fill(Black);
    ssd1306_SetCursor(30, 10);
    ssd1306_WriteString("Klaida", Font_11x18, White);
    ssd1306_UpdateScreen(&hi2c1);
	} 
	else {
		HAL_TIM_Base_Start_IT(&htim6); //jei viskas gerai, laikmatis vel paleidziamas
		// Skaiciavimai
		Hum1 = (float) RH1;
		Hum2 = (float) RH2;
		Avg = (Hum1 + Hum2) / 2;
		Diff = fabs(Hum1 - Hum2);

		// Duomenu perdavimas kompiuteriui
		sprintf(txBuf, "Avg: %.1f%%, Diff: %d%%\r\n", Avg, Diff);
        if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY) != HAL_OK) {
            HandleError();
        }

		// Vidurkinimas kas 10 matavimu
		sum1 += Hum1;
		sum2 += Hum2;
		sum3 += Diff;
		sample++;

		if (sample == 10 || OLED1) {
			Avg10 = (sum1 + sum2) / (2 * sample);
			Diff10 = sum3 / sample;
			sample = 0;
			sum1 = 0;
			sum2 = 0;
			sum3 = 0;

			// OLED atnaujinimas
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 5);
			sprintf(buf, "Avg: %.1f %%", Avg10);
			ssd1306_WriteString(buf, Font_11x18, White);

			ssd1306_SetCursor(0, 25);
			sprintf(buf, "Diff: %.1f %%", Diff10);
			ssd1306_WriteString(buf, Font_11x18, White);

			ssd1306_UpdateScreen(&hi2c1);
			OLED1 = 0;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @reval int
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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	// Issiunciam "STM32 READY" po paleidimo
    sprintf(txBuf, "STM32 READY\r\n");
    if (HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY) != HAL_OK) {
        HandleError();
    }
		
	ssd1306_Init(&hi2c1);
	HAL_Delay(2000); // Palaukiame 2 sekundes
  ssd1306_Fill(Black);

HAL_TIM_Base_Start(&htim6); // Paleid엍am laikmati be pertraukciu
HAL_TIM_Base_Start_IT(&htim7); // Paleid엍am su pertrauktim

  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0040131C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 24-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 24000 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT1_Pin|DHT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DHT1_Pin DHT2_Pin */
  GPIO_InitStruct.Pin = DHT1_Pin|DHT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
