/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "asio.h"
#include "assupport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MODULE_NAME            "NUCLEO_STM32L432KC"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int testInteger = 0;

//= global variables ======================================
static HAL_StatusTypeDef halState = 0;
static char auxText[32];

//= communication variables ===============================
volatile static char rxBuffer[MSG_BUFFER_LENGTH];
volatile static char rxWorking[MSG_BUFFER_LENGTH];
static char response[MSG_BUFFER_LENGTH];
static int rxSize = MSG_BUFFER_LENGTH;
static uint8_t *rxBuff = (uint8_t*) rxBuffer;

//= adc variables =========================================
volatile static uint16_t adcBuffer[] = { 0, 0, 0, 0 };
volatile static uint16_t adcCount = sizeof(adcBuffer) / sizeof(adcBuffer[0]);

volatile static int adcReady = 0;
//volatile static int timReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char fromBtToSerial();
char* sendATCmd(char *cmd);
void sendATCmdAsync(char *cmd);
//void clearBuffer(void *buffer, int length);

void waitToBtReady(int print);
char* removeFrame(char *msg);

GPIO_PinState btn1State();

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
	asClearBuffer(rxBuffer, sizeof(rxBuffer));
	asClearBuffer(rxWorking, sizeof(rxWorking));
	asClearBuffer(response, sizeof(response));
	asClearBuffer(adcBuffer, sizeof(adcBuffer));
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

	asIoPort = &huart2;
	asDbgPort = &huart1;

	HAL_Delay(3000);

	char *msg = asCenterText("Start: " MODULE_NAME, 60, '=', '=');
	asDebug(_NL);
	asDebug(msg);
	asDebug(_NL);

//	waitToBtReady(0);
//	char *rsp = sendATCmd("AT+HELP");
//	printf("AT+HELP" _NL);
//	printf(rsp);
//	printf(_NL);
//	printf("------------------------------"
//			"------------------------------" _NL);

//	asDebug("ADC will be calibrated." _NL);

	while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		; // calibrate AD convertor

	asDebugNL("ADC1 has been calibrated.");

//	halState = HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, adcCount);
	halState = HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, adcCount);
	if (halState != HAL_OK) {
		asDebugNL("ADC1 Start Failed: %s", asHalStatusToStr(halState));
		Error_Handler();
	}
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT); // Avoid DMA half transfer interrupt trigger
	asDebugNL("ADC1 has been started.");

	halState = HAL_TIM_Base_Start_IT(&htim15);
	if (halState != HAL_OK) {
		asDebugNL("TIM15 Star Failed: %s", asHalStatusToStr(halState));
		Error_Handler();
	}
	asDebugNL("TIM15 has been started.");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuff, rxSize);
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	while (1) {
//		if (response[0] != 0) {
//			int length = strlen(response);
//			if ((response[length - 1] < ' ')) {
//				asDebugNL(response);
//				asClearBuffer(response, sizeof(response));
//			}
//		}

		asClearBuffer(response, sizeof(response));

		if (adcReady != 0) {
			sprintf(response, "Values: ");
			for (int i = 0; i < adcCount; i++) {
				sprintf(auxText, "%6d;", adcBuffer[i]);
				strcat(response, auxText);
			}
			asDebugNL(response);
			adcReady = 0;
		}

		btn1State();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
char fromBtToSerial() {
	char ch = 0;
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == SET) {
		HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	}
	return ch;
}

char* removeFrame(char *msg) {
	int last = strlen(msg);
	last--;
	while (last >= 0 && (msg[last] == '\r' || msg[last] == '\n')) {
		msg[last--] = 0;
	}
	return msg;
}

char* sendATCmd(char *cmd) {
	static uint8_t buffer[0x1000];
	static char *pbuffer = (char*) buffer;
	static uint16_t buflen = sizeof(buffer);
	static uint16_t rxLen = 0;

	asClearBuffer(buffer, sizeof(buffer));
	memcpy(buffer, cmd, strlen(cmd) + 1);
	strcat(pbuffer, _NL);
	uint16_t length = strlen((char*) buffer);
	HAL_UART_Transmit(&huart1, buffer, length, 100);

	uint16_t currentSpace = buflen;
	asClearBuffer(buffer, sizeof(buffer));
	uint8_t *xbuffer = buffer;
	rxLen = 1;
	while (rxLen > 0) {
		HAL_UARTEx_ReceiveToIdle(&huart1, xbuffer, currentSpace, &rxLen, 100);
		xbuffer += rxLen;
		currentSpace -= rxLen;
		if (currentSpace < 0)
			break;
	}
	return pbuffer;
}

void waitToBtReady(int print) {
	int isActive = 0;
	while (isActive == 0) {
		char *rsp = sendATCmd("AT");
		if (print != 0)
			asDebugNL("Check with:    ");
		isActive = *rsp != 0;
		if (print != 0) {
			if (isActive)
				asDebugNL(removeFrame(rsp));
			else
				asDebugNL("NO ANSWER");
			asDebugNL(_NL);
		}
	}
}

void sendATCmdAsync(char *cmd) {
	HAL_UART_Transmit(&huart1, (uint8_t*) cmd, strlen(cmd), 100);
	HAL_UART_Transmit(&huart1, (uint8_t*) _NL, strlen(_NL), 100);
}

//= Test button state =====================================
GPIO_PinState btn1State() {
	GPIO_PinState btn1 = HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
	GPIO_PinState led =
			(btn1 == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(LDE1_GPIO_Port, LDE1_Pin, led);
	return led;
}

//= Callbacks function ====================================

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == htim15.Instance) {
//		timReady = 1;
//	}
//}

/**
 * @brief  Conversion complete callback in non-blocking mode
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == hadc1.Instance) {
		adcReady = 1;
	}
}

/**
 * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == huart1.Instance) {
		// příjímání povelu z nadřízené jednotky

		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

		char *pos = rxWorking + strlen(rxWorking);
		memcpy(pos, rxBuffer, Size);
		pos = rxWorking + strlen(rxWorking);
		pos--;
		if (pos >= rxWorking) {
			if (*pos == '\n') {
				strcpy(response, rxWorking);
				asClearBuffer(rxWorking, sizeof(rxWorking));
			}
		}

		asClearBuffer(rxBuffer, sizeof(rxBuffer));

		// Příjem další sekvence znaků
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuff, rxSize);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
	while (1) {
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
