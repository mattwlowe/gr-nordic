/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "nRF24L01P.h"
#include "ssd1306.h"
#include "fonts.h"
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
nRF24L01P myNRF;
const uint8_t RXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t TXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
//char TXData[5];
char strrec[32];
uint8_t RXBuffer[32], TXBuffer[32], uartSent;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    myNRF.hspi = &hspi1;														// You should make this definition at CubeMX
  	myNRF.CRC_Width = nRF_CRC_WIDTH_HALFWORD;
  	myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
  	myNRF.Data_Rate = nRF_DATA_RATE_2MBPS;
  	myNRF.TX_Power = nRF_TX_PWR_0dBm;
  	myNRF.State = nRF_STATE_RX;

  	myNRF.RF_Channel = 120;
  	myNRF.PayloadWidth = nRF_RXPW_8BYTES;
  	myNRF.RetransmitCount = nRF_RETX_DISABLED;;
  	myNRF.RetransmitDelay = nRF_RETX_DELAY_1000uS;

  	myNRF.RX_Address = (uint8_t *)RXAddr;
  	myNRF.TX_Address = (uint8_t *)TXAddr;

  	myNRF.RX_Buffer = RXBuffer;
  	myNRF.TX_Buffer = TXBuffer;

  	myNRF.nRF_nSS_GPIO_PORT = nRF24_nSS_GPIO_Port;	// You should make this definition at CubeMX
  	myNRF.nRF_nSS_GPIO_PIN = nRF24_nSS_Pin;					// You should make this definition at CubeMX

  	myNRF.nRF_CE_GPIO_PORT = nRF24_CE_GPIO_Port;		// You should make this definition at CubeMX
  	myNRF.nRF_CE_GPIO_PIN = nRF24_CE_Pin;						// You should make this definition at CubeMX

  	if(HAL_nRF24L01P_Init((nRF24L01P *)&myNRF) != HAL_OK)
  	{
  		printf("nRF24 Init fucked UP!\r\n");
  		Error_Handler();
  	} else {
  		printf("nRF24 Init is OK!\r\n");
  	}

  	uartSent = 1;

  	printf("Application has started!\r\n");

  	SSD1306_Init();
  	SSD1306_Fill(0);
  	SSD1306_GotoXY(0,0);
  	SSD1306_Puts("NRF24 Temp.", &Font_11x18, SSD1306_COLOR_WHITE);
  	SSD1306_UpdateScreen();
  	SSD1306_GotoXY(0,22);
  	SSD1306_Puts("Receiver", &Font_11x18, SSD1306_COLOR_WHITE);
  	SSD1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if((myNRF.RX_Buffer[0] != 0x00) && !myNRF.Busy && uartSent)
	  		{
		        sprintf(strrec,"%s",myNRF.RX_Buffer);
		        for (int i=5;i<8;i++)
		        	 strrec[i]='\0';
	  			SSD1306_GotoXY(0,45);
	  			SSD1306_Puts("Temp:", &Font_11x18, SSD1306_COLOR_WHITE);
	  			SSD1306_UpdateScreen();
	  			SSD1306_GotoXY(62,45);
	  			SSD1306_Puts(strrec, &Font_11x18, SSD1306_COLOR_WHITE);
	  		    SSD1306_UpdateScreen();
	  			printf("%s\r\n",strrec);
	  			uartSent = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == nRF24_INT_Pin)
	{
		//HAL_UART_Transmit(&huart1, (uint8_t *)"Interrupt is OK!\r\n", 19, 250);
		if(HAL_nRF24L01P_IRQ_Handler((nRF24L01P *)&myNRF) != HAL_OK)
		{
			Error_Handler();
		}
		/*HAL_nRF24L01P_ReadRegister(&myNRF, nRF_STATUS, &regStatus);
		HAL_UART_Transmit(&huart1, (uint8_t *)"Status >", 8, 250);
		HAL_UART_Transmit(&huart1, &regStatus, 1, 250);
		HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 250);*/
		uartSent = 1;
	}
}
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART1 and Loop until the end of transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);

return ch;
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
	  while(1) {
	  	printf("There was something wrong!\n\r");
	  		HAL_Delay(1000);
	  	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
