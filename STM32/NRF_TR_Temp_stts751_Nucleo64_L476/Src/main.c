/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "stts751_reg.h"
#include "nRF24L01P.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static axis1bit16_t data_raw_temperature;
static float temperature_degC;
static stts751_id_t whoamI;
static char tx_buffer[5];

nRF24L01P myNRF;
const uint8_t RXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t TXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
//char TXData[5];

uint8_t RXBuffer[32], TXBuffer[32];
volatile uint8_t regStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
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
	stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;
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
  stts751_device_id_get(&dev_ctx, &whoamI);

  if ( //(whoamI.product_id != STTS751_ID_0xxxx) ||
           (whoamI.product_id != STTS751_ID_1xxxx) ||
             (whoamI.manufacturer_id != STTS751_ID_MAN) ||
             (whoamI.revision_id != STTS751_REV) )
  	  while(1); /* manage here device not found */

   /* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
   float temperature_high_limit = 49.5f;
   stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

   float temperature_low_limit = -4.5f;
   stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

   stts751_pin_event_route_set(&dev_ctx,  PROPERTY_ENABLE);

   /* Set Output Data Rate */
   stts751_temp_data_rate_set(&dev_ctx, STTS751_TEMP_ODR_1Hz);

   /* Set Resolution */
   stts751_resolution_set(&dev_ctx, STTS751_11bit);

   myNRF.hspi = &hspi1;														// You should make this definition at CubeMX
   myNRF.CRC_Width = nRF_CRC_WIDTH_HALFWORD;
   myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
   myNRF.Data_Rate = nRF_DATA_RATE_2MBPS;
   myNRF.TX_Power = nRF_TX_PWR_0dBm;
   myNRF.State = nRF_STATE_TX;

   myNRF.RF_Channel = 120;
   myNRF.PayloadWidth = nRF_RXPW_32BYTES;
   myNRF.RetransmitCount = nRF_RETX_DISABLED;
   myNRF.RetransmitDelay = nRF_RETX_DELAY_1000uS;

   myNRF.RX_Address = (uint8_t *)RXAddr;
   myNRF.TX_Address = (uint8_t *)TXAddr;

   myNRF.RX_Buffer = RXBuffer;
   myNRF.TX_Buffer = TXBuffer;

   myNRF.nRF_nSS_GPIO_PORT = nRF24_nSS_GPIO_Port;	// You should make this definition at CubeMX
   myNRF.nRF_nSS_GPIO_PIN = nRF24_nSS_Pin;					// You should make this definition at CubeMX

   myNRF.nRF_CE_GPIO_PORT = nRF24_CE_GPIO_Port;		// You should make this definition at CubeMX
   myNRF.nRF_CE_GPIO_PIN = nRF24_CE_Pin;						// You should make this definition at CubeMX

   if(HAL_nRF24L01P_Init(&myNRF) != HAL_OK)
   	{
   		printf("nRF24 Init fucked UP!\r\n");
   		//Error_Handler();
   	} else {
   		printf("nRF24 Init is OK!\r\n");

   printf ("Application has started!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* Read output only if not busy */
	uint8_t flag;
    stts751_flag_busy_get(&dev_ctx, &flag);

    if (flag)
    {
	/* Read temperature data */
	memset(data_raw_temperature.u8bit, 0, sizeof(int16_t));
	stts751_temperature_raw_get(&dev_ctx, &data_raw_temperature.i16bit);
	temperature_degC = stts751_from_lsb_to_celsius(data_raw_temperature.i16bit);

	sprintf(tx_buffer,"%3.1f",temperature_degC);
	printf("Temperature:  *%s* degres\r\n",tx_buffer);

    if(!myNRF.Busy)
    		{
    			if(HAL_nRF24L01P_TransmitPacket(&myNRF, (uint8_t *)tx_buffer) != HAL_OK)
    			{
    				printf("There was something wrong!\n\r");
    				//Error_Handler();
    			}
    			else
    				printf("Data sent!\r\n");
    		}
    HAL_Delay(1000);
    }
  }
  /* USER CODE END 3 */
}
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, STTS751_1xxxx_ADD_7K5, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, STTS751_1xxxx_ADD_7K5, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}

PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART1 and Loop until the end of transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);

return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == nRF24_INT_Pin)
	{
		//printf("Interrupt OK!\n\r");
		if(HAL_nRF24L01P_IRQ_Handler(&myNRF) != HAL_OK)
		{
			Error_Handler();
		}
	 }
		//myNRF.Busy = 1;
		//HAL_nRF24L01P_ReadRegister(&myNRF, nRF_STATUS, &regStatus);
		//printf("Status >");
		//HAL_UART_Transmit(&huart1, &regStatus, 1, 250);
		//uartSent = 1;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
