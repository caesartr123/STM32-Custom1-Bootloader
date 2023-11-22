/* USER CODE BEGIN Header */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t supported_commands[] = {
                               BL_GET_VER  };

#define C_UART       &huart2
#define BL_RX_LEN    200
uint8_t bl_rx_buffer[BL_RX_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void printMessage(char *format, ...) ;
/* USER CODE BEGIN PFP */



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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */



if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
{
	printMessage("Butona basilmadi/bootloader mode\n");
	jump_to_app() ;
}else{
	printMessage("Buton basildi/app mode") ;
	bootloader_uart_read_data() ;


}


}
// int main end //

////////////*/////////////////////////////////////////////*****///////////////////////7

void bootloader_uart_read_data(void)
{
uint8_t rcv_len=0 ;
while(1)
      {
	// C_uart = &huart2
	     memset(bl_rx_buffer,0,200); ////// bufferin 200 bytei 0'lanıyor.
	     HAL_UART_Receive(&huart2, bl_rx_buffer,1, HAL_MAX_DELAY) ;
	     rcv_len=bl_rx_buffer[0];
	     HAL_UART_Receive(&huart2, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
	     switch(bl_rx_buffer[1])
	     {
	     case BL_GET_VER:
	    	 bootloader_handle_getver_cmd(bl_rx_buffer);
	    	 break ;





	     default :
	    	 printMessage("Message: Yanlis komut !!!\n");
	    	 break ;

	     }


      }
}

typedef void (*pFunction)(void);
void jump_to_app(void)
{

    uint32_t JumpAddress;
    pFunction Jump_To_Application;
    __disable_irq();
    JumpAddress = *(volatile uint32_t *) (APP_FLASH_ADDR + 4);
    __set_MSP(*(volatile uint32_t *) APP_FLASH_ADDR);
    Jump_To_Application = (pFunction) JumpAddress;
    Jump_To_Application();
}



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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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


/// FONKSIYONLAR ///

void printMessage(char *format, ...)
{
	char comingMessage[100];

	va_list vaList;
	va_start(vaList, format);
	vsprintf(comingMessage, format, vaList);
	HAL_UART_Transmit(&huart2, (uint8_t*)comingMessage, strlen(comingMessage), HAL_MAX_DELAY);
	va_end(vaList);
}

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
	uint8_t bl_version ;
	// printMessage("Message:bootloader_handle_getver_cmd\n");
	// total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
    uint32_t host_crc =*((uint32_t*)(bl_rx_buffer+command_packet_len - 4)) ;

   if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len -4 ,host_crc))
   {
	   printMessage("crc basarili\n");
	   bootloader_send_ack(bl_rx_buffer[0],1);
	   bl_version=get_bootloader_version() ;
	   printMessage("Version : %d %#x\n",bl_version,bl_version);
	   bootloader_uart_write_data(&bl_version,1);

   }else{
	   printMessage("crc basarisiz\n");
	   bootloader_send_nack() ;
   }
}

uint8_t bootloader_verify_crc(uint8_t *pData,uint32_t len ,uint32_t crc_host)
{
	uint32_t CRCValue ;
	// CRC->CR |=CRC_CR_RESET ;
	for(uint32_t i=0 ; i<len ; i++)
	{
		uint32_t i_data = pData[i] ;
		CRCValue=HAL_CRC_Accumulate(&hcrc, &i_data ,1) ;
	} __HAL_CRC_DR_RESET(&hcrc) ;
	if(CRCValue==crc_host)
	{
		return VERIFY_CRC_OKAY  ;
	}else{
		return VERIFY_CRC_NOKAY ;
	}
}
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	uint8_t ack_buffer[2] ;
	ack_buffer[0]=BL_ACK  ;
	ack_buffer[1]=BL_NACK ;
	HAL_UART_Transmit(&huart2, ack_buffer,2, HAL_MAX_DELAY);

}
void bootloader_send_nack()
{
	uint8_t nack=BL_NACK ;
	HAL_UART_Transmit(&huart2,&nack,1, HAL_MAX_DELAY);
}
uint8_t get_bootloader_version(void)
{
	return (uint8_t) BL_VERSION ;
}

void bootloader_uart_write_data(uint8_t *pBuffer ,uint32_t len)
{
	HAL_UART_Transmit(&huart2, pBuffer, len, HAL_MAX_DELAY) ;
}

