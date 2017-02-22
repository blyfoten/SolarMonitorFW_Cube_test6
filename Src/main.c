/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)
#define MAX_COMMAND_LENGTH 30
#define ADC_VECTOR_LENGTH 1200
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int sweepOutputValue = 0;
int sweepMaxOutputValue = 4095;
int sweepDirection = 1;
int sweepIncrement = 5;
int adcVectorPos = 0;
volatile int lengthOfLastSweep = 0;
int idleVoltage = 0;
volatile int sweepMinOutputValue = 1000;
int sweepWendAdcVoltage = 3000;
int inputValue;
volatile int sendData = 0;
int intPart,fracPart,i;
int size;
float adcVoltage1;

uint16_t adcValues[4];
uint8_t rxSerial[2];
uint8_t txSerial[20];
char tmpString[30];
char recievedCmd[MAX_COMMAND_LENGTH];

uint16_t adcVector[ADC_VECTOR_LENGTH];
HAL_StatusTypeDef status;


FLASH_EraseInitTypeDef eraseInitDef;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void Error_Handler(void);
void serial1Print(char * a);
int scmp(char * a,char * b, int * value);
void loadState(void);
void saveState(void);
int volt2Adc(float volt);
void serial1PrintVoltage(int inValue,char * format);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  //saveState();
  /* load state */
  if ((*(uint32_t*)0x0800A000) != 0xFFFFFFFF)
	  loadState();

  /* Start Timer Channel1 */
  if (HAL_DAC_Start(&hdac,DAC_CHANNEL_1) != HAL_OK)
	  Error_Handler();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues,4) != HAL_OK)
	  Error_Handler();
  if (HAL_UART_Receive_DMA(&huart1, rxSerial, 1) != HAL_OK )
	  Error_Handler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if (sendData == 1)
	  {
		  for (i = 0;i<=lengthOfLastSweep;i+=2)
		  {
			  serial1PrintVoltage(adcVector[i],"%1d.%04d ");
			  serial1PrintVoltage(adcVector[i+1],"%1d.%04d\n");
		  }
		  sendData = 0;
	  }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcVector[adcVectorPos]   = sweepOutputValue;
	adcVector[adcVectorPos+1] = adcValues[0];
	if ( (adcValues[0] < sweepWendAdcVoltage) ||
	   (sweepOutputValue >= sweepMaxOutputValue)  )
	{
		sweepDirection = -1;
	}
	adcVectorPos += 2;
	adcVectorPos = MIN(adcVectorPos,850);

	if ( ( (sweepDirection == -1 ) &&
		   (sweepOutputValue <= sweepMinOutputValue) )||
		 (adcVectorPos == 850) )
	{
		HAL_TIM_Base_Stop_IT(&htim3);
		sweepDirection = 1;
		sendData = 1;
		sweepOutputValue = sweepMinOutputValue;
		lengthOfLastSweep = adcVectorPos-2;
		adcVectorPos = 0;
		if ( HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0) != HAL_OK)
			Error_Handler();
	}
	else
	{
		if ( HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, (uint32_t)sweepOutputValue) != HAL_OK)
			Error_Handler();

		sweepOutputValue += sweepDirection*sweepIncrement;
	}

	sweepOutputValue  = MIN(MAX(sweepOutputValue,sweepMinOutputValue),sweepMaxOutputValue);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
			 Error_Handler();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static unsigned int pos  = 0;

	if ( (strncmp((const char *)rxSerial,"\r",1) == 0 ) ||
	     (pos == (MAX_COMMAND_LENGTH -1)) )
	{
		recievedCmd[pos] = '\0';
		pos = 0;
		if (strcmp(recievedCmd,"AT+SWEEP") == 0)
		{
			if ( HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, (uint32_t)sweepOutputValue) != HAL_OK)
						Error_Handler();
			if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
				 Error_Handler();
		}
		else if (strcmp(recievedCmd,"AT") == 0)
		{
			serial1Print(" OK\n");
		}
		else if (strcmp(recievedCmd,"AT+HELP") == 0)
		{
			serial1Print("\nAvailable commands:\n\n \
					       AT\n \
					       AT+HELP\n \
					       AT+SAVE\n \
					       AT+SWEEP\n \
					       AT+SHOWSETTINGS\n \
					       AT+STARTVOLTAGE=#\n \
					       AT+ENDVOLTAGE=#\n \
					       AT+STEPSIZE=#\n\n");
		}
		else if(scmp(recievedCmd,"AT+STARTVOLTAGE=",&inputValue) == 0 )
		{
			sweepMinOutputValue = inputValue;
			serial1Print("Got: ");
			serial1PrintVoltage(sweepMinOutputValue,"%1d.%04dv");
			serial1Print(" OK\n");
		}
		else if(scmp(recievedCmd,"AT+ENDVOLTAGE=",&inputValue) == 0 )
		{
			sweepWendAdcVoltage = inputValue;
			serial1Print(" OK\n");
		}
		else if(scmp(recievedCmd,"AT+STEPSIZE=",&inputValue) == 0 )
		{
			sweepIncrement = inputValue;
			serial1Print(" OK\n");
		}
		else if(strcmp(recievedCmd,"AT+SAVE") == 0 )
		{
			saveState();
			serial1Print(" SETTINGS SAVED\n OK\n");
		}

	}
	else
	{
		recievedCmd[pos] = rxSerial[0];
		pos++;
	}
	if (HAL_UART_Receive_DMA(&huart1, rxSerial, 1) != HAL_OK )
		Error_Handler();
}

void saveState(void)
{
	uint32_t tmp;

	HAL_FLASH_Unlock();
	eraseInitDef.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitDef.PageAddress = 0x0800A000;
	eraseInitDef.NbPages = 1;
	HAL_FLASHEx_Erase(&eraseInitDef,&tmp);
	HAL_FLASH_Lock();

	HAL_FLASH_Unlock();
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A000, (uint64_t)sweepMinOutputValue);
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A004, (uint64_t)sweepMaxOutputValue);
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A008, (uint64_t)sweepIncrement);
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A00C, (uint64_t)sweepWendAdcVoltage);
	//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A010, (uint64_t)sweepMinOutputValue);
	//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A014, (uint64_t)sweepMinOutputValue);
	//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A018, (uint64_t)sweepMinOutputValue);
	//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800A01C, (uint64_t)sweepMinOutputValue);

	HAL_FLASH_Lock();
}

void loadState(void)
{
    sweepMinOutputValue = (*(int*)0x0800A000);
	sweepMaxOutputValue = (*(int*)0x0800A004);
	sweepIncrement      = (*(int*)0x0800A008);
	sweepWendAdcVoltage = (*(int*)0x0800A00C);
}

int scmp(char *a,char *b, int * value)
{
	*value = atoi(a + strlen(b));
	return strncmp(a,b,strlen(b));
}

void serial1Print(char * a)
{

	HAL_UART_Transmit(&huart1,(uint8_t *)a, (uint16_t)strlen(a), 50);
}

void serial1PrintVoltage(int inValue,char * format)
{
	  adcVoltage1 = 3.3*(float)inValue/4096.0;
	  intPart = (int)adcVoltage1;
	  fracPart = 10*((int)(1000.0*adcVoltage1) - intPart*1000);
	  size=snprintf( (char*)txSerial,9,format,intPart,fracPart);
	  HAL_UART_Transmit(&huart1,txSerial,size,20);
}

int volt2Adc(float volt)
{
	return (int)(volt);
}

void Error_Handler(void)
{
	while (1)
	{
		asm("nop");
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
