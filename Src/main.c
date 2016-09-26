/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#define X_AXIS_LOW_TH	1400
#define X_AXIS_HIGH_TH	2500
#define Y_AXIS_LOW_TH	500
#define Y_AXIS_HIGH_TH	2600

#define X_AXIS			(axis[0] & 0xffffu)
#define Y_AXIS			(axis[1] & 0xffffu)
#define T_AXIS			(axis[2] & 0xffffu)
#define B_AXIS			(axis[3] & 0xffffu)
#define C_AXIS			(axis[4] & 0xffffu)

__ALIGN_BEGIN static volatile uint32_t axis[5] __ALIGN_END = { 0, 0, 0, 0, 0 };    // Y, X, T, B, C
__ALIGN_BEGIN static uint8_t rx_buffer[2] __ALIGN_END;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
#define X_AXIS_LOW_VADDR	0x1001
#define X_AXIS_HIGH_VADDR	0x1002
#define Y_AXIS_LOW_VADDR	0x1003
#define Y_AXIS_HIGH_VADDR	0x1004

__ALIGN_BEGIN uint16_t VirtAddVarTab[NB_OF_VAR] __ALIGN_END = {
		X_AXIS_LOW_VADDR, X_AXIS_HIGH_VADDR, Y_AXIS_LOW_VADDR, Y_AXIS_HIGH_VADDR };

struct __packed
{
	uint8_t		id;			// 0x01
	uint8_t		buttons;	// red + black
	uint8_t		gears;		// 1-7 (reverse)
	uint8_t		d_pad;		// lower 4 bits
	uint16_t	axis[5];
} report;

unsigned short x_low_th  = X_AXIS_LOW_TH;
unsigned short y_low_th  = Y_AXIS_LOW_TH;
unsigned short x_high_th = X_AXIS_HIGH_TH;
unsigned short y_high_th = Y_AXIS_HIGH_TH;

unsigned short report2send = 0;

/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);
/* Private function prototypes -----------------------------------------------*/

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
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

  /* EEPROM Init */
  EE_Init();

  EE_ReadVariable(VirtAddVarTab[0], &x_low_th);
  EE_ReadVariable(VirtAddVarTab[1], &x_high_th);
  EE_ReadVariable(VirtAddVarTab[2], &y_low_th);
  EE_ReadVariable(VirtAddVarTab[3], &y_high_th);

  /* Lock the Flash Program Erase controller */
  HAL_FLASH_Lock();

  report.gears = 0;
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)axis, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_SET);
	  HAL_Delay(1);

	  HAL_StatusTypeDef status =
			  HAL_SPI_Receive(&hspi1, rx_buffer, sizeof(rx_buffer), 3000);

	  switch(status) {
	      case HAL_OK:
	    	  report.buttons = 0x00;
	    	  report.d_pad = 0x00;

	    	  if ((rx_buffer[0] & 0xff) != 0xff) { // if all bits of rx_buffer[0] is 1 assume shifter is disconnected

				  if (rx_buffer[0] & 4)   report.buttons |= 1; else report.buttons &= ~1;
				  if (rx_buffer[0] & 1)   report.buttons |= (1 << 1); else report.buttons &= ~(1 << 1);
				  if (rx_buffer[0] & 2)   report.buttons |= (1 << 2); else report.buttons &= ~(1 << 2);
				  if (rx_buffer[0] & 8)   report.buttons |= (1 << 3); else report.buttons &= ~(1 << 3);

				  if (rx_buffer[1] & 1)   report.d_pad |= 1; else report.d_pad &= ~1;
				  if (rx_buffer[1] & 2)   report.d_pad |= (1 << 1); else report.d_pad &= ~(1 << 1);
				  if (rx_buffer[1] & 4)   report.d_pad |= (1 << 2); else report.d_pad &= ~(1 << 2);
				  if (rx_buffer[1] & 8)   report.d_pad |= (1 << 3); else report.d_pad &= ~(1 << 3);

				  if (rx_buffer[1] & 32)  report.buttons |= (1 << 4); else report.buttons &= ~(1 << 4);
				  if (rx_buffer[1] & 128) report.buttons |= (1 << 5); else report.buttons &= ~(1 << 5);
				  if (rx_buffer[1] & 64)  report.buttons |= (1 << 6); else report.buttons &= ~(1 << 6);
				  if (rx_buffer[1] & 16)  report.buttons |= (1 << 7); else report.buttons &= ~(1 << 7);
	    	  }
	    	  break;

		  case HAL_TIMEOUT:
		  case HAL_BUSY:
		  case HAL_ERROR:
				Error_Handler();
		  default:
				report.buttons = 0xff;
				break;
	  }

	  HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_RESET);
  	  report.id = 0x01;

      report.axis[0] = ((X_AXIS << 2) + report.axis[0] * 96) / 100;
      report.axis[1] = ((Y_AXIS << 2) + report.axis[1] * 96) / 100;

	  if (report.axis[1] < y_low_th) { // stick towards player

			if (report.axis[0] < x_low_th) {
				if (!report.gears) report.gears = 2; // 2nd gear
			} else {

				if (report.axis[0] > x_high_th) {

					if (!report.gears) report.gears = (rx_buffer[0] & 64) ? 64 : 32; // 6th gear or reverse
				} else {
					if (!report.gears) report.gears = 8; // 4th gear
				}
			}
		} else {
			if (report.axis[1] > y_high_th) { // stick opposite to player

				if (report.axis[0] < x_low_th) {
					if (!report.gears) report.gears = 1; // 1st gear
				} else {
					if (report.axis[0] > x_high_th) {

						if (!report.gears) report.gears = 16; // 5th gear
					} else {
						if (!report.gears) report.gears = 4; // 3rd gear
					}
				}
			} else {
				report.gears = 0; // neutral
			}
		}


      if (!report2send) {
		  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&report, sizeof(report));
	  } else {

		  uint8_t buf[11];

		  buf[0]  = 0x03;
		  buf[1]  = 0x01;
		  buf[3]  = x_low_th & 0xff;
		  buf[4]  = x_low_th >> 8;
		  buf[5]  = x_high_th & 0xff;
		  buf[6]  = x_high_th >> 8;
		  buf[7]  = y_low_th & 0xff;
		  buf[8]  = y_low_th >> 8;
		  buf[9]  = y_high_th & 0xff;
		  buf[10] = y_high_th >> 8;

		  if (report2send == 2) {

			  /* Unlock the Flash Program Erase controller */
			  HAL_FLASH_Unlock();

			  EE_WriteVariable(VirtAddVarTab[0], x_low_th);
			  EE_WriteVariable(VirtAddVarTab[1], x_high_th);
			  EE_WriteVariable(VirtAddVarTab[2], y_low_th);
			  EE_WriteVariable(VirtAddVarTab[3], y_high_th);

			  /* Lock the Flash Program Erase controller */
			  HAL_FLASH_Lock();

			  HAL_Delay(10);

		  } else {
			  HAL_Delay(50); // wait SP_Profiler to read all previous packets
		  }

		  if (USBD_HID_SendReport(&hUsbDeviceFS, buf, sizeof(buf)) == USBD_OK) {
			  report2send = 0;
		  }
	  }
  	  do {
  		  HAL_Delay(1);

  	  } while (hUsbDeviceFS.pClassData
  			  && ((USBD_HID_HandleTypeDef *)hUsbDeviceFS.pClassData)->state != HID_IDLE);

  /* USER CODE BEGIN 3 */

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
//  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = RCC_HSI14CALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
//  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

// ADC DMA interrupt handler
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

    report.axis[2] = ((T_AXIS << 1) + report.axis[2] * 98) / 100;
    report.axis[3] = ((B_AXIS << 1) + report.axis[3] * 98) / 100;
    report.axis[4] = ((C_AXIS << 1) + report.axis[4] * 98) / 100;
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_nCS_Pin */
  GPIO_InitStruct.Pin = SPI1_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_nCS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
