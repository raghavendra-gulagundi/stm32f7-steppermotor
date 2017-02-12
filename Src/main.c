/**
 ******************************************************************************
 * @file    LTDC/LTDC_Display_1Layer/Src/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    25-June-2015
 * @brief   This example provides a description of how to configure LTDC peripheral
 *          to display BMP image on LCD using only one layer.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;


/* Private function prototypes -----------------------------------------------*/
static void LCD_Config(void);
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
#define motor_dir                        GPIO_PIN_2   //D8
#define motor_step                       GPIO_PIN_3   //D7
static TS_StateTypeDef TS_State;
/* Private functions ---------------------------------------------------------*/
#define usec_scale 1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//static TS_StateTypeDef TS_State;
static uint16_t usect = 20;
static uint16_t usecTim = 600;
static uint16_t cw = 1;
static uint16_t Phyy = 130, Phyx=50;


/* Private function prototypes -----------------------------------------------*/
static void WaitForPressedState(uint8_t Pressed);

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();

	/* STM32F7xx HAL library initialization */
	HAL_Init();

	/* Configure the system clock to 216 MHz */
	SystemClock_Config();

	Pin_Init(motor_step);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOI, motor_step, GPIO_PIN_SET);

	BSP_LED_Init(LED1);
	  /* Set TIMx instance */
	  setTimerVal(usecTim);
		/* Configure LCD : Only one layer is used */
	  LCD_Config();
	Pin_Init(motor_dir);
	Pin_Init(motor_step);

	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	HAL_Delay(1000);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	LCD_clear_redraw_screen(20);
	uint8_t status = 0;

	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	uint8_t stst = 0;

	while (1) {

		if (status == TS_OK) {

			WaitForPressedState(1);
			BSP_TS_GetState(&TS_State);
				WaitForPressedState(0);
				// FOR speed Y= 130;
			if ((TS_State.touchX[0] < 430 && TS_State.touchX[0] > 20)
					&& (TS_State.touchY[0] < 180 && TS_State.touchY[0] > 120)) {
				usect = map_lcd_scale_usec(TS_State.touchX[0]);
				usect=usect*10;
				Phyx = TS_State.touchX[0];
				setTimerVal(usecTim);
				LCD_clear_redraw_screen();
			}
			// FOR cw Y= 200;
			else if(
					(TS_State.touchX[0] < 110 && TS_State.touchX[0] > 90)
							&& (TS_State.touchY[0] < 210
									&& TS_State.touchY[0] > 190))
			{
				cw = 1;
				HAL_GPIO_WritePin(GPIOI, motor_dir, GPIO_PIN_SET);
				LCD_clear_redraw_screen();
				HAL_Delay(500);
			}
			// FOR speed Y= 230;
			else if(
					(TS_State.touchX[0] < 110 && TS_State.touchX[0] > 90)
							&& (TS_State.touchY[0] < 240
									&& TS_State.touchY[0] > 220))
			{
				cw = 0;
				HAL_GPIO_WritePin(GPIOI, motor_dir, GPIO_PIN_RESET);
				LCD_clear_redraw_screen();
				HAL_Delay(500);
			}

		}
		//  status=2;
		HAL_Delay(2);
			stst = BSP_TS_ITGetStatustouch();
		if (stst) {
			//BSP_LED_Toggle(LED1);
			status = BSP_TS_ResetTouchData(&TS_State);
		} else
			status = 3;

	}

}




/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}



static void LCD_Config(void) {
	/* LCD Initialization */
	BSP_LCD_Init();

	/* LCD Initialization */
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

	/* Enable the LCD */
	BSP_LCD_DisplayOn();

	/* Select the LCD Background Layer  */
	BSP_LCD_SelectLayer(0);

	/* Clear the Background Layer */
	BSP_LCD_Clear(LCD_COLOR_BLACK);

	/* Configure the transparency for background */
	BSP_LCD_SetTransparency(0, 100);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	/* Turn LED1 on */
	BSP_LED_On(LED1);
	while (1) {
	}
}

void HAL_delay_usecs(uint16_t usecs) {
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000 / usec_scale);
	HAL_Delay(usecs);
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
}

void map_lcd_scale_usec(uint16_t scale) {

	if (scale > 400){
			usect = scale / 2;  // /5
			usecTim= 2;
	}else if (scale > 300){
		usect = scale; // /2
		usecTim= 900;
	}else if (scale > 200){
		usect = scale / 2;  // /5
		usecTim= 750;
    }else if (scale > 100){
		usect = scale / 6;  // /10
		usecTim= 600;
    }else if (scale > 50){
		usect = scale / 10;  // /10
		usecTim= 450;
    }
}

void Pin_Init(uint16_t Led) {
	GPIO_InitTypeDef gpio_init_structure;
	GPIO_TypeDef* gpio_led;

	gpio_led = GPIOI;
	/* Enable the GPIO_LED clock */
	LED1_GPIO_CLK_ENABLE()
	;

	/* Configure the GPIO_LED pin */
	gpio_init_structure.Pin = Led;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(gpio_led, &gpio_init_structure);

	/* By default, turn off LED */
	HAL_GPIO_WritePin(gpio_led, Led, GPIO_PIN_RESET);
}

void LCD_clear_redraw_screen() {

	//	default Phyx=20,Phyy=130;
	//uint16_t Phyy = 130;
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, 24, (uint8_t*) "Motor Speed Controller",
			CENTER_MODE);

	/* Draw the ring */
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(20, Phyy + 60, (uint8_t*) "CCW", LEFT_MODE);
	BSP_LCD_FillCircle(100, Phyy + 70, 10);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(100, Phyy + 70, 8);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

	/* Draw the ring */
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(20, Phyy + 100, (uint8_t*) "CW", LEFT_MODE);
	BSP_LCD_FillCircle(100, Phyy + 110, 10);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(100, Phyy + 110, 8);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

	if (cw == 1) {
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_FillCircle(100, Phyy + 70, 4);
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_FillCircle(100, Phyy + 110, 4);
	}
	BSP_LCD_DisplayStringAt(20, Phyy - 30, (uint8_t*) "Max", LEFT_MODE);
	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize() - 60, Phyy - 30,
			(uint8_t*) "Min", LEFT_MODE);

	BSP_LCD_DrawHLine(20, Phyy, BSP_LCD_GetXSize() - 40);
	BSP_LCD_DrawHLine(20, Phyy + 40, BSP_LCD_GetXSize() - 40);
	BSP_LCD_DrawVLine(20, Phyy, 40);
	BSP_LCD_DrawVLine(BSP_LCD_GetXSize() - 20, Phyy, 40);
	BSP_LCD_FillRect(Phyx, Phyy, 30, 40);

}

/**
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable(void) {
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
static void WaitForPressedState(uint8_t Pressed) {
	TS_StateTypeDef State;

	do {
		BSP_TS_GetState(&State);
		HAL_Delay(10);
		if (State.touchDetected == Pressed) {
			uint16_t TimeStart = HAL_GetTick();
			do {
				BSP_TS_GetState(&State);
				HAL_Delay(10);
				if (State.touchDetected != Pressed) {
					break;
				} else if ((HAL_GetTick() - 100) > TimeStart) {
					return;
				}
			} while (1);
		}
	} while (1);
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED1);
  HAL_GPIO_TogglePin(GPIOI,motor_step);
}


void setTimerVal(uint16_t scl){

	  /* Set TIMx instance */
		  TimHandle.Instance = TIMx;

	 /* Compute the prescaler value to have TIMx counter clock equal to 1000000 Hz */
		  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 1000000/ 1) - 1;
		  TimHandle.Init.Period            = scl-1;//10000 - 1;
		  TimHandle.Init.Prescaler         = uwPrescalerValue;
		  TimHandle.Init.ClockDivision     = 0;
		  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
		  TimHandle.Init.RepetitionCounter = 0;

		  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
		    {
		      /* Initialization Error */
		      Error_Handler();
		    }

		    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
		    /* Start Channel1 */
		    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
		    {
		      /* Starting Error */
		      Error_Handler();
		    }

}
