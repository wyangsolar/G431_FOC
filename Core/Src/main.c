/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	uint8_t char_buf_1[30];
	//uint16_t ADC_Value = 0;
	float bus_voltage = 0;
	float temperature = 0;
	float potentiometer = 0;
	//float I = 0;
	float x = 0.2f;
	float y = 0.3f;
	float z = 0.0f;
	
	float tag_value = 0;
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
  MX_OPAMP3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	Uart_receive_Init(&Uart2);//串口二接收初始化
	PWM_Start();							//PWM启动
	ADC_Start(&Motor_ADC);		//ADC启动
	
	LED_State(off);						//LED状态：关

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Get_bus_voltage(&Motor_ADC,&bus_voltage);
		//printf("%.6f\n",bus_voltage);
		
		//temperature = Get_temperature(&Motor_ADC);
		//printf("%.1f\n",temperature);
			
		//Get_potentiometer(&Motor_ADC,&potentiometer);
		//printf("%.6f\n",potentiometer);
		
		//printf("%d\n",Get_Button_Value());
		
		//Uart_printf(&Uart2,"%.3f,%.3f,%.3f\n",Motor_ADC.current_meas_phU,Motor_ADC.current_meas_phV,Motor_ADC.current_meas_phW);
		
		//printf("%.3f,%.3f,%.3f\n",Motor_ADC.calib_phU,Motor_ADC.calib_phV,Motor_ADC.calib_phW);
		
		//printf("%d,%.3f,%.3f\n",Motor_ADC.adc_val_phU,Motor_ADC.current_phU,Motor_ADC.calib_phU);
		//printf("%d,%.3f,%.3f\n",Motor_ADC.adc_val_phV,Motor_ADC.current_phV,Motor_ADC.calib_phV);
		//printf("%d,%.3f,%.3f\n",Motor_ADC.adc_val_phW,Motor_ADC.current_phW,Motor_ADC.calib_phW);
		if(Uart_scanf(&Uart2,char_buf_1,sizeof(char_buf_1)))
		{
			//Uart_printf(&Uart2,"%s\r\n",char_buf_1);//把接收到的字符再发送出去
			
			sscanf((char *)char_buf_1,"%f",&tag_value);//把字符串转化成数值
			//set_tag_val(tag_value);
			
			if('0' == char_buf_1[0])
			{
				LED_State(off);
				
				//svpwm_test();
				//pll_test();
				//PWM_HighLeaveTime_test();
				HFI_test_init();
				//Uart_printf(&Uart2,"%.6f\r\n",sinf(PI/3));
				//Uart_printf(&Uart2,"%.6f\r\n",arm_sin_f32(PI/3));
				//arm_mult_f32(&x, &y, &z, 1);
				//Uart_printf(&Uart2,"%.6f\r\n",z);
				
			}
			else if('1' == char_buf_1[0])
			{
				LED_State(on);
				//dsp_test_demo();
			}
			else if('2' == char_buf_1[0])
			{
				LED_State(breathe);
			}
			else if('3' == char_buf_1[0])
			{
				LED_State(twinkle);
			}
		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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
