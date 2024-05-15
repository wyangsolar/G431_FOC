/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：interrupt_callback.h
  * @version  ：v1.0
  * @date     ：2023年12月01日
  * @brief    ：基于STM32CubeMX HAL库的中断回调程序
  * @attention:					
	* @bug      : 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "interrupt_callback.h"
/* defines ------------------------------------------------------------------*/

/* code ------------------------------------------------------------------*/
/**
* @brief 	 重写定时器中断回调函数
* @param  
* @retval 
* @attention
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		
	}
	else if(htim == &htim2)
	{
		
	}
	else if(htim == &htim17)  //10KHz
	{
		LED_Interrupt();
		Button_Interrupt();
	
	}
	
}

/**
* @brief 	 重写PWM脉冲结束中断回调函数，触发频率20khz
* @param  
* @retval 
* @attention
*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)     // 20KHz 定时器的通道4中断请求
		{
			HFI_test_time_update();
			//HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		}
	}
	else if(htim == &htim2)//1KHz
	{
		
	}
	else if(htim == &htim17)//没开PWM,不会进入
	{
		
	}
}	

/**
* @brief 	 重写ADC规则通道的中断回调函数,定时器通道2触发频率1khz
* @param  
* @retval 
* @attention 轮询计算
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1)
	{
		//HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		Look_Up_Tables_resistance_and_temperature(&Motor_ADC);	//查找表轮询NTC阻值对应的温度
	}
	else if(hadc == &hadc2)
	{
		
	}
	
}


/**
* @brief 	 	重写ADC注入通道的中断回调函数，定时器1更新事件触发频率20khz,计数模式中央对齐模式3,向上和向下计数溢出都会触发中断
*						所以RCR= 0 ,中断频率为 20KHz / 0.5 = 40KHz
*								RCR= 1 ,中断频率为 20KHz / 1 = 20KHz
*								RCR= 2 ,中断频率为 20KHz / 1.5 = 13.3333KHz
*           这里RCR设定为2,即再重复两次计数才会中断;
* @param  
* @retval 
* @attention
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) //13.3333KHz
{
	three_phase_current_calc_interrupt(&Motor_ADC , &htim1 , hadc);
}
