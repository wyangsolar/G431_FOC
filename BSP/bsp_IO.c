/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_IO.h
  * @version  ：v1.0
  * @date     ：2021年2月1日
  * @brief    ：基于STM32CubeMX HAL库的GPIO功能函数,包含LED,按键等功能函数.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_IO.h"
/* defines ------------------------------------------------------------------*/
__IO static uint16_t led_toggle_threshold = 0;	//LED翻转阈值
__IO static uint8_t led_interrupt_mode = 0;			//LED中断模式

__IO static uint8_t button_state = 0;	//按键状态
/* code ------------------------------------------------------------------*/
/**
* @brief      LED状态
* @param      state:on ; state:off
* @retval     None
* @attention  None
*/
void LED_State(uint8_t state)
{
	if(state==off)
	{
		HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
	}
	
	switch(state)
	{
		case off:
		{
			led_toggle_threshold = 10000;
			led_interrupt_mode = off;
		}break;
		case on:
		{
			led_toggle_threshold = 0;
			led_interrupt_mode = on;
		}break;
		case breathe:
		{
			led_toggle_threshold = 0;
			led_interrupt_mode = breathe;
		}break;
		case twinkle:
		{
			led_toggle_threshold = 5000;
			led_interrupt_mode = twinkle;
		}break;
	}
}

/**
* @brief      LED 定时器中断执行程序,推荐10KHz中断频率
* @param      
* @retval     
* @attention  
*/
void LED_Interrupt(void)
{
	__IO static uint16_t times = 0;
	__IO static uint8_t led_flag = 0;
	
	switch(led_interrupt_mode)
	{
		case off:
		{
			
		}break;
		case on:
		{
			
		}break;
		case twinkle:
		{
			if(times < led_toggle_threshold)
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);//熄灭
			}
			else
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);//点亮
			}
			times++;
			if(times > 10000)
			{
				times = 0;
			}
		}break;
		case breathe:
		{
			if(times < led_toggle_threshold)
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);//熄灭
			}
			else
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);//点亮
			}
			times++;
			if(times > 100)
			{
				times = 0;
				if(led_flag == 0)
				{
					led_toggle_threshold++;//逐渐熄灭
				}
				else if(led_flag == 1)
				{
					led_toggle_threshold--;//逐渐点亮
				}
				if(led_toggle_threshold > 100)
				{
					led_toggle_threshold = 100;
					led_flag = 1;
				}
				else if(led_toggle_threshold < 1)
				{
					led_toggle_threshold = 0;
					led_flag = 0;
				}
			}
		}break;
	}
}

/**
* @brief 	   读取按键状态
* @param  
* @retval 
* @attention 放在中断里运行
*/
void Read_Button(void)
{
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
	{
		button_state = off;//按键松开
	}
	else
	{
		button_state = on;//按键按下
	}
}

/**
* @brief 	   返回按键状态
* @param  
* @retval 
* @attention 
*/
uint8_t Get_Button_Value(void)
{
	return button_state;
}
	

/**
* @brief 			按键 定时器中断执行程序,推荐10KHz中断频率
* @param      
* @retval     
* @attention  
*/
void Button_Interrupt(void)
{
	__IO static uint16_t times = 0;
	if(200 <= times) //每隔200次就来读取一下按键状态,50Hz的检测频率,起防抖动作用
	{
		Read_Button();
		times = 0;
	}
	times ++;
}
