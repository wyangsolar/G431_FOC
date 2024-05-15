/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_PWM.h
  * @version  ：v1.0
  * @date     ：2023年11月29日
  * @brief    ：基于STM32CubeMX HAL库的G4开发板的PWM功能函数.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "bsp_PWM.h"
/* defines ------------------------------------------------------------------*/


/* code ------------------------------------------------------------------*/
/**
* @brief  PWM初始化
* @param  
* @retval 
* @attention
*/
void PWM_Start(void)
{
	// 初始化定时器
	uint32_t half_load = (PWM_PERIOD_CYCLES / 2) / 2;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,half_load);//TIM 捕获/比较寄存器 1 设定为重载值一半  50%占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,half_load);//TIM 捕获/比较寄存器 2 设定为重载值一半  50%占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,half_load);//TIM 捕获/比较寄存器 3 设定为重载值一半  50%占空比
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    	//开启定时器1通道1 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);	//开启定时器1通道1 PWM(互补)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    	//开启定时器1通道2 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  //开启定时器1通道2 PWM(互补)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);			//开启定时器1通道3 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  //开启定时器1通道3 PWM(互补)
	
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);    	//开启定时器1通道4 PWM
	
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);    	//开启定时器2通道2 PWM
	
	HAL_TIM_Base_Start_IT(&htim17);	//开启定时器17
}

/**
* @brief  设置ABC各相 PWM 高电平时间 us,由于存着死区,需要死区补偿后才能填入
* @param  
* @retval 
* @attention 170MHz = 5.8824ns,输入值不能超过 50
*/
void set_pwm_high_leave_time_us(TIM_HandleTypeDef *htim,float U_time,float V_time,float W_time)
{
	float once_time = (1000000.0f / ADV_TIM_CLK_FREQ);//计数一次的时间 5.8824ns   once_time = 0.0058824 us
	uint32_t A_time = (uint32_t)((U_time / 2.0f)/ once_time);
	uint32_t B_time = (uint32_t)((V_time / 2.0f)/ once_time);
	uint32_t C_time = (uint32_t)((W_time / 2.0f)/ once_time);

	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(PWM_PERIOD_CYCLES / 2) - A_time);//TIM 捕获/比较寄存器 1 
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(PWM_PERIOD_CYCLES / 2) - B_time);//TIM 捕获/比较寄存器 2 
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,(PWM_PERIOD_CYCLES / 2) - C_time);//TIM 捕获/比较寄存器 3 
}

