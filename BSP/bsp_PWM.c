/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��bsp_PWM.h
  * @version  ��v1.0
  * @date     ��2023��11��29��
  * @brief    ������STM32CubeMX HAL���G4�������PWM���ܺ���.
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
* @brief  PWM��ʼ��
* @param  
* @retval 
* @attention
*/
void PWM_Start(void)
{
	// ��ʼ����ʱ��
	uint32_t half_load = (PWM_PERIOD_CYCLES / 2) / 2;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,half_load);//TIM ����/�ȽϼĴ��� 1 �趨Ϊ����ֵһ��  50%ռ�ձ�
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,half_load);//TIM ����/�ȽϼĴ��� 2 �趨Ϊ����ֵһ��  50%ռ�ձ�
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,half_load);//TIM ����/�ȽϼĴ��� 3 �趨Ϊ����ֵһ��  50%ռ�ձ�
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    	//������ʱ��1ͨ��1 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);	//������ʱ��1ͨ��1 PWM(����)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    	//������ʱ��1ͨ��2 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  //������ʱ��1ͨ��2 PWM(����)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);			//������ʱ��1ͨ��3 PWM
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  //������ʱ��1ͨ��3 PWM(����)
	
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);    	//������ʱ��1ͨ��4 PWM
	
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);    	//������ʱ��2ͨ��2 PWM
	
	HAL_TIM_Base_Start_IT(&htim17);	//������ʱ��17
}

/**
* @brief  ����ABC���� PWM �ߵ�ƽʱ�� us,���ڴ�������,��Ҫ�����������������
* @param  
* @retval 
* @attention 170MHz = 5.8824ns,����ֵ���ܳ��� 50
*/
void set_pwm_high_leave_time_us(TIM_HandleTypeDef *htim,float U_time,float V_time,float W_time)
{
	float once_time = (1000000.0f / ADV_TIM_CLK_FREQ);//����һ�ε�ʱ�� 5.8824ns   once_time = 0.0058824 us
	uint32_t A_time = (uint32_t)((U_time / 2.0f)/ once_time);
	uint32_t B_time = (uint32_t)((V_time / 2.0f)/ once_time);
	uint32_t C_time = (uint32_t)((W_time / 2.0f)/ once_time);

	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,(PWM_PERIOD_CYCLES / 2) - A_time);//TIM ����/�ȽϼĴ��� 1 
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,(PWM_PERIOD_CYCLES / 2) - B_time);//TIM ����/�ȽϼĴ��� 2 
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,(PWM_PERIOD_CYCLES / 2) - C_time);//TIM ����/�ȽϼĴ��� 3 
}

