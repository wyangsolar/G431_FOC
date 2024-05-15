/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��interrupt_callback.h
  * @version  ��v1.0
  * @date     ��2023��12��01��
  * @brief    ������STM32CubeMX HAL����жϻص�����
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
* @brief 	 ��д��ʱ���жϻص�����
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
* @brief 	 ��дPWM��������жϻص�����������Ƶ��20khz
* @param  
* @retval 
* @attention
*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)     // 20KHz ��ʱ����ͨ��4�ж�����
		{
			HFI_test_time_update();
			//HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		}
	}
	else if(htim == &htim2)//1KHz
	{
		
	}
	else if(htim == &htim17)//û��PWM,�������
	{
		
	}
}	

/**
* @brief 	 ��дADC����ͨ�����жϻص�����,��ʱ��ͨ��2����Ƶ��1khz
* @param  
* @retval 
* @attention ��ѯ����
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1)
	{
		//HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		Look_Up_Tables_resistance_and_temperature(&Motor_ADC);	//���ұ���ѯNTC��ֵ��Ӧ���¶�
	}
	else if(hadc == &hadc2)
	{
		
	}
	
}


/**
* @brief 	 	��дADCע��ͨ�����жϻص���������ʱ��1�����¼�����Ƶ��20khz,����ģʽ�������ģʽ3,���Ϻ����¼���������ᴥ���ж�
*						����RCR= 0 ,�ж�Ƶ��Ϊ 20KHz / 0.5 = 40KHz
*								RCR= 1 ,�ж�Ƶ��Ϊ 20KHz / 1 = 20KHz
*								RCR= 2 ,�ж�Ƶ��Ϊ 20KHz / 1.5 = 13.3333KHz
*           ����RCR�趨Ϊ2,�����ظ����μ����Ż��ж�;
* @param  
* @retval 
* @attention
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) //13.3333KHz
{
	three_phase_current_calc_interrupt(&Motor_ADC , &htim1 , hadc);
}
