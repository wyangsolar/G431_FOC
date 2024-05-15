/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��bsp_IO.h
  * @version  ��v1.0
  * @date     ��2021��2��1��
  * @brief    ������STM32CubeMX HAL���GPIO���ܺ���,����LED,�����ȹ��ܺ���.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_IO.h"
/* defines ------------------------------------------------------------------*/
__IO static uint16_t led_toggle_threshold = 0;	//LED��ת��ֵ
__IO static uint8_t led_interrupt_mode = 0;			//LED�ж�ģʽ

__IO static uint8_t button_state = 0;	//����״̬
/* code ------------------------------------------------------------------*/
/**
* @brief      LED״̬
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
* @brief      LED ��ʱ���ж�ִ�г���,�Ƽ�10KHz�ж�Ƶ��
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
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);//Ϩ��
			}
			else
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);//����
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
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);//Ϩ��
			}
			else
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);//����
			}
			times++;
			if(times > 100)
			{
				times = 0;
				if(led_flag == 0)
				{
					led_toggle_threshold++;//��Ϩ��
				}
				else if(led_flag == 1)
				{
					led_toggle_threshold--;//�𽥵���
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
* @brief 	   ��ȡ����״̬
* @param  
* @retval 
* @attention �����ж�������
*/
void Read_Button(void)
{
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
	{
		button_state = off;//�����ɿ�
	}
	else
	{
		button_state = on;//��������
	}
}

/**
* @brief 	   ���ذ���״̬
* @param  
* @retval 
* @attention 
*/
uint8_t Get_Button_Value(void)
{
	return button_state;
}
	

/**
* @brief 			���� ��ʱ���ж�ִ�г���,�Ƽ�10KHz�ж�Ƶ��
* @param      
* @retval     
* @attention  
*/
void Button_Interrupt(void)
{
	__IO static uint16_t times = 0;
	if(200 <= times) //ÿ��200�ξ�����ȡһ�°���״̬,50Hz�ļ��Ƶ��,�����������
	{
		Read_Button();
		times = 0;
	}
	times ++;
}
