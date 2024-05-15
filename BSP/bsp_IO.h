/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��bsp_IO.h
  * @version  ��v1.0
  * @date     ��2021��2��1��
  * @brief    ������STM32CubeMX HAL���XC-DAQ Card��GPIO���ܺ���,����LED,�������ȹ��ܺ���.
  * @attention:	��STM32CubeMX����GPIOʱ,LED��������Ϊ"LED",��������������Ϊ"BUZZER".
	* @bug      :
  ******************************************************************************
  */
#ifndef __BSP_IO_H__
#define __BSP_IO_H__
/* Includes ------------------------------------------------------------------*/
#include "main.h"
typedef enum 
{
  off = 0,
	on,
	breathe,
	twinkle
} Led_State;


/* function ------------------------------------------------------------------*/
void LED_State(uint8_t state);
void LED_Interrupt(void);
void Read_Button(void);
uint8_t Get_Button_Value(void);
void Button_Interrupt(void);
#endif 

