/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_IO.h
  * @version  ：v1.0
  * @date     ：2021年2月1日
  * @brief    ：基于STM32CubeMX HAL库的XC-DAQ Card的GPIO功能函数,包含LED,蜂鸣器等功能函数.
  * @attention:	在STM32CubeMX配置GPIO时,LED引脚命名为"LED",蜂鸣器引脚命名为"BUZZER".
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

