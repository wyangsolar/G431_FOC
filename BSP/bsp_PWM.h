/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_IO.h
  * @version  ：v1.0
  * @date     ：2021年2月1日
  * @brief    ：基于STM32CubeMX HAL库的G4开发板的PWM功能函数.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
#ifndef __BSP_PWM_H
#define __BSP_PWM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* function ------------------------------------------------------------------*/
void PWM_Start(void);
void set_pwm_high_leave_time_us(TIM_HandleTypeDef *htim,float U_time,float V_time,float W_time);
#endif 

