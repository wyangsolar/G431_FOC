/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：PID.c
  * @version  ：v1.0
  * @date     ：2023年12月3日
  * @brief    ：PID计算过程
	*							y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
	*							A0 = Kp + Ki + Kd
	*							A1 = (-Kp ) - (2 * Kd )
	*							A2 = Kd
  * @attention:					
	*/
#ifndef __PID_H
#define __PID_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* defines ------------------------------------------------------------------*/
typedef struct
{
	float A0;				//推导增益, A0 = Kp + Ki + Kd
	float A1;				//推导增益, A1 = -Kp - 2Kd
	float A2;				//推导增益, A2 = Kd
	float state[3];	//长度为 3 的状态数组
	float Kp;				//比例增益
	float Ki;				//积分增益
	float Kd;				//微分增益
}PID;

/* function ------------------------------------------------------------------*/
void pid_init(PID *pid,uint8_t resetStateFlag);
float pid_calc(PID *pid,float in);
#endif
