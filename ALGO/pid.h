/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��PID.c
  * @version  ��v1.0
  * @date     ��2023��12��3��
  * @brief    ��PID�������
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
	float A0;				//�Ƶ�����, A0 = Kp + Ki + Kd
	float A1;				//�Ƶ�����, A1 = -Kp - 2Kd
	float A2;				//�Ƶ�����, A2 = Kd
	float state[3];	//����Ϊ 3 ��״̬����
	float Kp;				//��������
	float Ki;				//��������
	float Kd;				//΢������
}PID;

/* function ------------------------------------------------------------------*/
void pid_init(PID *pid,uint8_t resetStateFlag);
float pid_calc(PID *pid,float in);
#endif
