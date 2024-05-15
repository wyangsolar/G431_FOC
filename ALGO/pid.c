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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"
/* defines ------------------------------------------------------------------*/

/* code ------------------------------------------------------------------*/
/**
* @brief PID��ʼ��
* @param pid ָ��PID�ṹ��һ��ʵ��; 
*        resetStateFlag  ��־������״̬�� 0 = ״̬�ޱ仯 1 = ����״̬,��״̬�������е�ֵ���㡣
* @retval 
*/
void pid_init(PID *pid,uint8_t resetStateFlag)
{
	pid->A0 = (pid->Kp) + (pid->Ki) + (pid->Kd);
	pid->A1 = - (pid->Kp) - (2.0f * (pid->Kd));
	pid->A2 = (pid->Kd);
	
	if(resetStateFlag == 1)
	{
		pid->state[0] = 0;
		pid->state[1] = 0;
		pid->state[2] = 0;
	}
	else
	{
		
	}
	
}

/**
* @brief  PID����
* @param 	pid �Ǹ���PID���ƽṹ��һ��ʵ�� ; in ��һ������ֵ
* @retval out ���ֵ
*/
float pid_calc(PID *pid,float in)
{
	float out = 0;

	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
	out =  (pid->state[2]) + (pid->A0 * in) + (pid->A1 * pid->state[0]) + (pid->A2 * pid->state[1]);

	/* ״̬���� */
	pid->state[1] = pid->state[0];//���ϴ�����ֵ
	pid->state[0] = in;						//��һ������ֵ
	pid->state[2] = out;					//��һ�����ֵ

	/* ����ֵ */
	return (out);
}


