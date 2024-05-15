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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"
/* defines ------------------------------------------------------------------*/

/* code ------------------------------------------------------------------*/
/**
* @brief PID初始化
* @param pid 指向PID结构的一个实例; 
*        resetStateFlag  标志来重置状态。 0 = 状态无变化 1 = 重置状态,将状态缓冲区中的值清零。
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
* @brief  PID计算
* @param 	pid 是浮点PID控制结构的一个实例 ; in 这一次输入值
* @retval out 输出值
*/
float pid_calc(PID *pid,float in)
{
	float out = 0;

	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
	out =  (pid->state[2]) + (pid->A0 * in) + (pid->A1 * pid->state[0]) + (pid->A2 * pid->state[1]);

	/* 状态更新 */
	pid->state[1] = pid->state[0];//上上次输入值
	pid->state[0] = in;						//上一次输入值
	pid->state[2] = out;					//上一次输出值

	/* 返回值 */
	return (out);
}


