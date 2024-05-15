#ifndef __FOC_H
#define __FOC_H

#include "main.h"

#define FOC_SQRT_3  1.7320508075688772935274463415059f

/**
* @brief  Clarke���任
* @param  
* @retval 
* @attention
*/
void clarke_calc( float Ia, float Ib, float *Ialpha, float *Ibeta);

/**
* @brief  Clarke���任
* @param  
* @retval 
* @attention
*/
void inv_clarke_calc( float Ialpha, float Ibeta, float *Ia, float *Ib);


/**
* @brief  Park���任
* @param  
* @retval 
* @attention  ����Ialpha��Ibeta�Ƕ���ʸ��������Id��Iq��ת��ʸ������ radianת�ӻ��ȡ�
*/
void park_calc( float Ialpha, float Ibeta, float *Id, float *Iq, float radian);


/**
* @brief  Park���任
* @param  
* @retval 
* @attention  ����Valpha��Vbeta�Ƕ���ʸ��������Vd��Vq��ת��ʸ������ radianת�ӻ��ȡ�
*/
void inv_park_calc( float Vd, float Vq, float *Valpha, float *Vbeta, float radian);

#endif
