#include "main.h"
#include "foc.h"


/**
* @brief  Clarke���任
* @param  
* @retval 
* @attention
*/
void clarke_calc( float Ia, float Ib, float *Ialpha, float *Ibeta)
{
	*Ialpha = Ia;
	*Ibeta = (Ia + 2.0f * Ib) / FOC_SQRT_3;
}

/**
* @brief  Clarke���任
* @param  
* @retval 
* @attention
*/
void inv_clarke_calc( float Ialpha, float Ibeta, float *Ia, float *Ib)
{
	*Ia = Ialpha;
	*Ib = (FOC_SQRT_3 * Ibeta - Ialpha)/2.0f;
}

/**
* @brief  Park���任
* @param  ����Ialpha��Ibeta�Ƕ���ʸ������,Id��Iq��ת��ʸ������ radianת�ӻ��ȡ�
* @retval 
* @attention  Park�任������ʵ�־�ֹ�ο�ϵ���ƶ��ο�ϵ�ĵ����͵�����ת��,���ƶ���ʸ��������ת�Ӵ�ͨʸ��֮��Ŀռ��ϵ��
*/
void park_calc( float Ialpha, float Ibeta, float *Id, float *Iq, float radian)
{
	*Id = Ialpha * arm_cos_f32(radian) + Ibeta * arm_sin_f32(radian);
  *Iq = -Ialpha * arm_sin_f32(radian) + Ibeta * arm_cos_f32(radian);
}

/**
* @brief  Park���任
* @param  ����Ialpha��Ibeta�Ƕ���ʸ������,Id��Iq��ת��ʸ������ radianת�ӻ��ȡ�
* @retval 
* @attention  �������ͨ��Ť�ط���ת��Ϊ˫����ʸ��
*/
void inv_park_calc( float Vd, float Vq, float *Valpha, float *Vbeta, float radian)
{
	*Valpha = Vd * arm_cos_f32(radian) - Vq * arm_sin_f32(radian);
	*Vbeta = Vd * arm_sin_f32(radian) + Vq * arm_cos_f32(radian);
}
