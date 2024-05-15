#include "main.h"
#include "foc.h"


/**
* @brief  Clarke正变换
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
* @brief  Clarke反变换
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
* @brief  Park正变换
* @param  其中Ialpha和Ibeta是定子矢量分量,Id和Iq是转子矢量分量 radian转子弧度。
* @retval 
* @attention  Park变换可用于实现静止参考系向移动参考系的电流和电流的转换,控制定子矢量电流与转子磁通矢量之间的空间关系。
*/
void park_calc( float Ialpha, float Ibeta, float *Id, float *Iq, float radian)
{
	*Id = Ialpha * arm_cos_f32(radian) + Ibeta * arm_sin_f32(radian);
  *Iq = -Ialpha * arm_sin_f32(radian) + Ibeta * arm_cos_f32(radian);
}

/**
* @brief  Park反变换
* @param  其中Ialpha和Ibeta是定子矢量分量,Id和Iq是转子矢量分量 radian转子弧度。
* @retval 
* @attention  将输入磁通和扭矩分量转换为双坐标矢量
*/
void inv_park_calc( float Vd, float Vq, float *Valpha, float *Vbeta, float radian)
{
	*Valpha = Vd * arm_cos_f32(radian) - Vq * arm_sin_f32(radian);
	*Vbeta = Vd * arm_sin_f32(radian) + Vq * arm_cos_f32(radian);
}
