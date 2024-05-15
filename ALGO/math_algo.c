
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math_algo.h"
/* defines ------------------------------------------------------------------*/

/* code ------------------------------------------------------------------*/
/**
* @brief 数值映射
* @param 输入范围(x1,x2)，和另一个要映射输出的范围(x3,x4)，再输入实际值
* @retval 
*/
float Map(float x1,float x2,float x3,float x4,float in_val)
{
	return (((x4-x3)*(in_val-x1))/(x2-x1))+x3;
}


/**
  * @brief  一阶低通滤波
  * @note   计算公式：α= (2*pi*fc) / (fs+(2*pi*fc))
	*										y[n]= α*x[n] + (1-α)*y[n-1]
  * @param  
  * @retval 
  */
// 初始化一阶低通滤波器
void initLowPassFilter(LowPassFilter *filter) 
{
	filter->alpha = (2.0f * PI * (filter->fc)) / ((2.0f * PI * (filter->fc)) + (filter->fs));
}
// 更新一阶低通滤波器
void updateLowPassFilter(LowPassFilter *filter)
{
	filter->y = ((filter->alpha) * (filter->x)) + ((1.0f - (filter->alpha)) * (filter->y));
}

/**
  * @brief  二阶低通滤波
  * @note   
  * @param  
  * @retval 
  */
// 初始化二阶低通滤波器
void initSecondOrderLowPassFilter(SecondOrderLowPassFilter *filter) 
{
	float ohm = tan(PI * filter->fc / filter->fs);
	float c = 1.0f + 1.414f * ohm  + ohm * ohm;
	
	filter->b0 = ohm * ohm /c;
	filter->b1 = 2.0f * filter->b0;
	filter->b2 = filter->b0;

	filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	filter->a2 = (1.0f - 1.4142135623730950488016887242097f * ohm + ohm * ohm) / c;
}

// 更新二阶低通滤波器
void updateSecondOrderLowPassFilter(SecondOrderLowPassFilter *filter) 
{
	
	filter->y = filter->b0 * filter->x + filter->b1 * filter->xPrev1 + filter->b2 * filter->xPrev2 - filter->a1 * filter->yPrev1 - filter->a2 * filter->yPrev2;

	// 更新历史数据

	filter->xPrev2 = filter->xPrev1;
	filter->xPrev1 = filter->x;
	filter->yPrev2 = filter->yPrev1;
	filter->yPrev1 = filter->y;
}

/**
  * @brief  二阶带通滤波(巴特沃斯 Butterworth)
  * @note   
  * @param  
  * @retval 
  */
void initSecondOrderBandPassFilter(SecondOrderBandPassFilter *filter) 
{
	float w0 = 2.0f * PI * filter->fc / filter->fs;
  float Q = filter->fc / filter->B;
  float alpha = sin(w0) / (2.0f * Q);
    
  filter->a0 = 1.0f + alpha;
  filter->a1 = -2.0f * cos(w0);
  filter->a2 = 1.0f - alpha;

  filter->b0 = (1.0f - cos(w0)) / 2.0f;
  filter->b1 = 1.0f - cos(w0);
  filter->b2 = (1.0f - cos(w0)) / 2.0f;
	
	filter->s1 = (filter->b0 / filter->a0);
	filter->s2 = (filter->b1 / filter->a0);
	filter->s3 = (filter->b2 / filter->a0);
	filter->s4 = (filter->a1 / filter->a0);
	filter->s5 = (filter->a2 / filter->a0);

}

void updateSecondOrderBandPassFilter(SecondOrderBandPassFilter *filter) 
{

	filter->y = filter->s1 * filter->x + filter->s2 * filter->xPrev1 + filter->s3 * filter->xPrev2 - filter->s4 * filter->yPrev1 - filter->s5 * filter->yPrev2;

	// 更新状态变量
	filter->xPrev2 = filter->xPrev1;
	filter->xPrev1 = filter->x;
	filter->yPrev2 = filter->yPrev1;
	filter->yPrev1 = filter->y;

}

/**
  * @brief  PLL 锁相环,角度和速度估算
  * @note   输入传感器rad,输出估计的rad和估计的rad/s
  * @param  
  * @retval 
  */
void updatePLL(PLL *pll) 
{
	// 计算误差
	pll->error = (pll->phase) - (pll->phase_val);//误差 = 输入 - 输出
	utils_norm_angle_rad(&pll->error);
	// 更新相位,计算弧度
	pll->phase_val += (pll->speed_val + pll->kp * pll->error) * (pll->dt);
	// 限制相位在 [-pi,pi) 范围内
	utils_norm_angle_rad(&pll->phase_val);
	// 积分项,计算角速度
	pll->speed_val += (pll->ki) * (pll->error) * (pll->dt);
}



/*--其他数学计算函数----------------------------------------------------*/
/**
 * 确保 -pi <= angle < pi,
 *
 * @param angle
 * 要归一化的角度，单位为弧度。
 * 警告：不要使用太大的角度。
 */
void utils_norm_angle_rad(float *angle)
{
	while (*angle < -PI) { *angle += 2.0f * PI; }
	while (*angle >= PI) { *angle -= 2.0f * PI; }
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle)
{
	*angle = fmodf(*angle, 360.0f);

	if (*angle < 0.0f)
	{
		*angle += 360.0f;
	}
}
