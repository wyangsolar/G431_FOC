#ifndef __FOC_H
#define __FOC_H

#include "main.h"

#define FOC_SQRT_3  1.7320508075688772935274463415059f

/**
* @brief  Clarke正变换
* @param  
* @retval 
* @attention
*/
void clarke_calc( float Ia, float Ib, float *Ialpha, float *Ibeta);

/**
* @brief  Clarke反变换
* @param  
* @retval 
* @attention
*/
void inv_clarke_calc( float Ialpha, float Ibeta, float *Ia, float *Ib);


/**
* @brief  Park正变换
* @param  
* @retval 
* @attention  其中Ialpha和Ibeta是定子矢量分量，Id和Iq是转子矢量分量 radian转子弧度。
*/
void park_calc( float Ialpha, float Ibeta, float *Id, float *Iq, float radian);


/**
* @brief  Park反变换
* @param  
* @retval 
* @attention  其中Valpha和Vbeta是定子矢量分量，Vd和Vq是转子矢量分量 radian转子弧度。
*/
void inv_park_calc( float Vd, float Vq, float *Valpha, float *Vbeta, float radian);

#endif
