/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "svpwm.h"
/* defines ------------------------------------------------------------------*/
svpwm_t svpwm = 
{
	.Ts = 0, //PWM频率24KHz  单位微秒
	.Udc = 0,
};
/* code ------------------------------------------------------------------*/
/**
* @brief  svpwm参数初始化
* @param  Ts：SVPWM周期，单位us ；Udc：母线电压
* @retval 
* @attention
*/
void svpwm_init(float ts, float udc)
{
	svpwm.Ts = ts;			//SVPWM周期  us
  svpwm.Udc = udc;		//母线电压 
}

/**
* @brief  svpwm计算，返回一个svpwm周期内A B C 相的各自通电时间 us
* @param  Ualpha：Uα；Ubeta：Uβ   
* @retval 
* @attention  Uβ= 0 时， Uα= (2/3)Uab   Uab=Uac=线电压  
*/
void svpwm_calc(float Valpha, float Vbeta, float *Ta, float *Tb, float *Tc)
{
	float U1 = 0;//用于扇区判断
	float	U2 = 0;//用于扇区判断
	float U3 = 0;//用于扇区判断
	 
	uint8_t sector = 0;  //扇区
	uint8_t N = 0;  
  uint8_t a = 0;//系数a
  uint8_t b = 0;//系数b
  uint8_t c = 0;//系数c
	
	//扇区判断,利用Uα和Uβ来判断扇区
	//合成矢量Uref在二相坐标系α轴和β轴的分量分别为Uα、Uβ（在FOC中，Uα和 Uβ由反Park变换得到），
	//由合成矢量落在各扇区的充分必要条件分析可知，可按如下方法确定合成矢量所属扇区：
	
	U1 = Vbeta;
	U2 = Valpha * SVPWM_SQRT3_2 - Vbeta / 2.0f;
	U3 = -Valpha * SVPWM_SQRT3_2 - Vbeta / 2.0f;
	//Uart4_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Ualpha, Ubeta,U1,U2,U3);
	if (U1 > 0) //若Ua>0，则a = 1，否则a=0;
	{
    a = 1;
  } 
	else 
	{
    a = 0;
	}
  if (U2 > 0) //若Ub>0，则b = 1，否则b=0;
	{
    b = 1;
	} 
	else 
	{
    b = 0;
  }
  if (U3 > 0) //若Uc>0，则c = 1，否则c=0;
	{
    c = 1;
  } 
	else 
	{
    c = 0;
  } 
	
/* 
 * 令 N = 4c +2b+a
 * N值与扇区关系对应如下：
 * |------------------------------------------
 * |  N  |  1  |  2  |  3  |  4  |  5  |  6  |
 * |------------------------------------------
 * |扇 区| II  | VI  |  I  | IV  | III |  V  |
 * |------------------------------------------
 * |                                         |
 * |------------------------------------------
 */
	N = 4*c + 2*b + a;
	switch (N) 
	{
		case 3:
			sector = 1;
			break;
		case 1:
			sector = 2;
			break;
		case 5:
			sector = 3;
			break;
		case 4:
			sector = 4;
			break;
		case 6:
			sector = 5;
			break;
		case 2:
			sector = 6;
			break;
	}
	
	//计算各相邻矢量作用时间
	float t0 = 0;		//0矢量作用时长
	float t1 = 0;		//1矢量作用时长
	float t2 = 0;		//2矢量作用时长
	float t3 = 0;		//3矢量作用时长
	float t4 = 0;		//4矢量作用时长
	float t5 = 0;		//5矢量作用时长
	float t6 = 0;		//6矢量作用时长
	float t7 = 0;		//7矢量作用时长
	
	float K = SVPWM_SQRT3 * svpwm.Ts / svpwm.Udc;    //系数K = sqrt(3) * Ts / Udc 
  switch (sector) 
	{
		case 1:
			t4 = K * U2;
			t6 = K * U1;
			t0 = t7 = (svpwm.Ts - t4 - t6) / 2.0f;
		
			*Ta = t4 + t6 + t7;
			*Tb = t6 + t7;
			*Tc = t7;
			break;
		case 2:
			t2 = -K * U2;
			t6 = -K * U3;
			t0 = t7 = (svpwm.Ts - t2 - t6) / 2.0f;
		
			*Ta = t6 + t7;
			*Tb = t2 + t6 + t7;
			*Tc = t7;
			break;
		case 3:
			t2 = K * U1;
			t3 = K * U3;
			t0 = t7 = (svpwm.Ts - t2 - t3) / 2.0f;
		
			*Ta = t7;
			*Tb = t2 + t3 + t7;
			*Tc = t3 + t7;
			break;
		case 4:
			t1 = -K * U1;
			t3 = -K * U2;
			t0 = t7 = (svpwm.Ts - t1 - t3) / 2.0f;
		
			*Ta = t7;
			*Tb = t3 + t7;
			*Tc = t1 + t3 + t7;
		break;
		case 5:
			t1 = K * U3;
			t5 = K * U2;
			t0 = t7 = (svpwm.Ts - t1 - t5) / 2.0f;
		
			*Ta = t5 + t7;
			*Tb = t7;
			*Tc = t1 + t5 + t7;
			break;
		case 6:
			t4 = -K * U3;
			t5 = -K * U1;
			t0 = t7 = (svpwm.Ts - t4 - t5) / 2.0f;
		
			*Ta = t4 + t5 + t7;
			*Tb = t7;
			*Tc = t5 + t7;
			break;
		default:
			break;
	} 
}

