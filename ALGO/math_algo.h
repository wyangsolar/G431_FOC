#ifndef __MATH_ALGO_H
#define __MATH_ALGO_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* defines ------------------------------------------------------------------*/
// 弧度/度和转速/弧度/秒的便捷转换
#define DEG_to_RAD_f(deg) ((deg) * (float)(PI / 180.0f))														//度->弧度
#define RAD_to_DEG_f(rad) ((rad) * (float)(180.0f / PI))														//弧度->度
#define RPM_to_RADPS_f(rpm) ((rpm) * (float)((2.0f * PI) / 60.0f))									//转速->弧度/秒
#define RADPS_to_RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0f / (2.0f * PI)))	//弧度/秒->转速


// 一阶低通滤波器结构体
typedef struct 
{
	__IO float x;  				//输入
	__IO float y;      		//输出
	float alpha;			//滤波器系数,可以控制滤波器的截止频率,初始化时由fs fc	计算得出
	float fs;					//滤波器采样频率
	float fc;					//滤波器截止频率	
} LowPassFilter;

// 二阶低通滤波器结构体
typedef struct 
{
	__IO float x;  				//输入
	__IO float y;  				//输出
	__IO float xPrev1; 		//上一时刻的输入
	__IO float xPrev2; 		//上上一时刻的输入
	__IO float yPrev1; 		//上一时刻的输出
	__IO float yPrev2; 		//上上一时刻的输出
	float fs;					//滤波器采样频率
	float fc;					//滤波器截止频率
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;

} SecondOrderLowPassFilter;

// 二阶带通滤波器结构体
typedef struct 
{
	__IO float x;  			//输入
	__IO float y;  			//输出
	__IO float xPrev1; 	//上一时刻的输入
	__IO float xPrev2; 	//上上一时刻的输入
	__IO float yPrev1; 	//上一时刻的输出
	__IO float yPrev2; 	//上上一时刻的输出
	float fs;				//滤波器采样频率
	float fc;				//滤波器中心频率
	float B;				//滤波器带宽 ，有放大作用，放大倍数：fc/B
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float s1; 
	float s2;
	float s3;
	float s4;
	float s5;

} SecondOrderBandPassFilter;


// 定义锁相环结构体
typedef struct 
{
	float kp;        	// 比例增益
	float ki;        	// 积分增益
	float phase;     	// 输入的相位			弧度
	float phase_val; 	// 输出估计的相位	弧度
	float speed_val; 	// 输出估计的转速	弧度/秒
	float error;     	// 偏差值
	float dt;					// 时间步长				秒
} PLL;


/* function ------------------------------------------------------------------*/
float Map(float x1,float x2,float x3,float x4,float in_val);

void initLowPassFilter(LowPassFilter *filter);
void updateLowPassFilter(LowPassFilter *filter);

void initSecondOrderLowPassFilter(SecondOrderLowPassFilter *filter);
void updateSecondOrderLowPassFilter(SecondOrderLowPassFilter *filter);

void initSecondOrderBandPassFilter(SecondOrderBandPassFilter *filter);
void updateSecondOrderBandPassFilter(SecondOrderBandPassFilter *filter);

void updatePLL(PLL *pll);

void utils_norm_angle_rad(float *angle);
void utils_norm_angle(float *angle);
#endif
