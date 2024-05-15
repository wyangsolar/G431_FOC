#ifndef __MATH_ALGO_H
#define __MATH_ALGO_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* defines ------------------------------------------------------------------*/
// ����/�Ⱥ�ת��/����/��ı��ת��
#define DEG_to_RAD_f(deg) ((deg) * (float)(PI / 180.0f))														//��->����
#define RAD_to_DEG_f(rad) ((rad) * (float)(180.0f / PI))														//����->��
#define RPM_to_RADPS_f(rpm) ((rpm) * (float)((2.0f * PI) / 60.0f))									//ת��->����/��
#define RADPS_to_RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0f / (2.0f * PI)))	//����/��->ת��


// һ�׵�ͨ�˲����ṹ��
typedef struct 
{
	__IO float x;  				//����
	__IO float y;      		//���
	float alpha;			//�˲���ϵ��,���Կ����˲����Ľ�ֹƵ��,��ʼ��ʱ��fs fc	����ó�
	float fs;					//�˲�������Ƶ��
	float fc;					//�˲�����ֹƵ��	
} LowPassFilter;

// ���׵�ͨ�˲����ṹ��
typedef struct 
{
	__IO float x;  				//����
	__IO float y;  				//���
	__IO float xPrev1; 		//��һʱ�̵�����
	__IO float xPrev2; 		//����һʱ�̵�����
	__IO float yPrev1; 		//��һʱ�̵����
	__IO float yPrev2; 		//����һʱ�̵����
	float fs;					//�˲�������Ƶ��
	float fc;					//�˲�����ֹƵ��
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;

} SecondOrderLowPassFilter;

// ���״�ͨ�˲����ṹ��
typedef struct 
{
	__IO float x;  			//����
	__IO float y;  			//���
	__IO float xPrev1; 	//��һʱ�̵�����
	__IO float xPrev2; 	//����һʱ�̵�����
	__IO float yPrev1; 	//��һʱ�̵����
	__IO float yPrev2; 	//����һʱ�̵����
	float fs;				//�˲�������Ƶ��
	float fc;				//�˲�������Ƶ��
	float B;				//�˲������� ���зŴ����ã��Ŵ�����fc/B
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


// �������໷�ṹ��
typedef struct 
{
	float kp;        	// ��������
	float ki;        	// ��������
	float phase;     	// �������λ			����
	float phase_val; 	// ������Ƶ���λ	����
	float speed_val; 	// ������Ƶ�ת��	����/��
	float error;     	// ƫ��ֵ
	float dt;					// ʱ�䲽��				��
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
