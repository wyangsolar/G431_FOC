/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��bsp_PWM.h
  * @version  ��v1.0
  * @date     ��2023��11��29��
  * @brief    ������STM32CubeMX HAL���G4�������ADC���ܺ���,������������,�¶ȵȹ��ܺ���.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
#ifndef __BSP_ADC_H
#define __BSP_ADC_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* defines ------------------------------------------------------------------*/
#define Up_Voltage_Divider_Resistors 169.0f	//�Ϸ�ѹ������ֵ 169K
#define Down_Voltage_Divider_Resistors 18.0f	//�Ϸ�ѹ������ֵ 18K

#define NTC_Down_Voltage_Divider_Resistors 4700.0f	//NTC��ѹ���� 4.7K

#define Set_Potentiometer_Output_Value_Max 10.0f	//�趨��λ����������ֵ
#define Set_Potentiometer_Output_Value_Min 0.0f	//�趨��λ�������С��ֵ

#define	Shunt_Value  0.003f				//����������ֵ 3m��

#define Calibration_Filter_K  0.001f  //У׼�˲�����

typedef struct
{
	__IO uint32_t adc_val_phU;
	__IO uint32_t adc_val_phV;
	__IO uint32_t adc_val_phW;
	__IO float current_phU;
	__IO float current_phV;
	__IO float current_phW;
	__IO float calib_phU;
	__IO float calib_phV;
	__IO float calib_phW;
	__IO float current_meas_phU;
	__IO float current_meas_phV;
	__IO float current_meas_phW;
	
	uint32_t adc_dma_val[3];
	uint32_t adc_val_bus_voltage;
	uint32_t adc_val_temperature;
	uint32_t adc_val_potentiometer;//��λ��
	float bus_voltage;
	float ntc_resistors;
	float temperature;
	float potentiometer;
}ADC;

extern ADC Motor_ADC;
/* function ------------------------------------------------------------------*/
void ADC_Start(ADC *motor_adc);
float Get_bus_voltage(ADC *motor_adc);
void Look_Up_Tables_resistance_and_temperature(ADC *motor_adc);
float Get_temperature(ADC *motor_adc);
float Get_potentiometer(ADC *motor_adc);
float phase_current_calc(uint32_t adc_value);
void three_phase_current_calc_interrupt(ADC *motor_adc , TIM_HandleTypeDef *htim , ADC_HandleTypeDef *hadc);
#endif 

