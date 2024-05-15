/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_PWM.h
  * @version  ：v1.0
  * @date     ：2023年11月29日
  * @brief    ：基于STM32CubeMX HAL库的G4开发板的ADC功能函数,包含电流采样,温度等功能函数.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
#ifndef __BSP_ADC_H
#define __BSP_ADC_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* defines ------------------------------------------------------------------*/
#define Up_Voltage_Divider_Resistors 169.0f	//上分压电阻阻值 169K
#define Down_Voltage_Divider_Resistors 18.0f	//上分压电阻阻值 18K

#define NTC_Down_Voltage_Divider_Resistors 4700.0f	//NTC分压电阻 4.7K

#define Set_Potentiometer_Output_Value_Max 10.0f	//设定电位器输出最大数值
#define Set_Potentiometer_Output_Value_Min 0.0f	//设定电位器输出最小数值

#define	Shunt_Value  0.003f				//采样电阻阻值 3mΩ

#define Calibration_Filter_K  0.001f  //校准滤波因子

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
	uint32_t adc_val_potentiometer;//电位器
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

