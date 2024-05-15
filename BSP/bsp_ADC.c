/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��bsp_ADC.h
  * @version  ��v1.0
  * @date     ��2023��11��29��
  * @brief    ������STM32CubeMX HAL���G4�������ADC���ܺ���,������������,�¶ȵȹ��ܺ���.
  * @attention:	
	* @bug      :
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "bsp_ADC.h"
#include "ntc_10K.h"
/* defines ------------------------------------------------------------------*/
ADC Motor_ADC = 
{
	.adc_val_phU = 0,
	.adc_val_phV = 0,
	.adc_val_phW = 0,
	.current_phU = 0,
	.current_phV = 0,
	.current_phW = 0,
	.calib_phU = 0,
	.calib_phV = 0,
	.calib_phW = 0,
	.current_meas_phU = 0,
	.current_meas_phV = 0,
	.current_meas_phW = 0,
	
	.adc_dma_val = {0},
	.adc_val_bus_voltage = 0,
	.adc_val_temperature = 0,
	.adc_val_potentiometer = 0,//��λ��
	.bus_voltage = 0,
	.ntc_resistors = 0,
	.temperature = 0,
	.potentiometer = 0,
};
/* code ------------------------------------------------------------------*/
/**
* @brief  ADC��ʼ��
* @param  
* @retval 
* @attention ��������ʱ��,������ADC,ADC��Ҫ��ʱ����������
*/
void ADC_Start(ADC *motor_adc)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);	//ADC ����ǰ��У׼
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED); //ADC ����ǰ��У׼
	HAL_OPAMP_Start(&hopamp1);//�˷�1����
	HAL_OPAMP_Start(&hopamp2);//�˷�2����
	HAL_OPAMP_Start(&hopamp3);//�˷�3����
	HAL_ADCEx_InjectedStart_IT(&hadc1);//����ADC1ע��ͨ��ת��
	HAL_ADCEx_InjectedStart_IT(&hadc2);//����ADC2ע��ͨ��ת��
	HAL_ADC_Start_DMA(&hadc1,motor_adc->adc_dma_val,3);//����ADC1����ͨ��DMA����
}

/**
* @brief  ��ȡĸ�ߵ�ѹ
* @param  
* @retval 
* @attention
*/
float Get_bus_voltage(ADC *motor_adc)
{
	motor_adc->adc_val_bus_voltage = motor_adc->adc_dma_val[0];
	motor_adc->bus_voltage = ((motor_adc->adc_val_bus_voltage) / 4096.0f ) * 3.3f; //����ADC�ֱ���4096,����3.3V
	motor_adc->bus_voltage = ((motor_adc->bus_voltage)*(Up_Voltage_Divider_Resistors + Down_Voltage_Divider_Resistors)) / Down_Voltage_Divider_Resistors;
	return (motor_adc->bus_voltage);
}

/**
* @brief  ���ж�����ѯ����ֵ����֮��Ӧ���¶�ֵ
* @param  
* @retval 
* @attention
*/
void Look_Up_Tables_resistance_and_temperature(ADC *motor_adc)
{
	static uint8_t row_number_LUT = 0;	//���ұ���б��
	
	motor_adc->adc_val_temperature = motor_adc->adc_dma_val[1];
	motor_adc->ntc_resistors = ((motor_adc->adc_val_temperature) / 4096.0f ) * 3.3f; //����ADC�ֱ���4096,����3.3V
	motor_adc->ntc_resistors = (NTC_Down_Voltage_Divider_Resistors * (3.3f - motor_adc->ntc_resistors)) / motor_adc->ntc_resistors;	//����NTC����ֵ
	
	if(motor_adc->ntc_resistors > NTC_table[row_number_LUT][1])
	{
		motor_adc->temperature = NTC_table[row_number_LUT][0];
		row_number_LUT = 0;
	}
	else
	{
		row_number_LUT ++;
	}
}


/**
* @brief  ��ȡ�����¶�
* @param  
* @retval 
* @attention
*/
float Get_temperature(ADC *motor_adc)
{
	return (motor_adc->temperature);
}

/**
* @brief  ��ȡ��λ����ֵ
* @param  
* @retval 
* @attention
*/
float Get_potentiometer(ADC *motor_adc)
{
	motor_adc->adc_val_potentiometer = motor_adc->adc_dma_val[2];
	return Map(0,4096,Set_Potentiometer_Output_Value_Min,Set_Potentiometer_Output_Value_Max,motor_adc->adc_val_potentiometer);
}

/**
* @brief  ����ADCֵ,���������
* @param  
* @retval �����
* @attention
*/
float phase_current_calc(uint32_t adc_value)
{
	float phase_current = 0;
	phase_current = (3.3f / 4096.0f) * (float)adc_value;	//ADC�ɼ����ķŴ��������ѹ
	phase_current = phase_current / 16.0f;								//��������ĵ�ѹ = �Ŵ�����ѹ / �Ŵ�������
	phase_current = phase_current / Shunt_Value;					//���� = ��������ĵ�ѹ / �����������ֵ
	
	return phase_current;
}

/**
* @brief  ���ж�������������,PWMģʽ2
* @param  ������ʱ�� , ������ADC
* @retval 
* @attention
*/
void three_phase_current_calc_interrupt(ADC *motor_adc , TIM_HandleTypeDef *htim , ADC_HandleTypeDef *hadc)
{
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))//�����1�Ļ�,�������¼���,��ʱ�Ϲܱպ�,�¹ܶϿ�,����0�����µ�У׼ֵ
	{
		//����У׼ֵ
		if(hadc == &hadc1)
		{
			//motor_adc->adc_val_phU = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);		//U���adcֵ
			//motor_adc->current_phU = phase_current_calc(motor_adc->adc_val_phU);								//U�����
			//motor_adc->calib_phU += (motor_adc->current_phU - motor_adc->calib_phU) * Calibration_Filter_K;//�Բ�ֵ������
			
			motor_adc->adc_val_phW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);		//W���adcֵ
			motor_adc->current_phW = phase_current_calc(motor_adc->adc_val_phW);								//W�����
			motor_adc->calib_phW += (motor_adc->current_phW - motor_adc->calib_phW) * Calibration_Filter_K;//�Բ�ֵ������
		}
		else if(hadc == &hadc2)
		{
			motor_adc->adc_val_phV = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);		//V���adcֵ
			motor_adc->current_phV = phase_current_calc(motor_adc->adc_val_phV);								//V�����
			motor_adc->calib_phV += (motor_adc->current_phV - motor_adc->calib_phV) * Calibration_Filter_K;//�Բ�ֵ������
		}
	}
	else //��ʱ�����ϼ���,��ʱ�ϹܶϿ�,�¹ܱպ�,�������,��ʱ�����������
	{
		//�����������
		if(hadc == &hadc1)
		{
			//motor_adc->adc_val_phU = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);		//U���adcֵ
			//motor_adc->current_phU = phase_current_calc(motor_adc->adc_val_phU);								//U�����
			//motor_adc->current_meas_phU = motor_adc->current_phU - motor_adc->calib_phU;
			
			motor_adc->adc_val_phW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);		//W���adcֵ
			motor_adc->current_phW = phase_current_calc(motor_adc->adc_val_phW);								//W�����
			motor_adc->current_meas_phW = (motor_adc->current_phW) - (motor_adc->calib_phW);
		}
		else if(hadc == &hadc2)
		{
			motor_adc->adc_val_phV = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);		//V���adcֵ
			motor_adc->current_phV = phase_current_calc(motor_adc->adc_val_phV);								//V�����
			motor_adc->current_meas_phV = (motor_adc->current_phV) - (motor_adc->calib_phV);
		}
		
		motor_adc->current_meas_phU = -(motor_adc->current_meas_phW) - (motor_adc->current_meas_phV);
	}
}


