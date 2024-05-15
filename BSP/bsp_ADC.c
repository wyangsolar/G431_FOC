/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：bsp_ADC.h
  * @version  ：v1.0
  * @date     ：2023年11月29日
  * @brief    ：基于STM32CubeMX HAL库的G4开发板的ADC功能函数,包含电流采样,温度等功能函数.
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
	.adc_val_potentiometer = 0,//电位器
	.bus_voltage = 0,
	.ntc_resistors = 0,
	.temperature = 0,
	.potentiometer = 0,
};
/* code ------------------------------------------------------------------*/
/**
* @brief  ADC初始化
* @param  
* @retval 
* @attention 先启动定时器,再启动ADC,ADC需要定时器触发采样
*/
void ADC_Start(ADC *motor_adc)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);	//ADC 启动前自校准
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED); //ADC 启动前自校准
	HAL_OPAMP_Start(&hopamp1);//运放1启动
	HAL_OPAMP_Start(&hopamp2);//运放2启动
	HAL_OPAMP_Start(&hopamp3);//运放3启动
	HAL_ADCEx_InjectedStart_IT(&hadc1);//启动ADC1注入通道转换
	HAL_ADCEx_InjectedStart_IT(&hadc2);//启动ADC2注入通道转换
	HAL_ADC_Start_DMA(&hadc1,motor_adc->adc_dma_val,3);//用于ADC1规则通道DMA传输
}

/**
* @brief  获取母线电压
* @param  
* @retval 
* @attention
*/
float Get_bus_voltage(ADC *motor_adc)
{
	motor_adc->adc_val_bus_voltage = motor_adc->adc_dma_val[0];
	motor_adc->bus_voltage = ((motor_adc->adc_val_bus_voltage) / 4096.0f ) * 3.3f; //除以ADC分辨率4096,乘以3.3V
	motor_adc->bus_voltage = ((motor_adc->bus_voltage)*(Up_Voltage_Divider_Resistors + Down_Voltage_Divider_Resistors)) / Down_Voltage_Divider_Resistors;
	return (motor_adc->bus_voltage);
}

/**
* @brief  在中断里轮询电阻值和与之对应的温度值
* @param  
* @retval 
* @attention
*/
void Look_Up_Tables_resistance_and_temperature(ADC *motor_adc)
{
	static uint8_t row_number_LUT = 0;	//查找表的行编号
	
	motor_adc->adc_val_temperature = motor_adc->adc_dma_val[1];
	motor_adc->ntc_resistors = ((motor_adc->adc_val_temperature) / 4096.0f ) * 3.3f; //除以ADC分辨率4096,乘以3.3V
	motor_adc->ntc_resistors = (NTC_Down_Voltage_Divider_Resistors * (3.3f - motor_adc->ntc_resistors)) / motor_adc->ntc_resistors;	//计算NTC电阻值
	
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
* @brief  获取主板温度
* @param  
* @retval 
* @attention
*/
float Get_temperature(ADC *motor_adc)
{
	return (motor_adc->temperature);
}

/**
* @brief  获取电位器数值
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
* @brief  输入ADC值,计算相电流
* @param  
* @retval 相电流
* @attention
*/
float phase_current_calc(uint32_t adc_value)
{
	float phase_current = 0;
	phase_current = (3.3f / 4096.0f) * (float)adc_value;	//ADC采集到的放大器输出电压
	phase_current = phase_current / 16.0f;								//采样电阻的电压 = 放大器电压 / 放大器增益
	phase_current = phase_current / Shunt_Value;					//电流 = 采样电阻的电压 / 采样电阻的阻值
	
	return phase_current;
}

/**
* @brief  在中断里计算三相电流,PWM模式2
* @param  触发定时器 , 被触发ADC
* @retval 
* @attention
*/
void three_phase_current_calc_interrupt(ADC *motor_adc , TIM_HandleTypeDef *htim , ADC_HandleTypeDef *hadc)
{
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))//如果是1的话,就是向下计数,此时上管闭合,下管断开,计算0电流下的校准值
	{
		//计算校准值
		if(hadc == &hadc1)
		{
			//motor_adc->adc_val_phU = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);		//U相的adc值
			//motor_adc->current_phU = phase_current_calc(motor_adc->adc_val_phU);								//U相电流
			//motor_adc->calib_phU += (motor_adc->current_phU - motor_adc->calib_phU) * Calibration_Filter_K;//对差值做积分
			
			motor_adc->adc_val_phW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);		//W相的adc值
			motor_adc->current_phW = phase_current_calc(motor_adc->adc_val_phW);								//W相电流
			motor_adc->calib_phW += (motor_adc->current_phW - motor_adc->calib_phW) * Calibration_Filter_K;//对差值做积分
		}
		else if(hadc == &hadc2)
		{
			motor_adc->adc_val_phV = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);		//V相的adc值
			motor_adc->current_phV = phase_current_calc(motor_adc->adc_val_phV);								//V相电流
			motor_adc->calib_phV += (motor_adc->current_phV - motor_adc->calib_phV) * Calibration_Filter_K;//对差值做积分
		}
	}
	else //定时器向上计数,此时上管断开,下管闭合,电机续流,此时测量电机电流
	{
		//测量电机电流
		if(hadc == &hadc1)
		{
			//motor_adc->adc_val_phU = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);		//U相的adc值
			//motor_adc->current_phU = phase_current_calc(motor_adc->adc_val_phU);								//U相电流
			//motor_adc->current_meas_phU = motor_adc->current_phU - motor_adc->calib_phU;
			
			motor_adc->adc_val_phW = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);		//W相的adc值
			motor_adc->current_phW = phase_current_calc(motor_adc->adc_val_phW);								//W相电流
			motor_adc->current_meas_phW = (motor_adc->current_phW) - (motor_adc->calib_phW);
		}
		else if(hadc == &hadc2)
		{
			motor_adc->adc_val_phV = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);		//V相的adc值
			motor_adc->current_phV = phase_current_calc(motor_adc->adc_val_phV);								//V相电流
			motor_adc->current_meas_phV = (motor_adc->current_phV) - (motor_adc->calib_phV);
		}
		
		motor_adc->current_meas_phU = -(motor_adc->current_meas_phW) - (motor_adc->current_meas_phV);
	}
}


