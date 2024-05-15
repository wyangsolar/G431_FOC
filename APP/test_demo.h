#ifndef __TEST_DEMO_H
#define __TEST_DEMO_H

#include "main.h"


void dsp_test_demo(void);

void pid_test_demo_init(void);
void set_tag_val(float tag_value);
void pid_test_demo_loop(void);

void encoder_test_demo(void);

/*-pll����------------------------------------------------------------------------*/
void pll_test(void);
/*-HFI����------------------------------------------------------------------------*/
void HFI_test_init(void);
void HFI_test_time_update(void);
//����ABC��ĸߵ�ƽʱ��
void PWM_HighLeaveTime_test(void);

/*-FOC��������-------------------------------------------------------------------------------*/
void foc_test(void);

/*-SVPWM����------------------------------------------------------------------------*/
void svpwm_test(void);

/*-ADC����------------------------------------------------------------------------*/
void adc_test(void);

/*-����������------------------------------------------------------------------------*/
void encoder_test(void);

/*-��������------------------------------------------------------------------------*/
void measure_phase_resistance_test(void);
#endif
