#include "main.h"
#include "tim.h"
#include "test_demo.h"

/*-DSP测试----------------------------------------------------------------------------------------*/
#define DELTA 0.0001f   //误差值 
#define times 1000000	//times:运算次数 1百万次

/*MDK 的标准库（math.h）提供：sin、cos、sinf 和 cosf 等 4 个函数，
带 f 的表示单精度浮点型运算，即 float 型，
而不带 f 的表示双精度浮点型，即 double。*/

void dsp_test_demo(void)
{
	//sin(x)2+cos(x)2=1  测试 
	float angle  =  PI/6;	//angle:起始角度
	float sinx   =	0;
	float cosx   =	0;
	float result =	0; 
	uint32_t i 	 =  0;
	uint32_t time = 0;
	uint32_t err_num = 0;
	Uart_printf(&Uart2,"开始DSP测试......\r\n");
	Uart_printf(&Uart2,"计算三角函数......\r\n");
	time = HAL_GetTick();//记下此时的时间
	for(i=0;i<times;i++) 
  { 
		cosx=cosf(angle);     //不使用 DSP 优化的 sin，cos 函数 
		sinx=sinf(angle); 
		result=sinx*sinx+cosx*cosx;    //计算结果应该等于 1   
		result=fabsf(result-1.0f);    //对比与 1 的差值 
		if(result>DELTA)
		{
			err_num++;
		}
		angle+=0.001f;     //角度自增 
  }
	time =	HAL_GetTick()-time;//计算时间差
	Uart_printf(&Uart2,"无DSP用时:%dms,不一致次数:%d\r\n",time,err_num);
	
	err_num = 0;
	time = HAL_GetTick();//记下此时的时间
	for(i=0;i<times;i++) 
  {         
    cosx=arm_cos_f32(angle);   //使用 DSP 优化的 sin，cos 函数 
    sinx=arm_sin_f32(angle); 
    result=sinx*sinx+cosx*cosx;   //计算结果应该等于 1   
    result=fabsf(result-1.0f);    //对比与 1 的差值 
    if(result>DELTA)
		{
			err_num++;
		}
    angle+=0.001f;     //角度自增 
  }
	time =	HAL_GetTick()-time;//计算时间差
	Uart_printf(&Uart2,"DSP用时:%dms,不一致次数:%d\r\n\r\n",time,err_num);
  Uart_printf(&Uart2,"结束DSP测试......\r\n");
}

/*-PID控制测试-----------------------------------------------------------------------------------------------*/
PID test_pid =
{
	.A0 = 0,
	.A1 = 0,
	.A2 = 0,
	.state = {0},
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
};

float tag_val = 0.0f;//目标值

/*-设定目标值-*/
void set_tag_val(float tag_value)
{
	tag_val = tag_value;
}

/*-pid初始化-*/
void pid_test_demo_init(void)
{
	
	test_pid.Kp = 0.1f;
	test_pid.Ki = 0.001f;
	test_pid.Kd = 0.00001f;
	
	pid_init(&test_pid,1);
	set_tag_val(0);
}

/*-需要循环起来-*/
void pid_test_demo_loop(void)
{
	static float out = 0;
	static float error = 0;
	
	error = tag_val - out;//偏差 = 目标值 - 输出值
	
	out = pid_calc(&test_pid,error);
	Uart_printf(&Uart2,"%.4f\n",out);
}

/*-PLL测试------------------------------------------------------------------------*/
//PLL pll = 
//{
//	.kp = 2000.0f,        	// 比例增益
//	.ki = 3000.0f,        	// 积分增益
//	.phase = 0.0f,     	// 输入的相位  弧度
//	.phase_val = 0.0f, 	// 输出估计的相位  弧度
//	.speed_val = 0.0f, 	// 输出估计的转速  弧度/秒
//	.error = 0.0f,     	// 偏差值
//	.dt = 0.001f,					// 时间步长 1ms
//};
//void pll_test(void)
//{
//	for (uint32_t i = 0; i < 1000000000; ++i) 
//	{
//		// 模拟电机角度，实际中应该根据传感器测量得到
//		pll.phase = DEG_to_RAD_f(i);

//		// 更新PLL
//		updatePLL(&pll);
//		utils_norm_angle_rad(&pll.phase);
//		// 打印结果
//		Uart_printf(&Uart2,"%.4f,%.4f,%.4f\n", pll.phase,pll.phase_val, pll.speed_val);
//  }
//}

/*-HFI测试---------------------------------------------------------------------------------------*/
PID speed_pid = //速度环PID
{
	.A0 = 0,
	.A1 = 0,
	.A2 = 0,
	.state = {0},
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
};

PID Id_pid =		//电流环PID
{
	.A0 = 0,
	.A1 = 0,
	.A2 = 0,
	.state = {0},
	.Kp = 0.000000,
	.Ki = 0.000000,
	.Kd = 0,
}; 

PID Iq_pid =		//电流环PID
{
	.A0 = 0,
	.A1 = 0,
	.A2 = 0,
	.state = {0},
	.Kp = 0.000000,
	.Ki = 0.000000,
	.Kd = 0,
}; 

SecondOrderBandPassFilter BPF_iq = //二阶带通滤波器
{
	.x = 0,  			//输入
	.y = 0,
	.xPrev1 = 0, 	//上一时刻的输入
	.xPrev2 = 0, 	//上上一时刻的输入
	.yPrev1 = 0, 	//上一时刻的输出
	.yPrev2 = 0, 	//上上一时刻的输出,调用这个输出即可
	.fs = 20000.0f,				//滤波器采样频率
	.fc = 800,				//滤波器中心频率
	.B = 10,				//滤波器带宽	10Hz   有放大作用，放大倍数：fc/B
	.a0 = 0,
	.a1 = 0,
	.a2 = 0,
	.b0 = 0,
	.b1 = 0,
	.b2 = 0,
	.s1 = 0, 
	.s2 = 0,
	.s3 = 0,
	.s4 = 0,
	.s5 = 0,
};

// 二阶低通滤波器结构体
SecondOrderLowPassFilter  LPF_iq =
{
	.x = 0.0f,  				//输入
	.y = 0.0f,  				//输出
	.fs = 20000.0f,					//滤波器采样频率
	.fc = 200.0f,					//滤波器截止频率
	.xPrev1 = 0.0f, 		//上一时刻的输入
	.xPrev2 = 0.0f, 		//上上一时刻的输入
	.yPrev1 = 0.0f, 		//上一时刻的输出
	.yPrev2 = 0.0f, 		//上上一时刻的输出
	.a1 = 0.0f,
	.a2 = 0.0f,
	.b0 = 0.0f,
	.b1 = 0.0f,
	.b2 = 0.0f,
};

// 二阶低通滤波器结构体
SecondOrderLowPassFilter  LPF_id =
{
	.x = 0.0f,  				//输入
	.y = 0.0f,  				//输出
	.fs = 20000.0f,					//滤波器采样频率
	.fc = 200.0f,					//滤波器截止频率
	.xPrev1 = 0.0f, 		//上一时刻的输入
	.xPrev2 = 0.0f, 		//上上一时刻的输入
	.yPrev1 = 0.0f, 		//上一时刻的输出
	.yPrev2 = 0.0f, 		//上上一时刻的输出
	.a1 = 0.0f,
	.a2 = 0.0f,
	.b0 = 0.0f,
	.b1 = 0.0f,
	.b2 = 0.0f,
};

// 二阶低通滤波器结构体
SecondOrderLowPassFilter  LPF_modem =
{
	.x = 0.0f,  				//输入
	.y = 0.0f,  				//输出
	.fs = 20000.0f,					//滤波器采样频率
	.fc = 200.0f,					//滤波器截止频率
	.xPrev1 = 0.0f, 		//上一时刻的输入
	.xPrev2 = 0.0f, 		//上上一时刻的输入
	.yPrev1 = 0.0f, 		//上一时刻的输出
	.yPrev2 = 0.0f, 		//上上一时刻的输出
	.a1 = 0.0f,
	.a2 = 0.0f,
	.b0 = 0.0f,
	.b1 = 0.0f,
	.b2 = 0.0f,
};

PLL hfi_pll =
{
	.kp = 8000,        	// 比例增益
	.ki = 80000,        	// 积分增益
	.phase = 0,     	// 输入的相位			弧度
	.phase_val = 0, 	// 输出估计的相位	弧度
	.speed_val = 0, 	// 输出估计的转速	弧度/秒
	.error = 0,     	// 偏差值
	.dt = 0.00005f,					// 时间步长				秒
};


float set_speed = 0.0f;	//设定转速 rpm
float set_id = 0.0f;		//设定id电流
float set_iq = 0.0f;		//设定iq电流

float error_speed = 0.0f;	//速度偏差
float error_id = 0.0f;		//id偏差
float error_iq = 0.0f;		//iq偏差

float vq_pid_out = 0.0f;	//PID输出的vq
float vd_pid_out = 0.0f;	//PID输出的vd

float estimated_rad = 0.0f;		//估计的角度值 弧度
float reality_rad = 0.0f;		//实际角度值
float motor_angle = 0.0f;		//电机角度值  角度
float motor_rad = 0.0f;		//电机弧度值
float rad_error = 0.0f;		//弧度误差

float va = 0.0f;					//由反park输出的vα
float vb = 0.0f;					//由反park输出的vβ

float ia = 0.0f;					//由clarke输出的iα
float ib = 0.0f;					//由clarke输出的iβ

float id = 0.0f;					//由park输出的id
float iq = 0.0f;					//由park输出的iq

float ta = 0.0f;
float tb = 0.0f;
float tc = 0.0f;

float HFI_uin = 0.1f;						//高频注入信号幅值
__IO float HFI_oumiga_dh = 0.0f;//要叠加的高频注入信号

__IO float add_signal = 0.0f;				//合成信号
__IO float time = 0;	//时间累加值

uint8_t hfi_state = 0;
void HFI_test_init(void)
{
	//uint8_t char_buf_2[30];
	float ts = (1000000.0f / PWM_FREQ); //时间步长50微秒 PWM频率20KHz   
	float udc= 12.0f;//母线电压12V
	
	svpwm_init(ts, udc);		//SVPWM初始化赋值
	initSecondOrderLowPassFilter(&LPF_id);//初始化低通滤波器
	initSecondOrderLowPassFilter(&LPF_iq);//初始化低通滤波器
	initSecondOrderLowPassFilter(&LPF_modem);//初始化低通滤波器
	initSecondOrderBandPassFilter(&BPF_iq);//初始化带通滤波器
	pid_init(&speed_pid,1);	//初始化速度环PID
	pid_init(&Id_pid,1);			//初始化电流环PID
	pid_init(&Iq_pid,1);			//初始化电流环PID
	
	hfi_state = 1;
	while(1)
	{
		//卡死在这里，在中断里执行电机计算
		//Uart_printf(&Uart2,"%.5f,%.5f,%.5f,%.5f\n",Motor_ADC.current_meas_phU,Motor_ADC.current_meas_phV,Motor_ADC.current_meas_phW,estimated_rad);
		Uart_printf(&Uart2,"%.5f,%.5f,%.5f,%.5f\n",rad_error,estimated_rad,motor_rad,motor_angle);
//		if(Uart_scanf(&Uart2,char_buf_2,sizeof(char_buf_2)))
//		{
//			if('0' == char_buf_2[0])
//			{
//				hfi_state = 0;
//				break;
//			}
//		}
	}
	
}

void HFI_test_time_update(void)
{
	if(hfi_state)//20KHz,一个周期为2*PI,每次累加PI/10,所以sin频率为20KHz / (10*2) = 1KHz
	{
		HFI_oumiga_dh = HFI_uin * arm_cos_f32(0.8f * time);//生成高频注入信号 800Hz
		add_signal = vd_pid_out + HFI_oumiga_dh;//叠加高频注入信号

		inv_park_calc( add_signal, vq_pid_out, &va, &vb,  estimated_rad);//反park变换
		svpwm_calc(va, vb, &ta, &tb, &tc);	//SVPWM计算三相输出电压
		if(ta > 49.0f)ta = 49.0f; 					//限制最高占空比,不超过50us
		if(tb > 49.0f)tb = 49.0f;						//限制最高占空比,不超过50us
		if(tc > 49.0f)tc = 49.0f; 					//限制最高占空比,不超过50us
		set_pwm_high_leave_time_us(&htim1,ta,tb,tc) ;//三相PWM输出，电机旋转
		
		clarke_calc( Motor_ADC.current_meas_phU, Motor_ADC.current_meas_phV, &ia, &ib);	//clarke变换
		park_calc( ia, ib, &id, &iq, estimated_rad);			//park变换
		
		LPF_id.x = id;//输入到滤波器
		LPF_iq.x = iq;//输入到滤波器
		BPF_iq.x = iq;//输入到滤波器
		updateSecondOrderLowPassFilter(&LPF_id);	//更新滤波器
		updateSecondOrderLowPassFilter(&LPF_iq);	//更新滤波器
		updateSecondOrderBandPassFilter(&BPF_iq);//更新滤波器
		BPF_iq.y = BPF_iq.y / 80.0f;//由于放大了80倍,需要缩小
		
		error_id = set_id - LPF_id.y;//id偏差计算
		error_iq = set_iq - LPF_iq.y;//iq偏差计算
		vd_pid_out = pid_calc(&Id_pid,error_id);//id的PID计算,输出vd
		vq_pid_out = pid_calc(&Iq_pid,error_iq);//iq的PID计算,输出vq
		
		LPF_modem.x = arm_sin_f32(0.8f * time) * BPF_iq.y; //iq解调
		updateSecondOrderLowPassFilter(&LPF_modem);	//更新滤波器
		
		rad_error = LPF_modem.y;//计算角度差

		hfi_pll.error = rad_error;
		utils_norm_angle_rad(&hfi_pll.error);
		// 更新相位,计算弧度
		hfi_pll.phase_val += (hfi_pll.speed_val + hfi_pll.kp * hfi_pll.error) * (hfi_pll.dt);
		// 限制相位在 [-pi,pi) 范围内
		utils_norm_angle_rad(&hfi_pll.phase_val);	
		// 积分项,计算角速度
		hfi_pll.speed_val += (hfi_pll.ki) * (rad_error) * (hfi_pll.dt);
		
		estimated_rad = hfi_pll.phase_val;//角度估计值
		
		motor_rad += hfi_pll.speed_val * (hfi_pll.dt);//角速度积分得到弧度
		
		motor_angle = motor_rad / (2.0f*PI);
		motor_angle = motor_angle * 36.0f;//10对极,每对36度
		utils_norm_angle(&motor_angle);
		//if(time == ((PI/10.0f) * 6.0f)) //此时刚好接近PI / 2
		//{
		//	id
		//}
		
		
		time += (PI/10.0f);
		if(time > 20.0f*PI)//如果大于一个周期
		{
			time -= 20.0f*PI;//就减去一个周期,
		}
	}
}

/*-PWM占空比改变测试------------------------------------------------------------*/
//输入ABC相的高电平时间
void PWM_HighLeaveTime_test(void)
{
	// 初始化底层电机控制
  //PWM_Start();
	
	set_pwm_high_leave_time_us(&htim1,20,49,49) ;//A相 10us  B相 20us  C相 30us

}

///*-FOC函数测试-------------------------------------------------------------------------------*/
//void foc_test(void)
//{
//	float Id = 0;
//	float Iq = 1;
//	float radian = 0;
//	float Ialpha = 0;
//	float Ibeta = 0;
//	for(uint16_t i=0;i<=360;i++)//假设转一圈
//	{
//		radian = (FOC_Pi/180.0f)*i;//角度换算成弧度
//		inv_park_calc( Id, Iq, &Ialpha, &Ibeta,  radian);//反park变换
//		Uart4_printf("%.2f,%.2f,%d\r\n",Ialpha, Ibeta,i);
//		HAL_Delay(10);
//	}
//	
//	Iq = 2;
//	for(uint16_t i=0;i<=360;i++)//假设转一圈
//	{
//		radian = (FOC_Pi/180.0f)*i;//角度换算成弧度
//		inv_park_calc( Id, Iq, &Ialpha, &Ibeta,  radian);//反park变换
//		Uart4_printf("%.2f,%.2f,%d\r\n",Ialpha, Ibeta,i);
//		HAL_Delay(10);
//	}
//}


/*-SVPWM测试------------------------------------------------------------------------*/
void svpwm_test(void)
{
	float Vd = 0;
	float Vq = 0.5;
	float radian = 0;
	float Valpha = 0;
	float Vbeta = 0;
	
	float Ts = (1000000.0f / PWM_FREQ); //PWM频率20KHz   50微秒
	float Udc= 12;//母线电压12V
	float Ta = 0;
	float Tb = 0;
	float Tc = 0;

	svpwm_init(Ts, Udc);//初始化赋值
	
	for(uint16_t i=0;i<360*7;i+=1)//假设转一圈
	{
		radian = (PI / 180.0f)*i;//角度换算成弧度
		inv_park_calc( Vd, Vq, &Valpha, &Vbeta,  radian);//反park变换
		//Uart_printf(&Uart2,"%.5f,%.5f\r\n",Ialpha, Ibeta);
		//svpwm_calc(0.01, 0, &Ta, &Tb, &Tc);//测试A相
		svpwm_calc(Valpha, Vbeta, &Ta, &Tb, &Tc);
		//Uart_printf(&Uart2,"%.5f,%.5f,%.5f\r\n",Ta, Tb,Tc);
		set_pwm_high_leave_time_us(&htim1,Ta,Tb,Tc) ;//A相 1us  B相 2us  C相 3us

		Uart_printf(&Uart2,"%.5f,%.5f,%.5f\n",Motor_ADC.current_meas_phU,Motor_ADC.current_meas_phV,Motor_ADC.current_meas_phW);
		HAL_Delay(1);
	}
	set_pwm_high_leave_time_us(&htim1,0,0,0) ;//A相 1us  B相 2us  C相 3us

}

///*-ADC测试------------------------------------------------------------------------*/
//void adc_test(void)
//{
//	// 初始化底层电机控制
//  init_motor_control();//默认是50%占空比
//	set_pwm_high_leave_time_us(&htim1,12,10,10) ;//A相 1us  B相 2us  C相 3us
//	set_pwm_high_leave_time_us(&htim8,16,10,10) ;//A相 1us  B相 2us  C相 3us*/
//	
//	Motor_adc[0].amp_gain = 40;//放大器增益40V
//	Motor_adc[1].amp_gain = 40;//放大器增益40V
//	start_adc();
//	
//	//svpwm_test();//SVPWM测试
//}
///*-编码器测试------------------------------------------------------------------------*/
//void encoder_test(void)
//{
//	//初始化编码器
//	init_encoder(&encoder[0]);
//	encoder[0].pll_kp=1.0f;
//	encoder[0].pll_ki=0.25f*encoder[0].pll_kp*encoder[0].pll_kp;
//	
//}
///*-相电阻测量------------------------------------------------------------------------*/

//void measure_phase_resistance_test(void)
//{
//	init_motor_control();
//	
//	Motor_adc[0].amp_gain = 40;//放大器增益40V
//	Motor_adc[1].amp_gain = 40;//放大器增益40V
//	
//	start_adc();
//	for(uint16_t i=0;i<360*7;i+=1)//假设转一圈
//	{
//		vbus_calc();//计算电源电压
//		svpwm.Udc = Board_adc.vbus_voltage;
//		
//		queue_voltage_timings(&motors[0], 0.1, 0.0f);
//		Uart4_printf("%.4f,%.4f,%.4f\r\n",Motor_adc[0].current_meas_phA,Motor_adc[0].current_meas_phB,Motor_adc[0].current_meas_phC);
//		HAL_Delay(1);
//	}
//}
