/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "svpwm.h"
/* defines ------------------------------------------------------------------*/
svpwm_t svpwm = 
{
	.Ts = 0, //PWMƵ��24KHz  ��λ΢��
	.Udc = 0,
};
/* code ------------------------------------------------------------------*/
/**
* @brief  svpwm������ʼ��
* @param  Ts��SVPWM���ڣ���λus ��Udc��ĸ�ߵ�ѹ
* @retval 
* @attention
*/
void svpwm_init(float ts, float udc)
{
	svpwm.Ts = ts;			//SVPWM����  us
  svpwm.Udc = udc;		//ĸ�ߵ�ѹ 
}

/**
* @brief  svpwm���㣬����һ��svpwm������A B C ��ĸ���ͨ��ʱ�� us
* @param  Ualpha��U����Ubeta��U��   
* @retval 
* @attention  U��= 0 ʱ�� U��= (2/3)Uab   Uab=Uac=�ߵ�ѹ  
*/
void svpwm_calc(float Valpha, float Vbeta, float *Ta, float *Tb, float *Tc)
{
	float U1 = 0;//���������ж�
	float	U2 = 0;//���������ж�
	float U3 = 0;//���������ж�
	 
	uint8_t sector = 0;  //����
	uint8_t N = 0;  
  uint8_t a = 0;//ϵ��a
  uint8_t b = 0;//ϵ��b
  uint8_t c = 0;//ϵ��c
	
	//�����ж�,����U����U�����ж�����
	//�ϳ�ʸ��Uref�ڶ�������ϵ����ͦ���ķ����ֱ�ΪU����U�£���FOC�У�U���� U���ɷ�Park�任�õ�����
	//�ɺϳ�ʸ�����ڸ������ĳ�ֱ�Ҫ����������֪���ɰ����·���ȷ���ϳ�ʸ������������
	
	U1 = Vbeta;
	U2 = Valpha * SVPWM_SQRT3_2 - Vbeta / 2.0f;
	U3 = -Valpha * SVPWM_SQRT3_2 - Vbeta / 2.0f;
	//Uart4_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Ualpha, Ubeta,U1,U2,U3);
	if (U1 > 0) //��Ua>0����a = 1������a=0;
	{
    a = 1;
  } 
	else 
	{
    a = 0;
	}
  if (U2 > 0) //��Ub>0����b = 1������b=0;
	{
    b = 1;
	} 
	else 
	{
    b = 0;
  }
  if (U3 > 0) //��Uc>0����c = 1������c=0;
	{
    c = 1;
  } 
	else 
	{
    c = 0;
  } 
	
/* 
 * �� N = 4c +2b+a
 * Nֵ��������ϵ��Ӧ���£�
 * |------------------------------------------
 * |  N  |  1  |  2  |  3  |  4  |  5  |  6  |
 * |------------------------------------------
 * |�� ��| II  | VI  |  I  | IV  | III |  V  |
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
	
	//���������ʸ������ʱ��
	float t0 = 0;		//0ʸ������ʱ��
	float t1 = 0;		//1ʸ������ʱ��
	float t2 = 0;		//2ʸ������ʱ��
	float t3 = 0;		//3ʸ������ʱ��
	float t4 = 0;		//4ʸ������ʱ��
	float t5 = 0;		//5ʸ������ʱ��
	float t6 = 0;		//6ʸ������ʱ��
	float t7 = 0;		//7ʸ������ʱ��
	
	float K = SVPWM_SQRT3 * svpwm.Ts / svpwm.Udc;    //ϵ��K = sqrt(3) * Ts / Udc 
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

