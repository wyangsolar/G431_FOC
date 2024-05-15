/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��DMA_UART.h
  * @version  ��v1.2
  * @date     ��2023��11��29��
  * @brief    ������STM32CubeMX HAL��Ĵ���DMA�жϲ������շ�����
  * @attention:	���ò���				
  * STM32CubeMX	1.��ѡ��Ҫ�򿪵Ĵ���ͨ��,Modeѡ�� Asynchronous (�첽);
  *             2.Parameter Settings ��ѡ��Ĭ������;           
  *							3.NVIC Settings �ﹴѡEnabledѡ���µķ���,���ȼ����ÿ��������,��ֵԽС,���ȼ�Խ��;
	*             4.DMA Settings ��,�������Add,ѡ��UARTX_RX��UARTX_TX,����Ĭ��,���ȼ����ÿ��������;
	*             5.GPIO Settings ��Ĭ������;
	*             6.�������FreeRTOS,������ Mutexes ѡ�������� Add ,����һ����������Ĭ�ϲ�������;
	* Keil        7.�� DMA_UART.c �� DMA_UART.h �����ļ��ŵ�������;
	*             10.�� UartX_receive_Init(); �����ŵ� int main(void) �ĳ�ʼ�������;
	*             11.�� UartX_receive(); �����ŵ�stm32f4xx_it.c�ļ�����Ӧ�����жϺ���UARTX_IRQHandler()��;
	*             12.���ú궨��Uart_OS ,0��ʾ����FreeRTOSϵͳ,1��ʾ��FreeRTOSϵͳ;
	*             13.������ DMA_UART.h ������ rx_buf_size �� tx_buf_size �����С��Ĭ�ϸ�����255���ֽ�;
	*             14.����ͷ�ļ� DMA_UART.h ,����ʹ��UARTX_printf ��ӡ, Uart4_scanf ����;
	*             15.�ڴ�ӡUARTX_printfʱ,Ҫ�ں���ǰ����ϻ�ȡ���ͷŻ�����,
	*             ���磺osMutexWait(myMutex01Handle, osWaitForever);//��ȡ��������û�л�ȡ����һֱ�ȴ�
	*									  Uart4_printf("%s\r\n",char_buf);
											//osDelay(10);//��Ҫ�����
	*									  osMutexRelease( myMutex01Handle );          //�ͷŻ�����
	* @bug      : �������û�з�Ӧ,���ܴ��ڵ�����:
	*             1.emwin���õ��ڴ����,�� GUIConf.c ��ĺ궨�� GUI_NUMBYTES ��С���ɡ�
	*             2.FreeRTOS�ڴ�ռ�ù���Ҳ��ʹDMAʧЧ
	*             3.����ѡ΢��(use MicroLIB),�Ͳ���ʹ��printf,���Ҳ�������оƬ
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "DMA_UART.h"

/* defines ------------------------------------------------------------------*/
#define Uart_OS  (0)  //0��ʾ����FreeRTOSϵͳ,1��ʾ��FreeRTOSϵͳ
#if Uart_OS
	#include "cmsis_os.h"
#endif

//���崮��2 
DMA_Uart Uart2 =
{
	.UartTxBuf 		= {0},		//�������ݻ���
	.UartRxBuf 		= {0},		//�������ݻ���
	.Uart_Rx_flag = 0,			//���ڽ��ձ�־λ
  .Uart_Rx_len 	= 0,			//���ڽ��յ�ʵ�ʳ���
  .huart 				= &huart2,//����2
};
/* code ------------------------------------------------------------------*/
/**
* @brief ѡ����ʱ������
* @param Delay
* @retval None
*/
void Uart_Delay(uint32_t Delay)
{
	#if Uart_OS
	  /*FreeRTOSϵͳ��ʱ*/
		osDelay(Delay);
	#else
		/*�����ʱ*/
		HAL_Delay(Delay);
	#endif
}
/*-Uart-----------------------------------------------------------------------------------------------------*/
/**
* @brief  ����DMAʽprintf�ض���
* @param  Uart:���ں�, format:��ʽ
* @retval 
* @attention
*/
void Uart_printf(DMA_Uart *Uart,const char *format,...)
{
	while(!(__HAL_UART_GET_FLAG(Uart->huart,UART_FLAG_TC)))//�ж���һ���Ƿ�����ɣ����û�д�����ɣ��Ϳ������ﲻҪ��ȥ
	{
		//Uart_Delay(1); //������ʱ������ߴ���������
	}
	uint16_t len=0;
	va_list args;	
	va_start(args,format);
	len = vsnprintf((char*)Uart->UartTxBuf,sizeof(Uart->UartTxBuf)+1,(char*)format,args);
	va_end(args);
	HAL_UART_Transmit_DMA(Uart->huart, (uint8_t*)Uart->UartTxBuf, len);
}

/*
 * ����ʽ�ض���
 * ��DMA��������ʱ,�������������printf
 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}
/**
* @brief      ���� DMA�����жϳ�ʼ����
* @param      None
* @retval     None
* @attention  ��Ҫ�Ѵ˺����ŵ�main.c�ļ��ĳ����ʼ��������,����ʹ���жϽ���
*/
void Uart_receive_Init(DMA_Uart *Uart)
{
	HAL_UART_Receive_DMA(Uart->huart, (uint8_t *)Uart->UartRxBuf, rx_buf_size);  //ʹ�ܴ���DMA�����ж�
	__HAL_UART_ENABLE_IT(Uart->huart, UART_IT_IDLE);             //ʹ�ܿ����ж� 
}
/**
* @brief      ���� DMA�����жϲ��������ա�
* @param      None
* @retval     None
* @attention  ��Ҫ�Ѵ˺����ŵ�stm32f4xx_it.c�ļ�����Ӧ�����жϺ���UARTX_IRQHandler()��,����ͨ���жϽ���
*/
void Uart_receive(DMA_Uart *Uart)
{
	uint32_t temp=0;
	if((__HAL_UART_GET_FLAG(Uart->huart,UART_FLAG_IDLE) != RESET))  
	{   
		__HAL_UART_CLEAR_IDLEFLAG(Uart->huart);  //���״̬�Ĵ����ʹ������ݼĴ���
		HAL_UART_DMAStop(Uart->huart);           //ʧ��DMA����
		temp = __HAL_DMA_GET_COUNTER(Uart->huart->hdmarx);//��ȡ���ճ��ȣ��ܴ�С-ʣ���С
		Uart->Uart_Rx_len = rx_buf_size - temp; 
		Uart->Uart_Rx_flag=1;                                         //���ձ�־λ��1
		HAL_UART_Receive_DMA(Uart->huart, (uint8_t *)Uart->UartRxBuf, rx_buf_size);  //����ʹ�ܽ���DMA����
	}
}

/**
* @brief      ���ַ�����ֵ��
* @param     *pData:��Ҫ��ֵ���ַ���ָ��;length:����ַ����ĳ���
* @retval     flag:�����ݽ��յ��˾ͻ���1,û������ʱ���Զ�����
* @attention  
*/
uint8_t Uart_scanf(DMA_Uart *Uart,uint8_t *pData,uint8_t length)
{
	uint8_t flag=0;
	if(Uart->Uart_Rx_flag)    	// �����ձ�־λ=1ʱ��˵�������ݽ��յ���
	{
		flag=1;
		Uart->Uart_Rx_flag=0;	                    // ������ձ�־λ
		memset(pData,0, length);		          //����ս�������
    memcpy(pData,Uart->UartRxBuf,Uart->Uart_Rx_len);//�ٸ�������
		memset(Uart->UartRxBuf,0, rx_buf_size);		//��ȡ�����Ҫ��ս�������,��Ȼ��������벻���Ĵ���
	}
	else
	{
		flag=0;
	}
	return flag;
}
