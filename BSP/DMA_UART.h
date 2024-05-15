/**
  ******************************************************************************
  * @author   ��MoMoNiz
	* @file     ��DMA_UART.h
  * @version  ��v1.0
  * @date     ��2021��1��31��
  * @brief    ������DMA�жϲ������շ�
  * @attention:	���ò���				
  * STM32CubeMX	1.��ѡ��Ҫ�򿪵Ĵ���ͨ��,Modeѡ�� Asynchronous (�첽);
  *             2.Parameter Settings ��ѡ��Ĭ������;           
  *							3.NVIC Settings �ﹴѡEnabledѡ���µķ���,���ȼ����ÿ��������,��ֵԽС,���ȼ�Խ��;
	*             4.DMA Settings ��,�������Add,ѡ��UARTX_RX��UARTX_TX,����Ĭ��,���ȼ����ÿ��������;
	*             5.GPIO Settings ��Ĭ������;
	*             6.�������FreeRTOS,������ Mutexes ѡ�������� Add ,����һ����������Ĭ�ϲ�������;
	* Keil        7.�� DMA_UART.c �� DMA_UART.h �����ļ��ŵ�������;
	*             10.�� UartX_receive_Init(); �����ŵ� int main(void) �ĳ�ʼ�������;
	*             11.�� Uart4_receive(); �����ŵ�stm32f4xx_it.c�ļ�����Ӧ�����жϺ���UARTX_IRQHandler()��;
	*             12.���ú궨��Uart_OS ,0��ʾ����FreeRTOSϵͳ,1��ʾ��FreeRTOSϵͳ;
	*             13.������ DMA_UART.h ������ rx_buf_size �� tx_buf_size �����С��Ĭ�ϸ�����255���ֽ�;
	*             14.����ͷ�ļ� DMA_UART.h ,����ʹ��UARTX_printf ��ӡ, Uart4_scanf ����;
	*             15.�ڴ�ӡUARTX_printfʱ,Ҫ�ں���ǰ����ϻ�ȡ���ͷŻ�����,
	*             ���磺osMutexWait(myMutex01Handle, osWaitForever);//��ȡ��������û�л�ȡ����һֱ�ȴ�
	*									  Uart4_printf("%s\r\n",char_buf);
	*									  osMutexRelease( myMutex01Handle );          //�ͷŻ�����
	* @bug      : �������û�з�Ӧ,���ܴ��ڵ�����:
	*             1.emwin���õ��ڴ����,�� GUIConf.c ��ĺ궨�� GUI_NUMBYTES ��С���ɡ�
	*             2.������......
  ******************************************************************************
  */
#ifndef __DMA_UART_H__
#define __DMA_UART_H__
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
/* defines ------------------------------------------------------------------*/
#define rx_buf_size 255     //Ĭ�ϴ��255���ֽ�����
#define tx_buf_size 255

typedef struct
{
	uint8_t 	UartTxBuf[tx_buf_size];	//�������ݻ���
	uint8_t  	UartRxBuf[rx_buf_size];	//�������ݻ���
	uint8_t		Uart_Rx_flag;						//���ڽ��ձ�־λ
  uint16_t 	Uart_Rx_len;						//���ڽ��յ�ʵ�ʳ���
  UART_HandleTypeDef *huart;
}DMA_Uart;

extern DMA_Uart Uart2;
/* function ------------------------------------------------------------------*/
void Uart_Delay(uint32_t Delay);
void Uart_printf(DMA_Uart *Uart,const char *format,...);
void Uart_receive_Init(DMA_Uart *Uart);
void Uart_receive(DMA_Uart *Uart);
uint8_t Uart_scanf(DMA_Uart *Uart,uint8_t *pData,uint8_t length);


#endif 
