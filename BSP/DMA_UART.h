/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：DMA_UART.h
  * @version  ：v1.0
  * @date     ：2021年1月31日
  * @brief    ：串口DMA中断不定长收发
  * @attention:	配置步骤				
  * STM32CubeMX	1.勾选需要打开的串口通道,Mode选择 Asynchronous (异步);
  *             2.Parameter Settings 里选择默认配置;           
  *							3.NVIC Settings 里勾选Enabled选项下的方框,优先级设置看需求情况,数值越小,优先级越高;
	*             4.DMA Settings 里,点击左下Add,选择UARTX_RX和UARTX_TX,参数默认,优先级设置看需求情况;
	*             5.GPIO Settings 用默认设置;
	*             6.如果用了FreeRTOS,还需在 Mutexes 选项栏里点击 Add ,增加一个互斥量，默认参数即可;
	* Keil        7.将 DMA_UART.c 和 DMA_UART.h 两个文件放到工程里;
	*             10.将 UartX_receive_Init(); 函数放到 int main(void) 的初始化框架里;
	*             11.将 Uart4_receive(); 函数放到stm32f4xx_it.c文件的相应串口中断函数UARTX_IRQHandler()里;
	*             12.配置宏定义Uart_OS ,0表示不用FreeRTOS系统,1表示用FreeRTOS系统;
	*             13.可以在 DMA_UART.h 里配置 rx_buf_size 和 tx_buf_size 缓存大小，默认各缓存255个字节;
	*             14.复制头文件 DMA_UART.h ,即可使用UARTX_printf 打印, Uart4_scanf 接收;
	*             15.在打印UARTX_printf时,要在函数前后加上获取和释放互斥量,
	*             例如：osMutexWait(myMutex01Handle, osWaitForever);//获取互斥量，没有获取到就一直等待
	*									  Uart4_printf("%s\r\n",char_buf);
	*									  osMutexRelease( myMutex01Handle );          //释放互斥量
	* @bug      : 如果串口没有反应,可能存在的问题:
	*             1.emwin配置的内存过大,将 GUIConf.c 里的宏定义 GUI_NUMBYTES 改小即可。
	*             2.待补充......
  ******************************************************************************
  */
#ifndef __DMA_UART_H__
#define __DMA_UART_H__
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
/* defines ------------------------------------------------------------------*/
#define rx_buf_size 255     //默认存放255个字节数据
#define tx_buf_size 255

typedef struct
{
	uint8_t 	UartTxBuf[tx_buf_size];	//发送数据缓存
	uint8_t  	UartRxBuf[rx_buf_size];	//接受数据缓存
	uint8_t		Uart_Rx_flag;						//串口接收标志位
  uint16_t 	Uart_Rx_len;						//串口接收的实际长度
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
