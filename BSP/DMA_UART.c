/**
  ******************************************************************************
  * @author   ：MoMoNiz
	* @file     ：DMA_UART.h
  * @version  ：v1.2
  * @date     ：2023年11月29日
  * @brief    ：基于STM32CubeMX HAL库的串口DMA中断不定长收发程序
  * @attention:	配置步骤				
  * STM32CubeMX	1.勾选需要打开的串口通道,Mode选择 Asynchronous (异步);
  *             2.Parameter Settings 里选择默认配置;           
  *							3.NVIC Settings 里勾选Enabled选项下的方框,优先级设置看需求情况,数值越小,优先级越高;
	*             4.DMA Settings 里,点击左下Add,选择UARTX_RX和UARTX_TX,参数默认,优先级设置看需求情况;
	*             5.GPIO Settings 用默认设置;
	*             6.如果用了FreeRTOS,还需在 Mutexes 选项栏里点击 Add ,增加一个互斥量，默认参数即可;
	* Keil        7.将 DMA_UART.c 和 DMA_UART.h 两个文件放到工程里;
	*             10.将 UartX_receive_Init(); 函数放到 int main(void) 的初始化框架里;
	*             11.将 UartX_receive(); 函数放到stm32f4xx_it.c文件的相应串口中断函数UARTX_IRQHandler()里;
	*             12.配置宏定义Uart_OS ,0表示不用FreeRTOS系统,1表示用FreeRTOS系统;
	*             13.可以在 DMA_UART.h 里配置 rx_buf_size 和 tx_buf_size 缓存大小，默认各缓存255个字节;
	*             14.复制头文件 DMA_UART.h ,即可使用UARTX_printf 打印, Uart4_scanf 接收;
	*             15.在打印UARTX_printf时,要在函数前后加上获取和释放互斥量,
	*             例如：osMutexWait(myMutex01Handle, osWaitForever);//获取互斥量，没有获取到就一直等待
	*									  Uart4_printf("%s\r\n",char_buf);
											//osDelay(10);//主要靠这个
	*									  osMutexRelease( myMutex01Handle );          //释放互斥量
	* @bug      : 如果串口没有反应,可能存在的问题:
	*             1.emwin配置的内存过大,将 GUIConf.c 里的宏定义 GUI_NUMBYTES 改小即可。
	*             2.FreeRTOS内存占用过多也会使DMA失效
	*             3.不勾选微库(use MicroLIB),就不能使用printf,而且不能启动芯片
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
#define Uart_OS  (0)  //0表示不用FreeRTOS系统,1表示用FreeRTOS系统
#if Uart_OS
	#include "cmsis_os.h"
#endif

//定义串口2 
DMA_Uart Uart2 =
{
	.UartTxBuf 		= {0},		//发送数据缓存
	.UartRxBuf 		= {0},		//接受数据缓存
	.Uart_Rx_flag = 0,			//串口接收标志位
  .Uart_Rx_len 	= 0,			//串口接收的实际长度
  .huart 				= &huart2,//串口2
};
/* code ------------------------------------------------------------------*/
/**
* @brief 选择延时函数。
* @param Delay
* @retval None
*/
void Uart_Delay(uint32_t Delay)
{
	#if Uart_OS
	  /*FreeRTOS系统延时*/
		osDelay(Delay);
	#else
		/*裸机延时*/
		HAL_Delay(Delay);
	#endif
}
/*-Uart-----------------------------------------------------------------------------------------------------*/
/**
* @brief  串口DMA式printf重定向。
* @param  Uart:串口号, format:格式
* @retval 
* @attention
*/
void Uart_printf(DMA_Uart *Uart,const char *format,...)
{
	while(!(__HAL_UART_GET_FLAG(Uart->huart,UART_FLAG_TC)))//判断上一次是否传输完成，如果没有传输完成，就卡在这里不要下去
	{
		//Uart_Delay(1); //加入延时可以提高传输完整性
	}
	uint16_t len=0;
	va_list args;	
	va_start(args,format);
	len = vsnprintf((char*)Uart->UartTxBuf,sizeof(Uart->UartTxBuf)+1,(char*)format,args);
	va_end(args);
	HAL_UART_Transmit_DMA(Uart->huart, (uint8_t*)Uart->UartTxBuf, len);
}

/*
 * 阻塞式重定向
 * 当DMA不起作用时,就用这个来串口printf
 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}
/**
* @brief      串口 DMA空闲中断初始化。
* @param      None
* @retval     None
* @attention  需要把此函数放到main.c文件的程序初始化操作里,才能使能中断接收
*/
void Uart_receive_Init(DMA_Uart *Uart)
{
	HAL_UART_Receive_DMA(Uart->huart, (uint8_t *)Uart->UartRxBuf, rx_buf_size);  //使能串口DMA接收中断
	__HAL_UART_ENABLE_IT(Uart->huart, UART_IT_IDLE);             //使能空闲中断 
}
/**
* @brief      串口 DMA空闲中断不定长接收。
* @param      None
* @retval     None
* @attention  需要把此函数放到stm32f4xx_it.c文件的相应串口中断函数UARTX_IRQHandler()里,才能通过中断接收
*/
void Uart_receive(DMA_Uart *Uart)
{
	uint32_t temp=0;
	if((__HAL_UART_GET_FLAG(Uart->huart,UART_FLAG_IDLE) != RESET))  
	{   
		__HAL_UART_CLEAR_IDLEFLAG(Uart->huart);  //清除状态寄存器和串口数据寄存器
		HAL_UART_DMAStop(Uart->huart);           //失能DMA接收
		temp = __HAL_DMA_GET_COUNTER(Uart->huart->hdmarx);//读取接收长度，总大小-剩余大小
		Uart->Uart_Rx_len = rx_buf_size - temp; 
		Uart->Uart_Rx_flag=1;                                         //接收标志位置1
		HAL_UART_Receive_DMA(Uart->huart, (uint8_t *)Uart->UartRxBuf, rx_buf_size);  //重新使能接收DMA接收
	}
}

/**
* @brief      给字符串赋值。
* @param     *pData:需要赋值的字符串指针;length:这个字符串的长度
* @retval     flag:有数据接收到了就会置1,没有数据时会自动置零
* @attention  
*/
uint8_t Uart_scanf(DMA_Uart *Uart,uint8_t *pData,uint8_t length)
{
	uint8_t flag=0;
	if(Uart->Uart_Rx_flag)    	// 当接收标志位=1时，说明有数据接收到了
	{
		flag=1;
		Uart->Uart_Rx_flag=0;	                    // 清零接收标志位
		memset(pData,0, length);		          //先清空接收数组
    memcpy(pData,Uart->UartRxBuf,Uart->Uart_Rx_len);//再复制数组
		memset(Uart->UartRxBuf,0, rx_buf_size);		//读取完后需要清空接收数组,不然会出现意想不到的错误
	}
	else
	{
		flag=0;
	}
	return flag;
}
