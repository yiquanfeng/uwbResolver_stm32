/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ����PG5.8A��վ�ͱ�ǩ������ݽ�������
 * ����ʹ�õ�Ƭ����STM32F103C8T6
 * @author ���������Ƽ����޹�˾
 * @web www.gzlwkj.com
 */
 
 /*
   2023/08/14 v2.2
	 �޸��˴��������������
	 ���A��վ���ݽ�������
	 ���DMA����
   2023/08/14 v2.3
     ���������Ź�
	 �����˶Խ��ղ�ȫ���ݵĴ���
	 �޸�������ƫ�Ʋ���
	 �޸��˻��λ������Ķ�ȡ
*/

#include "stm32f10x.h"
#include "usart.h"
#include "timer.h"
#include "dma.h"
#include <string.h>


uint8_t usart1_rx_buf[400];	//��ȡ�������ݻ���
#define ERROR_MAX	1200

int main()
{
    uint16_t usart1_rx_length=0;
	Usartx_Init(USART1,115200,72);	
	MYDMA_Config1();  //����1 DMA�򿪣����Ϳ�DMA
	TIMx_Init(TIM2,3600-1,25-1);//ͨ����ʱ��2�������ڽ�������, 2.5ms
	TIMx_Init(TIM3,3600-1,2-1);
	while(1)
	{
		if(USART1_rx.usart1_flag)
		{
			usart1_rx_length = Usart1_Annular_txdata(usart1_rx_buf);//��ȡ������������
			if(usart1_rx_length)	//��ȡ���ĳ���
			{
				MODBUS(usart1_rx_buf, usart1_rx_length);
			}
		}
		if(ERROR_FLAG > ERROR_MAX)
		{
			ERROR_FLAG = 0;
			USART1_rx.usart1_flag = 0;
			USART1_rx.r = 0;
			USART1_rx.w = 0;
			USART1_rx.usart1_rx_len = 0;
			memset(usart1_rx_buf,0,sizeof(usart1_rx_buf));
		}
	}
}
