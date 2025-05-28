#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include <stdio.h>
#include "sys.h"
/********************���н��մ�������********************/
#define USART1_LEN 800 //��������С
typedef struct 
{
	unsigned char buff[USART1_LEN];//������
    uint16_t usart1_rx_len;//��������ݳ���
    uint8_t usart1_flag;//���ݽ�����ɱ�־
	uint16_t w;//д
	uint16_t r;//��
	uint8_t is_last_reserve;//��һ�ν��ղ�ȫ��־λ
	uint16_t last_reserve_length;//��һ�ν��ղ�ȫ�ĳ���
}USART1_RX;

extern USART1_RX USART1_rx;
void Usartx_Init(USART_TypeDef *USARTx,u32 baud,u32 sysclk);
extern uint16_t ERROR_FLAG;  						              //��������������־λ���ﵽһ����������


uint16_t Usart1_Annular_txdata(uint8_t *tx_data);
void Usart1_SendString(unsigned char *data,unsigned int num);//����1�����ַ���

void MODBUS(unsigned char *recv_buffer,unsigned int length);

#endif

