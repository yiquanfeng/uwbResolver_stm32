#ifndef _SYS_H
#define _SYS_H
#include "stm32f10x.h"
/******Cortex-M3Ȩ��ָ��92ҳ*/
/*��λ����ַ��λ���ת��Ϊ������ַ�ĺ�*/
/*
addr --�Ĵ����ĵ�ַ
bitnum -- λ���
*/
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
//�ѵ�ַת��Ϊָ����ʽ
#define MEM_ADDR(addr) *((volatile unsigned long *) (addr))

/*****GPIO�Ĵ�������ַ******/
#define GPIOA_IDR (0x40010800+0x8)
#define GPIOA_ODR (0x40010800+0xC)
#define GPIOB_IDR (0X40010C00+0x8)
#define GPIOB_ODR (0X40010C00+0xC)

#define PAin(bitnum) MEM_ADDR(BITBAND(GPIOA_IDR, bitnum))
#define PAout(bitnum) MEM_ADDR(BITBAND(GPIOA_ODR, bitnum))
#define PBin(bitnum) MEM_ADDR(BITBAND(GPIOB_IDR, bitnum))
#define PBout(bitnum) MEM_ADDR(BITBAND(GPIOB_ODR, bitnum))

/**
4��λ�����ж����ȼ�

@code  
 The table below gives the allowed values of the pre-emption priority and subpriority according
 to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
                         |                                   |                             |   4 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority                       
  ============================================================================================================================
@endcode
*/
void STM32_NVIC_SetPriority(IRQn_Type IRQn,uint32_t PreemptPriority, uint32_t SubPriority);//�������ȼ�

#endif
