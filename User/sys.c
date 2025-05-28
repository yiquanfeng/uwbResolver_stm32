#include "sys.h"
/*****************************�������ȼ�******************
**1.�������ȼ�����
**2.��ȡ���ȼ�����
**3.�������ȼ�
**4.ʹ���ж���
**�βΣ�IRQn_Type IRQn --- �ж���
**			uint32_t PreemptPriority -- ������ռ���ȼ�
**			uint32_t SubPriority  --���������ȼ�
*********************************************************/
void STM32_NVIC_SetPriority(IRQn_Type IRQn,uint32_t PreemptPriority, uint32_t SubPriority)
{
	uint32_t priority;
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);//�������ȼ�����,��ռ����2λ�������ȼ�2λ
	priority=NVIC_EncodePriority (NVIC_PriorityGroup_2, PreemptPriority, SubPriority);//�������ȼ�����
	NVIC_SetPriority(IRQn,priority);//�������ȼ�
	NVIC_EnableIRQ(IRQn);//ʹ���ж���
}
