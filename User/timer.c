#include "timer.h"
/****************TIM������װ************************
**�βΣ�TIM_TypeDef *TIMx -- �Ǹ���ʱ��
**			u16 psc -- Ԥ��Ƶϵ����0~65535��
**			u16 arr -- ��װ��ֵ��0~16��
****************************************************/
uint16_t time2_usart1 = 0;


void TIMx_Init(TIM_TypeDef *TIMx,u16 psc,u16 arr)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	if(TIM2 == TIMx)
	{
		/*1.��ʱ��*/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��	
		#ifdef TIM2_IRQ
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�
		STM32_NVIC_SetPriority(TIM2_IRQn,1,2);//�������ȼ�
		#endif
	}
	if(TIM3 == TIMx)
	{
		/*1.��ʱ��*/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��	
		#ifdef TIM3_IRQ
		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
		STM32_NVIC_SetPriority(TIM3_IRQn,2,2);//�������ȼ�
		#endif
	}
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIMx->CR1|=1<<0;//������ʱ��	
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
	{
		TIM_Cmd(TIM2, DISABLE);//�رն�ʱ��2
		USART1_rx.usart1_flag=1;//���ڽ��ձ�־λ��1
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIMx�����жϱ�־ 
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
	{
		if(ERROR_FLAG <= 3000)
			ERROR_FLAG++;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־ 
}
