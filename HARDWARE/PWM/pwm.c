#include "pwm.h"
#include"usart.h"

extern u16 period_TIM1 ;
extern u16 duty_TIM1  ;
extern u8 CollectFlag_TIM1;
  
extern u16 period_TIM2 ;
extern u16 duty_TIM2  ;
extern u8 CollectFlag_TIM2 ;

extern u16 period_TIM4 ;
extern u16 duty_TIM4  ;
extern u8 CollectFlag_TIM4 ;

extern u16 period_TIM5 ;
extern u16 duty_TIM5 ;
extern u8 CollectFlag_TIM5 ;

extern u32 frequency1;
extern	u32 frequency2;
extern	u32 frequency3;
extern	u32 frequency4;
/*�������ƣTvoid MOTOR_INIT()
	����      ������Ƶ����4������
*/
void MOTOR_INIT()
{
	   GPIO_InitTypeDef GPIO_InitStructure;
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);//ʹ��D��ʱ��
	   	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5 
	                               | GPIO_Pin_6 | GPIO_Pin_7; //��ʼ��GPIO
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //ͨ���������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOF,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
	    //��ʼ��Ϊ�㣬��ֹ�������ת��	

}

/*�������ƣTvoid MOTOR_OUT(u8 direct1,u16 pwm1,u8 direct2,u16 pwm2,u8 direct3,pwm3,u8 direct4,pwm4)
	����      ����4�������ת���ת�٣����directx==1,��ô���ת�����Ϊ10�����������Ϊ01
*/
void MOTOR_OUT(u8 direct1,u16 speed1,u8 direct2,u16 speed2,u8 direct3,u16 speed3,u8 direct4,u16 speed4)
{
//	  motor1_out0=  direct1;	//���1
//	  motor1_out1= !direct1;
	  if(direct1==0x01) { motor1_out0=1;motor1_out1=0;}
	  else 				{motor1_out0=0;motor1_out1=1;}
//	  motor2_out0=  direct2;	//���2
//	  motor2_out1= !direct2;
	  if(direct2==0x01) { motor2_out0=1;motor2_out1=0;}
	  else 				{motor2_out0=0;motor2_out1=1;}
//	  motor3_out0=  direct3;	//���3
//	  motor3_out1= !direct3;
	  if(direct3==0x01) { motor3_out0=1;motor3_out1=0;}
	  else 				{motor3_out0=0;motor3_out1=1;}
//	  motor4_out0=  direct4;	//���4
//	  motor4_out1= !direct4;
 	  if(direct4==0x01) { motor4_out0=1;motor4_out1=0;}
	  else 				{motor4_out0=0;motor4_out1=1;}
  	TIM_SetCompare1(TIM3,speed1);		//����ռ�ձ�
	TIM_SetCompare2(TIM3,speed2);		//����ռ�ձ�
	TIM_SetCompare3(TIM3,speed3);		//����ռ�ձ�
	TIM_SetCompare4(TIM3,speed4);		//����ռ�ձ�
}

 void TIM_SetCompare(u16 val1,u16 val2,u16 val3,u16 val4)
 {
   	TIM_SetCompare1(TIM3,val1);		//����ռ�ձ�
	TIM_SetCompare2(TIM3,val2);		//����ռ�ձ�
	TIM_SetCompare3(TIM3,val3);		//����ռ�ձ�
	TIM_SetCompare4(TIM3,val4);		//����ռ�ձ�
 
 }
/*�������ƣTIM3_PWM_Init(u16 arr,u16 psc)
	����      TIM3������·PWM
*/
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
	
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //Timer3ȫӳ�� GPIOC-> 6,7,8,9                                                                    	 //����TIM3��CH2�����PWMͨ����LED��ʾ
 
   //���ø�����Ϊ�����������,���TIM3 CH1 CH2 CH3 CH4 ��PWM���岨��
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//Ĭ�ϵ��ʹ�ܶ�״̬����ʹ�� 

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ������72��Ƶ����ôʱ��Ƶ�ʾ���1M 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIMx��CCR1�ϵ�Ԥװ�ؼĴ���
	
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIMx��CCR3�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIMx��CCR4�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
 

}



/*�������ƣTTIM4_PWMINPUT_INIT(u16 arr,u16 psc)
  ����      PWM�����ʼ��*/

void TIM4_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM�ĳ�ʼ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;			//�ж�����
	TIM_ICInitTypeDef  TIM4_ICInitStructure;		 //TIM4  PWM���ýṹ��
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO�����ýṹ��
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);     //Open TIM4 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //open gpioB clock
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;             //GPIO 7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	
	/*�����ж����ȼ�*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM4_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:����
 
  TIM_PWMIConfig(TIM4, &TIM4_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);//������ʱ���ı�������                                       
  TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //�ж�����
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
  TIM_Cmd(TIM4, ENABLE);    
}


void TIM4_IRQHandler(void)
{

//	if(CollectFlag_TIM4)
//	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)//����1���������¼�
			{	
				duty_TIM4    =   TIM_GetCapture1(TIM4); 	 //�ɼ�ռ�ձ�		
			    //period_TIM4	=	TIM_GetCapture2(TIM4);     //�ɼ�����
		//		frequency1=1000000/period_TIM4	;		  //�˴����ܻ�ռ��һ��ʱ��
	//	printf("per=%d\r\n",period_TIM4);
	if  (TIM_GetCapture2(TIM4)>600)	 period_TIM4	=	TIM_GetCapture2(TIM4);
				CollectFlag_TIM4 = 0;
				
		//	}
			
			
	}	
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}


/*�������ƣTTIM1_PWMINPUT_INIT(u16 arr,u16 psc)
  ����      PWM�����ʼ��*/
 
void TIM1_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM�ĳ�ʼ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;			//�ж�����
	TIM_ICInitTypeDef  TIM1_ICInitStructure;		 //PWM���ýṹ��
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO�����ýṹ��
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);     //Open TIM1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  //open gpioE clock
   GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH2->PE11	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //GPIO 11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	
	/*�����ж����ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannel =  TIM1_CC_IRQn;   //TIM1�����ж�                      
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM1_ICInitStructure.TIM_ICFilter = 0x03;   //Filter:����
 
  TIM_PWMIConfig(TIM1, &TIM1_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//������ʱ���ı�������                                       
 // TIM_ITConfig(TIM1, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //�ж�����
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); //ͨ��2 �����жϴ�
  //TIM_ClearITPendingBit(TIM1, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
  TIM_Cmd(TIM1, ENABLE);    
}


void TIM1_CC_IRQHandler(void)
{

//	if(CollectFlag_TIM1)
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//����1���������¼�
			{	
				duty_TIM1    =   TIM_GetCapture1(TIM1); 	 //�ɼ�ռ�ձ�		
			  // period_TIM1	=	TIM_GetCapture2(TIM1);     //�ɼ�����
			   if  (TIM_GetCapture2(TIM1)>600)	 period_TIM1	=	TIM_GetCapture2(TIM1);
				CollectFlag_TIM1 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}

 
/*�������ƣTTIM2_PWMINPUT_INIT(u16 arr,u16 psc)
  ����      PWM�����ʼ��*/

void TIM2_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM�ĳ�ʼ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;			//�ж�����
	TIM_ICInitTypeDef  TIM2_ICInitStructure;		 //TIM2  PWM���ýṹ��
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO�����ýṹ��
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);     //Open TIM2 clock
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //open gpioB clock
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);          //�ر�JTAG
 	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2��ȫ��ӳ��  TIM2_CH2->PB3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             //GPIO 3
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;	  //�������� ��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	
	/*�����ж����ȼ�*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM2_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:����
 
  TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//������ʱ���ı�������                                       
  TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //�ж�����
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
  TIM_Cmd(TIM2, ENABLE);    
}


void TIM2_IRQHandler(void)
{

	//if(CollectFlag_TIM2)
	{
		if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//����1���������¼�
			{	
				duty_TIM2    =   TIM_GetCapture1(TIM2); 	 //�ɼ�ռ�ձ�		
			   // period_TIM2	=	TIM_GetCapture2(TIM2);     //�ɼ�����
			   if  (TIM_GetCapture2(TIM2)>600)	 period_TIM2	=	TIM_GetCapture2(TIM2);
				CollectFlag_TIM2 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}

 /*�������ƣTTIM5_PWMINPUT_INIT(u16 arr,u16 psc)
  ����      PWM�����ʼ��*/

void TIM5_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM�ĳ�ʼ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;			//�ж�����
	TIM_ICInitTypeDef  TIM5_ICInitStructure;		 //TIM4  PWM���ýṹ��
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO�����ýṹ��
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);     //Open TIM5 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //open gpioB clock
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;             //GPIO 1
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;	  //��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	
	/*�����ж����ȼ�*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM5_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:����
 
  TIM_PWMIConfig(TIM5, &TIM5_ICInitStructure);     //PWM��������           
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);     //ѡ����Ч�����        
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
  TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);//������ʱ���ı�������                                       
  TIM_ITConfig(TIM5, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //�ж�����
  TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
  TIM_Cmd(TIM5, ENABLE);    
}


void TIM5_IRQHandler(void)
{

//	if(CollectFlag_TIM5)
	{
		if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//����1���������¼�
			{	
				duty_TIM5    =   TIM_GetCapture1(TIM5); 	 //�ɼ�ռ�ձ�		
			//    period_TIM5	=	TIM_GetCapture2(TIM5);     //�ɼ�����
			if  (TIM_GetCapture2(TIM5)>600)	 period_TIM5	=	TIM_GetCapture2(TIM5);
				CollectFlag_TIM5 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
}

