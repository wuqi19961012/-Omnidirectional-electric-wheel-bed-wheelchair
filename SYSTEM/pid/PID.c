#include"pid.h"
#include"pwm.h"


extern PIDtypedef PID1;
extern PIDtypedef PID2;
extern PIDtypedef PID3;
extern PIDtypedef PID4;

extern u16 period_TIM1 ;
extern u8 CollectFlag_TIM1; 
extern u16 period_TIM2 ;
extern u8 CollectFlag_TIM2 ;
extern u16 period_TIM4 ;
extern u8 CollectFlag_TIM4 ;
extern u16 period_TIM5 ;
extern u8 CollectFlag_TIM5 ;

extern u32 frequency1;
extern	u32 frequency2;
extern	u32 frequency3;
extern	u32 frequency4;
extern u8 start_flag;
extern u16 pwm1,pwm2,pwm3,pwm4;

void PIDperiodinit(u16 arr,u16 psc)
 {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM6��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


//	TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx

 }


 void TIM6_IRQHandler(void)	//	����ʱ�䵽���жϴ�����
{	  u16 temp;
	
 if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)//�����ж�
			{
//			 TIM_Cmd(TIM4, DISABLE);
//			 TIM_Cmd(TIM1, DISABLE);
//			 TIM_Cmd(TIM2, DISABLE);
//			 TIM_Cmd(TIM5, DISABLE);
	 //if(!CollectFlag_TIM4)
	//	{	
	//	if((period_TIM4>600)||(start_flag==0))	   // period_TIM4�ǲ�����С��600us��
	//	{
		//printf("period=%d\r\n",period_TIM4);
		//	if(period_TIM4>600)
			{
			frequency1=1000000/period_TIM4	;
			frequency2=1000000/period_TIM1	;
			frequency3=1000000/period_TIM2	;
			frequency4=1000000/period_TIM5	;
		//	printf("f1    =%dKHz\r\n",frequency1);
		//	 if(((frequency1<=PID1.setpoint)&&((PID1.setpoint-frequency1))<20)||(frequency1>PID1.setpoint)&&((frequency1-PID1.setpoint)<20))TIM_Cmd(TIM6, DISABLE);
	//	if(frequency1 >1536) frequency1=1536;
		//	printf("f2    =%dKHz\r\n",frequency2);

temp= (frequency1*215)/100	;
	//printf("f*2.15=%d\r\n",temp)  ;
//			 Dac1_Set_Vol(temp); 

	      	 PID1.sum_error+=(incPIDcalc(&PID1,frequency1));	 //�����������ۼ�
			 pwm1=PID1.sum_error*0.0651  ;   //*0.651    ��100/1536
		//	 printf("pwm1=%d\r\n",pwm1);
			frequency1=0;
			period_TIM4=0;
		//	CollectFlag_TIM4 = 1 ;

		//	printf("f2=%dKHz\r\n",frequency2);
			temp= (frequency2*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
		//	 Dac1_Set_Vol(temp);
	      	 PID2.sum_error+=(incPIDcalc(&PID2,frequency2));	 //�����������ۼ�  Y=Y+Y'
			 pwm2=PID2.sum_error*0.0651 ;   //*0.651 
	//		 printf("pwm2=%d\r\n",pwm2);
			frequency2=0;
			period_TIM1=0;

     	//	printf("f3=%dKHz\r\n",frequency3);
			temp= (frequency3*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
			// Dac1_Set_Vol(temp);
	      	 PID3.sum_error+=(incPIDcalc(&PID3,frequency3));	 //�����������ۼ�
			 pwm3=PID3.sum_error*0.0651 ;   //*0.651 
		//	 printf("pwm3=%d\r\n",pwm3);
			frequency3=0;
			period_TIM2=0;

		//	printf("f4=%dKHz\r\n",frequency4);
			temp= (frequency4*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
		//	 Dac1_Set_Vol(temp);
	      	 PID4.sum_error+=(incPIDcalc(&PID4,frequency4));	 //�����������ۼ�
			 pwm4=PID4.sum_error*0.0651 ;   //*0.651 
		//	 printf("pwm4=%d\r\n",pwm4);
			frequency4=0;
			period_TIM5=0; 
			}
		   }
			 TIM_SetCompare(pwm1,pwm2,pwm3,pwm4);	     //�����趨PWMֵ
			 TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //����жϱ�־λ



//			 TIM_Cmd(TIM4, ENABLE);
//			 TIM_Cmd(TIM1, ENABLE);
//			 TIM_Cmd(TIM2, ENABLE);
//			 TIM_Cmd(TIM5, ENABLE);
		//	 }
		//	}
			
			
		
		
}
void incPIDinit(void)
{
//PID1������ʼ��
 PID1.sum_error=0;
 PID1.last_error=0;
 PID1.prev_error=0;
 PID1.proportion=0;
 PID1.integral=0;
 PID1.derivative=0;
 PID1.setpoint=0;

//PID2������ʼ��
 PID2.sum_error=0;
 PID2.last_error=0;
 PID2.prev_error=0;
 PID2.proportion=0;
 PID2.integral=0;
 PID2.derivative=0;
 PID2.setpoint=0;

//PID3������ʼ��
 PID3.sum_error=0;
 PID3.last_error=0;
 PID3.prev_error=0;
 PID3.proportion=0;
 PID3.integral=0;
 PID3.derivative=0;
 PID3.setpoint=0;

//PID4������ʼ��
 PID4.sum_error=0;
 PID4.last_error=0;
 PID4.prev_error=0;
 PID4.proportion=0;
 PID4.integral=0;
 PID4.derivative=0;
 PID4.setpoint=0;
}
 void PID_setpoint(PIDtypedef*PIDx,u16 setvalue)
 {
  PIDx->setpoint=setvalue; 
 }
int incPIDcalc(PIDtypedef *PIDx,u16 nextpoint)
{
 int iError,iincpid;
 iError=PIDx->setpoint-nextpoint;  //��ǰ���
 iincpid=					       //��������
 PIDx->proportion*iError	        //e[k]��
 -PIDx->integral*PIDx->last_error	  //e[k-1]
 +PIDx->derivative*PIDx->prev_error;//e[k-2]

 PIDx->prev_error=PIDx->last_error; //�洢�������´μ���
 PIDx->last_error=iError;
 return(iincpid) ;
}
void PID_set(float pp,float ii,float dd)
{
  PID1.proportion=pp;
 PID1.integral=ii;
 PID1.derivative=dd; 
 PID2.proportion=pp;
 PID2.integral=ii;
 PID2.derivative=dd;
  PID3.proportion=pp;
 PID3.integral=ii;
 PID3.derivative=dd;
  PID4.proportion=pp;
 PID4.integral=ii;
 PID4.derivative=dd;
}

/*����趨�ĸ����ӵ�ת�٣�ת�� 1rad/s�ȼ���122.23������ÿ���ת��*/
  void set_speed(float W1,float W2,float W3,float W4)
	{
			float temp;
  if(W1>0) 		  //�ж�W ��������������
	  { 
	  	motor1_out0=0;
	  	motor1_out1=1;
	  	temp=W1*122.23;
	  	PID_setpoint(&PID1,temp);
//		printf("W1=%f\r\n",W1);			  //����ͨ��
//		printf("temp=%f\r\n",temp);
//		printf("PID1=%d\r\n",PID1.setpoint);
	  }
	  else   if(W1==0) 			//��ֵ
	  { 
	  //	motor1_out0=0;
	  //	motor1_out1=0;
	  	
	  	PID_setpoint(&PID1,0);
	  }
	  else 				  //��������
	  {
	  	motor1_out0=1;
		motor1_out1=0;
		temp=-W1*122.23;
		PID_setpoint(&PID1,temp);
		}

  if(W2>0) 		
	  { 
	  	motor2_out0=0;
	  	motor2_out1=1;
	  	temp=W2*122.23;
	  	PID_setpoint(&PID2,temp);
	  }
	  else   if(W2==0) 		
	  { 
	  //	motor2_out0=0;
	  //	motor2_out1=0;
	  	
	  	PID_setpoint(&PID2,0);
	  }
	  else 				
	  {
	  	motor2_out0=1;
		motor2_out1=0;
		temp=-W2*122.23;
		PID_setpoint(&PID2,temp);
		}

 if(W3>0) 		
	  { 
	  	motor3_out0=0;
	  	motor3_out1=1;
	  	temp=W3*122.23;
	  	PID_setpoint(&PID3,temp);
	  }
	  else   if(W3==0) 		
	  { 
	  //	motor3_out0=0;
	  //	motor3_out1=0;
	  	
	  	PID_setpoint(&PID3,0);
	  }
	  else 				
	  {
	  	motor3_out0=1;
		motor3_out1=0;
		temp=-W3*122.23;
		PID_setpoint(&PID3,temp);
		}

  if(W4>0) 		
	  { 
	  	motor4_out0=0;
	  	motor4_out1=1;
	  	temp=W4*122.23;
	  	PID_setpoint(&PID4,temp);
	  }
	  else   if(W4==0) 		
	  { 
	  //	motor4_out0=0;
	  //	motor4_out1=0;
	  	
	  	PID_setpoint(&PID4,0);
	  }
	  else 				
	  {
	  	motor4_out0=1;
		motor4_out1=0;
		temp=-W4*122.23;
		PID_setpoint(&PID4,temp);
		}
//	  if(W2>=0)			 { motor2_out0=1;motor2_out1=0;}
//	  else 				{motor2_out0=0;motor2_out1=1;}
//
//	  if(W3>0) 			{ motor3_out0=1;motor3_out1=0;}
//	  else 				{motor3_out0=0;motor3_out1=1;}
//
// 	  if(W4>=0)			 { motor4_out0=1;motor4_out1=0;}
//	  else 				{motor4_out0=0;motor4_out1=1;}
//
//		//	temp=abs(W1)*122.23;
//		temp=abs(W1);			
//	        PID_setpoint(&PID1,temp);
//				printf("Wwww=%f\r\n",temp); //������ ����
//				 printf("PID1=%d\r\n",PID1.setpoint);  //���� ����
//			temp=abs(W2)*122.23;
//			PID_setpoint(&PID2,temp);
//			temp=abs(W3)*122.23;
//			PID_setpoint(&PID3,temp);
//			temp=abs(W4)*122.23;
//			PID_setpoint(&PID4,temp);
         }
void stop()
{
//	TIM_SetCompare1(TIM3,0);		//����ռ�ձ�	   PC6
//	TIM_SetCompare2(TIM3,0);		//����ռ�ձ�	   PC7
//	TIM_SetCompare3(TIM3,0);		//����ռ�ձ�	   PC8
//	TIM_SetCompare4(TIM3,0);		//����ռ�ձ�	   PC9
    PID_setpoint(&PID1,0);
	 PID1.sum_error=0;
     PID1.last_error=0;
     PID1.prev_error=0;
	PID_setpoint(&PID2,0);
	 PID2.sum_error=0;
	 PID2.last_error=0;
	 PID2.prev_error=0;
	PID_setpoint(&PID3,0);
	 PID3.sum_error=0;
	 PID3.last_error=0;
	 PID3.prev_error=0;
	PID_setpoint(&PID4,0);
	 PID4.sum_error=0;
	 PID4.last_error=0;
	 PID4.prev_error=0;
}

