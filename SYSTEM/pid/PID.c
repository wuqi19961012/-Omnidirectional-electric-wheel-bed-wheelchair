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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


//	TIM_Cmd(TIM6, ENABLE);  //使能TIMx

 }


 void TIM6_IRQHandler(void)	//	采样时间到，中断处理函数
{	  u16 temp;
	
 if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)//更新中断
			{
//			 TIM_Cmd(TIM4, DISABLE);
//			 TIM_Cmd(TIM1, DISABLE);
//			 TIM_Cmd(TIM2, DISABLE);
//			 TIM_Cmd(TIM5, DISABLE);
	 //if(!CollectFlag_TIM4)
	//	{	
	//	if((period_TIM4>600)||(start_flag==0))	   // period_TIM4是不可能小于600us的
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

	      	 PID1.sum_error+=(incPIDcalc(&PID1,frequency1));	 //计算增量并累加
			 pwm1=PID1.sum_error*0.0651  ;   //*0.651    即100/1536
		//	 printf("pwm1=%d\r\n",pwm1);
			frequency1=0;
			period_TIM4=0;
		//	CollectFlag_TIM4 = 1 ;

		//	printf("f2=%dKHz\r\n",frequency2);
			temp= (frequency2*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
		//	 Dac1_Set_Vol(temp);
	      	 PID2.sum_error+=(incPIDcalc(&PID2,frequency2));	 //计算增量并累加  Y=Y+Y'
			 pwm2=PID2.sum_error*0.0651 ;   //*0.651 
	//		 printf("pwm2=%d\r\n",pwm2);
			frequency2=0;
			period_TIM1=0;

     	//	printf("f3=%dKHz\r\n",frequency3);
			temp= (frequency3*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
			// Dac1_Set_Vol(temp);
	      	 PID3.sum_error+=(incPIDcalc(&PID3,frequency3));	 //计算增量并累加
			 pwm3=PID3.sum_error*0.0651 ;   //*0.651 
		//	 printf("pwm3=%d\r\n",pwm3);
			frequency3=0;
			period_TIM2=0;

		//	printf("f4=%dKHz\r\n",frequency4);
			temp= (frequency4*215)/100	;
		//		printf("f*2.15=%d\r\n",temp)  ;
		//	 Dac1_Set_Vol(temp);
	      	 PID4.sum_error+=(incPIDcalc(&PID4,frequency4));	 //计算增量并累加
			 pwm4=PID4.sum_error*0.0651 ;   //*0.651 
		//	 printf("pwm4=%d\r\n",pwm4);
			frequency4=0;
			period_TIM5=0; 
			}
		   }
			 TIM_SetCompare(pwm1,pwm2,pwm3,pwm4);	     //重新设定PWM值
			 TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //清除中断标志位



//			 TIM_Cmd(TIM4, ENABLE);
//			 TIM_Cmd(TIM1, ENABLE);
//			 TIM_Cmd(TIM2, ENABLE);
//			 TIM_Cmd(TIM5, ENABLE);
		//	 }
		//	}
			
			
		
		
}
void incPIDinit(void)
{
//PID1参数初始化
 PID1.sum_error=0;
 PID1.last_error=0;
 PID1.prev_error=0;
 PID1.proportion=0;
 PID1.integral=0;
 PID1.derivative=0;
 PID1.setpoint=0;

//PID2参数初始化
 PID2.sum_error=0;
 PID2.last_error=0;
 PID2.prev_error=0;
 PID2.proportion=0;
 PID2.integral=0;
 PID2.derivative=0;
 PID2.setpoint=0;

//PID3参数初始化
 PID3.sum_error=0;
 PID3.last_error=0;
 PID3.prev_error=0;
 PID3.proportion=0;
 PID3.integral=0;
 PID3.derivative=0;
 PID3.setpoint=0;

//PID4参数初始化
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
 iError=PIDx->setpoint-nextpoint;  //当前误差
 iincpid=					       //增量计算
 PIDx->proportion*iError	        //e[k]项
 -PIDx->integral*PIDx->last_error	  //e[k-1]
 +PIDx->derivative*PIDx->prev_error;//e[k-2]

 PIDx->prev_error=PIDx->last_error; //存储误差，便于下次计算
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

/*最后设定四个轮子的转速，转速 1rad/s等价于122.23个脉冲每秒的转速*/
  void set_speed(float W1,float W2,float W3,float W4)
	{
			float temp;
  if(W1>0) 		  //判断W 正负，正数处理
	  { 
	  	motor1_out0=0;
	  	motor1_out1=1;
	  	temp=W1*122.23;
	  	PID_setpoint(&PID1,temp);
//		printf("W1=%f\r\n",W1);			  //调试通过
//		printf("temp=%f\r\n",temp);
//		printf("PID1=%d\r\n",PID1.setpoint);
	  }
	  else   if(W1==0) 			//零值
	  { 
	  //	motor1_out0=0;
	  //	motor1_out1=0;
	  	
	  	PID_setpoint(&PID1,0);
	  }
	  else 				  //负数处理
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
//				printf("Wwww=%f\r\n",temp); //浮点数 正常
//				 printf("PID1=%d\r\n",PID1.setpoint);  //整数 正常
//			temp=abs(W2)*122.23;
//			PID_setpoint(&PID2,temp);
//			temp=abs(W3)*122.23;
//			PID_setpoint(&PID3,temp);
//			temp=abs(W4)*122.23;
//			PID_setpoint(&PID4,temp);
         }
void stop()
{
//	TIM_SetCompare1(TIM3,0);		//设置占空比	   PC6
//	TIM_SetCompare2(TIM3,0);		//设置占空比	   PC7
//	TIM_SetCompare3(TIM3,0);		//设置占空比	   PC8
//	TIM_SetCompare4(TIM3,0);		//设置占空比	   PC9
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

