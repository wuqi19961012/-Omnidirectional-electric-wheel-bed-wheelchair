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
/*功能名称void MOTOR_INIT()
	描述      定义控制电机的4组引脚
*/
void MOTOR_INIT()
{
	   GPIO_InitTypeDef GPIO_InitStructure;
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);//使能D口时钟
	   	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5 
	                               | GPIO_Pin_6 | GPIO_Pin_7; //初始化GPIO
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //通用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOF,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
	    //初始化为零，防止开机电机转动	

}

/*功能名称void MOTOR_OUT(u8 direct1,u16 pwm1,u8 direct2,u16 pwm2,u8 direct3,pwm3,u8 direct4,pwm4)
	描述      控制4个电机的转向和转速，如果directx==1,那么电机转向输出为10；否则电机输出为01
*/
void MOTOR_OUT(u8 direct1,u16 speed1,u8 direct2,u16 speed2,u8 direct3,u16 speed3,u8 direct4,u16 speed4)
{
//	  motor1_out0=  direct1;	//电机1
//	  motor1_out1= !direct1;
	  if(direct1==0x01) { motor1_out0=1;motor1_out1=0;}
	  else 				{motor1_out0=0;motor1_out1=1;}
//	  motor2_out0=  direct2;	//电机2
//	  motor2_out1= !direct2;
	  if(direct2==0x01) { motor2_out0=1;motor2_out1=0;}
	  else 				{motor2_out0=0;motor2_out1=1;}
//	  motor3_out0=  direct3;	//电机3
//	  motor3_out1= !direct3;
	  if(direct3==0x01) { motor3_out0=1;motor3_out1=0;}
	  else 				{motor3_out0=0;motor3_out1=1;}
//	  motor4_out0=  direct4;	//电机4
//	  motor4_out1= !direct4;
 	  if(direct4==0x01) { motor4_out0=1;motor4_out1=0;}
	  else 				{motor4_out0=0;motor4_out1=1;}
  	TIM_SetCompare1(TIM3,speed1);		//设置占空比
	TIM_SetCompare2(TIM3,speed2);		//设置占空比
	TIM_SetCompare3(TIM3,speed3);		//设置占空比
	TIM_SetCompare4(TIM3,speed4);		//设置占空比
}

 void TIM_SetCompare(u16 val1,u16 val2,u16 val3,u16 val4)
 {
   	TIM_SetCompare1(TIM3,val1);		//设置占空比
	TIM_SetCompare2(TIM3,val2);		//设置占空比
	TIM_SetCompare3(TIM3,val3);		//设置占空比
	TIM_SetCompare4(TIM3,val4);		//设置占空比
 
 }
/*功能名称IM3_PWM_Init(u16 arr,u16 psc)
	描述      TIM3产生四路PWM
*/
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
	
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //Timer3全映射 GPIOC-> 6,7,8,9                                                                    	 //用于TIM3的CH2输出的PWM通过该LED显示
 
   //设置该引脚为复用输出功能,输出TIM3 CH1 CH2 CH3 CH4 的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //初始化GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//默认电机使能端状态：不使能 

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  这里是72分频，那么时钟频率就是1M 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR1上的预装载寄存器
	
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR3上的预装载寄存器
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR4上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
 

}



/*功能名称TIM4_PWMINPUT_INIT(u16 arr,u16 psc)
  描述      PWM输入初始化*/

void TIM4_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM的初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;			//中断配置
	TIM_ICInitTypeDef  TIM4_ICInitStructure;		 //TIM4  PWM配置结构体
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO口配置结构体
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);     //Open TIM4 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //open gpioB clock
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;             //GPIO 7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	
	/*配置中断优先级*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM4_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:过滤
 
  TIM_PWMIConfig(TIM4, &TIM4_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);//启动定时器的被动触发                                       
  TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //中断配置
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
  TIM_Cmd(TIM4, ENABLE);    
}


void TIM4_IRQHandler(void)
{

//	if(CollectFlag_TIM4)
//	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
			{	
				duty_TIM4    =   TIM_GetCapture1(TIM4); 	 //采集占空比		
			    //period_TIM4	=	TIM_GetCapture2(TIM4);     //采集周期
		//		frequency1=1000000/period_TIM4	;		  //此处可能会占用一点时间
	//	printf("per=%d\r\n",period_TIM4);
	if  (TIM_GetCapture2(TIM4)>600)	 period_TIM4	=	TIM_GetCapture2(TIM4);
				CollectFlag_TIM4 = 0;
				
		//	}
			
			
	}	
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}


/*功能名称TIM1_PWMINPUT_INIT(u16 arr,u16 psc)
  描述      PWM输入初始化*/
 
void TIM1_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM的初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;			//中断配置
	TIM_ICInitTypeDef  TIM1_ICInitStructure;		 //PWM配置结构体
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO口配置结构体
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);     //Open TIM1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  //open gpioE clock
   GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1完全重映射  TIM1_CH2->PE11	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //GPIO 11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	
	/*配置中断优先级*/
  NVIC_InitStructure.NVIC_IRQChannel =  TIM1_CC_IRQn;   //TIM1捕获中断                      
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM1_ICInitStructure.TIM_ICFilter = 0x03;   //Filter:过滤
 
  TIM_PWMIConfig(TIM1, &TIM1_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//启动定时器的被动触发                                       
 // TIM_ITConfig(TIM1, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //中断配置
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); //通道2 捕获中断打开
  //TIM_ClearITPendingBit(TIM1, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
  TIM_Cmd(TIM1, ENABLE);    
}


void TIM1_CC_IRQHandler(void)
{

//	if(CollectFlag_TIM1)
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
			{	
				duty_TIM1    =   TIM_GetCapture1(TIM1); 	 //采集占空比		
			  // period_TIM1	=	TIM_GetCapture2(TIM1);     //采集周期
			   if  (TIM_GetCapture2(TIM1)>600)	 period_TIM1	=	TIM_GetCapture2(TIM1);
				CollectFlag_TIM1 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}

 
/*功能名称TIM2_PWMINPUT_INIT(u16 arr,u16 psc)
  描述      PWM输入初始化*/

void TIM2_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM的初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;			//中断配置
	TIM_ICInitTypeDef  TIM2_ICInitStructure;		 //TIM2  PWM配置结构体
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO口配置结构体
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);     //Open TIM2 clock
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //open gpioB clock
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);          //关闭JTAG
 	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2完全重映射  TIM2_CH2->PB3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             //GPIO 3
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;	  //浮空输入 上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	
	/*配置中断优先级*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM2_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:过滤
 
  TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//启动定时器的被动触发                                       
  TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //中断配置
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
  TIM_Cmd(TIM2, ENABLE);    
}


void TIM2_IRQHandler(void)
{

	//if(CollectFlag_TIM2)
	{
		if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
			{	
				duty_TIM2    =   TIM_GetCapture1(TIM2); 	 //采集占空比		
			   // period_TIM2	=	TIM_GetCapture2(TIM2);     //采集周期
			   if  (TIM_GetCapture2(TIM2)>600)	 period_TIM2	=	TIM_GetCapture2(TIM2);
				CollectFlag_TIM2 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}

 /*功能名称TIM5_PWMINPUT_INIT(u16 arr,u16 psc)
  描述      PWM输入初始化*/

void TIM5_PWMINPUT_INIT(u16 arr,u16 psc)
{
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM的初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;			//中断配置
	TIM_ICInitTypeDef  TIM5_ICInitStructure;		 //TIM4  PWM配置结构体
	GPIO_InitTypeDef GPIO_InitStructure;			 //IO口配置结构体
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);     //Open TIM5 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //open gpioB clock
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;             //GPIO 1
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;	  //上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值  
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	
	/*配置中断优先级*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;                     
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;                   
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
  TIM5_ICInitStructure.TIM_ICFilter = 0x3;   //Filter:过滤
 
  TIM_PWMIConfig(TIM5, &TIM5_ICInitStructure);     //PWM输入配置           
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);     //选择有效输入端        
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);  //配置为主从复位模式
  TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);//启动定时器的被动触发                                       
  TIM_ITConfig(TIM5, TIM_IT_CC2|TIM_IT_Update, ENABLE);          //中断配置
  TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
  TIM_Cmd(TIM5, ENABLE);    
}


void TIM5_IRQHandler(void)
{

//	if(CollectFlag_TIM5)
	{
		if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
			{	
				duty_TIM5    =   TIM_GetCapture1(TIM5); 	 //采集占空比		
			//    period_TIM5	=	TIM_GetCapture2(TIM5);     //采集周期
			if  (TIM_GetCapture2(TIM5)>600)	 period_TIM5	=	TIM_GetCapture2(TIM5);
				CollectFlag_TIM5 = 0;
			}
			
			
	}	
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
}

