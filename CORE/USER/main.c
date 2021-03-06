/*
  PID调节经验总结（搜索资源）
    PID控制器参数选择的方法很多，例如试凑法、临界比例度法、扩充临界比例度法等。但是，对于PID控制而言，参数的选择始终是一件非常烦杂的工作，
	需要经过不断的调整才能得到较为满意的控制效果。依据经验，一般PID参数确定的步骤如下：
(1)确定比例系数Kp
    确定比例系数Kp时，首先去掉PID的积分项和微分项，可以令Ti=0、Td=0，使之成为纯比例调节。输入设定为系统允许输出最大值的60％～70％，
	比例系数Kp由0开始逐渐增大，直至系统出现振荡；再反过来，从此时的比例系数Kp逐渐减小，直至系统振荡消失。记录此时的比例系数Kp，
	设定PID的比例系数Kp为当前值的60％～70％。
(2)确定积分时间常数Ti
    比例系数Kp确定之后，设定一个较大的积分时间常数Ti，然后逐渐减小Ti，直至系统出现振荡，然后再反过来，逐渐增大Ti，直至系统振荡消失。
	记录此时的Ti，设定PID的积分时间常数Ti为当前值的 150％～180％。
(3)确定微分时间常数Td
    微分时间常数Td一般不用设定，为0即可，此时PID调节转换为PI调节。如果需要设定，则与确定Kp的方法相同，取不振荡时其值的30％。
(4)系统空载、带载联调
    对PID参数进行微调，直到满足性能要求。

	疑问1；按键为什么是key1==1有效呢？
*/
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "exti.h"
#include "wdg.h"
#include "pwm.h"
#include "pid.h"
#include "dac.h"
#include "lcd12864.h"
#include"led.h"
#include"exti.h"
#include"key.h"
#include"kinematics.h"
#include<stdlib.h>//产生随机数


#define Kp      0.15   //比例常数
#define Ti 		0.017   //积分常数
#define Td 		0.015   //微分常数
#define T  		0.02 //采样周期
#define Kpp 	Kp*(1+(T/Ti)+(Td/T)) //Kp*(1+(Td/T))
#define Ki      (Kp)*(1+(2*Td/T))
#define Kd		Kp*Td/T
 extern u8   table1[];
 extern u8   table2[];
 extern u8   table3[];
 PIDtypedef PID1;
 PIDtypedef PID2;
 PIDtypedef PID3;
 PIDtypedef PID4;

u8 start_flag=0;
 u16 period_TIM1= 0;//周期
 u16 duty_TIM1 = 0;//占空比
 u8 CollectFlag_TIM1= 1 ;	//2轮									  

u16 period_TIM4= 0;
u16 duty_TIM4 = 0;
u8 CollectFlag_TIM4 = 1; // 1轮

u16 period_TIM2= 0;
 u16 duty_TIM2= 0 ;
u8 CollectFlag_TIM2 = 1;   //3轮

 u16 period_TIM5= 0;
 u16 duty_TIM5= 0 ;
u8 CollectFlag_TIM5= 1 ;   //4轮
    u32 frequency1;
	u32 frequency2;
	u32 frequency3;
	u32 frequency4;	 
 u16 pwm1=0,pwm2=0,pwm3=0,pwm4=0;        //     

 u8 flag_lcd=0;//液晶屏幕更新标志
 u8 flag_bluetooth =0;//蓝牙验证状态      1：已发出验证信息  0：未发出验证信息
 u8 status_bluetooth=0;//蓝牙连接状态位   1：已连接			 0：未连接
 int main(void)
 {
	//u16 zhankongbi=0;
	//u8 i=0;
	u8 len ,t;             //         【【len即字符长度参数
 	SystemInit();
	delay_init(72);	     		//延时初始化
	NVIC_Configuration();  	//中断配置  中断分组2:2位抢占优先级，2位响应优先�     
	 init_LCD_IO() ;				   //初始化LCD控制引脚 PG4 5
	uart_init(9600);				//串口初始化

     
   	  printf("KPP=%f\r\n",Kpp);                     //  【【   向         串口输出当前PID参数
		  printf("Ki=%f\r\n",Ki);
		  printf("Kd=%f\r\n",Kd);
   lcd_init();					   //LCD显示
   LED_GPIO_Config();             // led 初始化
   	LED1=0;// LIGHT ON
   MOTOR_INIT();
   	Dac1_Init();				//DAC初始化
   	incPIDinit();                  //PID初始化 置零
    KEY_Init();
	EXTIX_Init();		 	//外部中断初始化
    
	TIM3_PWM_Init(100-1,72-1);   //PWM输出  频率：10kHZ  周期：100us	原来是 1000-1 72-1【【频率变高了，周期变短了，噪声就变小了
//	TIM_SetCompare1(TIM3,40);		//设置占空比	   PC6
//	TIM_SetCompare2(TIM3,100);		//设置占空比	   PC7
	//TIM_SetCompare3(TIM3,60);		//设置占空比	   PC8
	//TIM_SetCompare4(TIM3,800);		//设置占空比	   PC9
	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉             【【1m的频率所对应的周期应该是1us这里却是65536us
	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉            【【这里用一个计时器输出四个计时器输入
	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
    MOTOR_OUT(1,0,1,0,1,0,1,0);//转速全为正，速度都是0
	//PID_set(Kpp,Ki,Kd);
	PID_set(Kpp,Ki,Kd);					                            //【设定PID参数
	//0.24 Kp临界，取0.15 Ki 0.011临界，取 0.17	  Kd 0.05临界 ，取0.015

//  PID_setpoint(&PID1,500);       //调试时给轮子初始速度
//	PID_setpoint(&PID2,300);
//	PID_setpoint(&PID3,300);
//	PID_setpoint(&PID4,300);

	PIDperiodinit(40,36000-1);                                  
		  //设定PID采样周期 T=20ms	  72000 000/36 000 = 2 KHz	  和 T 对应
   //set_speed(3,3,3,3);
   TIM_Cmd(TIM6, ENABLE);  //使能TIMx 开启PID处理
   
   while(1)                                               //           
   {

   	if(USART_RX_STA&0x8000)	  //如果完成一次接收                    
		{	
		   TIM_Cmd(TIM6, DISABLE);  //	 关闭PID运算
		   stop(); //PID相关参数清零，并且小车停止运动				   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度,本控制系统应该len==15【【运动学参数】】  或者 2【【蓝牙模块参数】】
	//	printf("len=%d\r\n",len);
			printf("MCU_GET:");
			for(t=0;t<len;t++) //返回所以数值                 
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束             【【作用？？
			}
			printf("\r\n");//插入换行
			if(len==15)
		  {
	        //报文格式 ：x+正负号+x速度+y+正负号+y速度+w+正负号+w速度
            //例如     ：x+123y+000w+000，表示只有x方向速度的，其余均为零的
			table1[3]=USART_RX_BUF[1];table1[4]=USART_RX_BUF[2]; table1[5]=USART_RX_BUF[3];table1[6]=USART_RX_BUF[4];	//更新到LCD
			table2[3]=USART_RX_BUF[6];table2[4]=USART_RX_BUF[7]; table2[5]=USART_RX_BUF[8];table2[6]=USART_RX_BUF[9];
			table3[3]=USART_RX_BUF[11];table3[4]=USART_RX_BUF[12]; table3[5]=USART_RX_BUF[13];table3[6]=USART_RX_BUF[14];
			lcd_init();					   //LCD显示

			X=(USART_RX_BUF[2]-0x30)*100+(USART_RX_BUF[3]-0x30)*10+(USART_RX_BUF[4]-0x30) ;	  //得到Vx
			if(USART_RX_BUF[1]=='-')X=-X;
						
			Y=(USART_RX_BUF[7]-0x30)*100+(USART_RX_BUF[8]-0x30)*10+(USART_RX_BUF[9]-0x30) ;	 //得到Xy
		    if(USART_RX_BUF[6]=='-')Y=-Y;
					
			W=(USART_RX_BUF[12]-0x30)*100+(USART_RX_BUF[13]-0x30)*10+(USART_RX_BUF[14]-0x30) ; //得到w
			W=W/100.0;
		    if(USART_RX_BUF[11]=='-')W=-W;
					
			printf("X=%d\r\n",X); 
			printf("Y=%d\r\n",Y);
			printf("W=%f\r\n",W);

			kinematics(X,Y,W,&W1,&W2,&W3,&W4);
			printf("W1=%f\r\n",W1);
			printf("W2=%f\r\n",W2);
			printf("W3=%f\r\n",W3);
			printf("W4=%f\r\n",W4);
		//	LED0=!LED0;		  //LED翻转
             set_speed(W1,W2,W3,W4);
   			 TIM_Cmd(TIM6, ENABLE);  //使能TIMx 开启PID              
		   }

		   if(len==2)
		   {
		   if ((USART_RX_BUF[0]=='B') && (USART_RX_BUF[1]=='L') && (flag_bluetooth==1))
				 {  status_bluetooth=1 ;
				   flag_bluetooth=0;
				   printf("blue_OK\r\n");
				  }
		   }
	   USART_RX_STA=0;	  //数据处理完毕，清除状态寄存器，准备下组数据接收
	   len=0;
 }    
 }
 }


	/*	if(!CollectFlag_TIM4)		
	{	
		//printf("per=%d\r\n",period_TIM4);
	//	printf("%dKHz\r\n",1000000/period_TIM4);
			//CollectFlag_TIM4 = 1; 
			 }

		  		  if(!CollectFlag_TIM1)
			{
			//	printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//打印占空比
				//printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//频率   1000us/周期= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //打印周期
				printf("f2=%dKHz\r\n",frequency2);
				CollectFlag_TIM1 = 1;
			}
				*/
//	PCout(13)=0;
	
//	delay_ms(500);
	
//	PCout(13)=1;
	
//	delay_ms(500);

//	lcd_speed_update();

 /*
//zhankongbi+=50;
if(zhankongbi==1050)
{zhankongbi=50; TIM_SetCompare1(TIM3,zhankongbi);delay_ms(3000);}
//TIM_SetCompare1(TIM3,zhankongbi);		//设置占空比	   PC6
		   for(i=0;i<20;)
		   {
 		  if(!CollectFlag_TIM4)
			{
		//	speed4=4090/period_TIM4;
				printf("zhan= %d \r\n",zhankongbi);				//打印占空比
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//频率   1000us/周期= f
				printf("fre_TIM4 	= %dKHz\r\n",frequency1);
				printf("period_TIM4 = %dus\r\n",period_TIM4);  //打印周期
//				printf("speed4 =%d rad/s\r\n",speed4);
				CollectFlag_TIM4 = 1;
				i++;
			}
			}
		 delay_ms(2000);
	 */
	//		delay_ms(100);
	/*
 		  if(!CollectFlag_TIM1)
			{
				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//打印占空比
				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//频率   1000us/周期= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //打印周期
				CollectFlag_TIM1 = 1;
			}
						delay_ms(100);
 		  if(!CollectFlag_TIM2)
			{
				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//打印占空比
				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//频率   1000us/周期= f
				printf("period_TIM2 = %dus\r\n",period_TIM2);    //打印周期
				CollectFlag_TIM2 = 1;
			}
		      delay_ms(100);
 		  if(!CollectFlag_TIM5)
			{
				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//打印占空比
				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//频率   1000us/周期= f
				printf("period_TIM5 = %dus\r\n",period_TIM5);    //打印周期
				CollectFlag_TIM5 = 1;
			}
   			   */
//   }	    
//
// }
	
//	SystemInit();
//	delay_init(72);	     		//延时初始化
//	NVIC_Configuration();  	//中断配置  中断分组2:2位抢占优先级，2位响应优先级
//	uart_init(9600);				//串口初始化
//	incPIDinit();                  //PID初始化
//	TIM3_PWM_Init(1000-1,72-1);   //PWM输出  1KHZ周期
//	TIM_SetCompare1(TIM3,200);		//设置占空比
//	TIM_SetCompare2(TIM3,400);		//设置占空比
//	TIM_SetCompare3(TIM3,600);		//设置占空比
//	TIM_SetCompare4(TIM3,800);		//设置占空比
//	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
//	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
//	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
//	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwm输入初始化以1M的频率捕捉
//
////	PWM_Init(900,0);	 //不分频。PWM频率=72000/900=8Khz
//		while(1)
//	{	 
//			delay_ms(100);
// 		  if(!CollectFlag_TIM4)
//			{
//				printf("duty_TIM4  	= %d%% \r\n",duty_TIM4*100/period_TIM4);				//打印占空比
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//打印周期另一种叫法
//				printf("period_TIM4 = %dus\r\n",period_TIM4);    //打印周期
//				CollectFlag_TIM4 = 1;
//			}
//			delay_ms(100);
// 		  if(!CollectFlag_TIM1)
//			{
//				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//打印占空比
//				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//打印周期另一种叫法
//				printf("period_TIM1 = %dus\r\n",period_TIM1);    //打印周期
//				CollectFlag_TIM1 = 1;
//			}
//						delay_ms(100);
// 		  if(!CollectFlag_TIM2)
//			{
//				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//打印占空比
//				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//打印周期另一种叫法
//				printf("period_TIM2 = %dus\r\n",period_TIM2);    //打印周期
//				CollectFlag_TIM2 = 1;
//			}
//		      delay_ms(100);
// 		  if(!CollectFlag_TIM5)
//			{
//				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//打印占空比
//				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//打印周期另一种叫法
//				printf("period_TIM5 = %dus\r\n",period_TIM5);    //打印周期
//				CollectFlag_TIM5 = 1;
//			}
//		  
//
//	 }
//		 
 


