/*
  PID���ھ����ܽᣨ������Դ��
    PID����������ѡ��ķ����ܶ࣬�����Դշ����ٽ�����ȷ��������ٽ�����ȷ��ȡ����ǣ�����PID���ƶ��ԣ�������ѡ��ʼ����һ���ǳ����ӵĹ�����
	��Ҫ�������ϵĵ������ܵõ���Ϊ����Ŀ���Ч�������ݾ��飬һ��PID����ȷ���Ĳ������£�
(1)ȷ������ϵ��Kp
    ȷ������ϵ��Kpʱ������ȥ��PID�Ļ������΢���������Ti=0��Td=0��ʹ֮��Ϊ���������ڡ������趨Ϊϵͳ����������ֵ��60����70����
	����ϵ��Kp��0��ʼ������ֱ��ϵͳ�����񵴣��ٷ��������Ӵ�ʱ�ı���ϵ��Kp�𽥼�С��ֱ��ϵͳ����ʧ����¼��ʱ�ı���ϵ��Kp��
	�趨PID�ı���ϵ��KpΪ��ǰֵ��60����70����
(2)ȷ������ʱ�䳣��Ti
    ����ϵ��Kpȷ��֮���趨һ���ϴ�Ļ���ʱ�䳣��Ti��Ȼ���𽥼�СTi��ֱ��ϵͳ�����񵴣�Ȼ���ٷ�������������Ti��ֱ��ϵͳ����ʧ��
	��¼��ʱ��Ti���趨PID�Ļ���ʱ�䳣��TiΪ��ǰֵ�� 150����180����
(3)ȷ��΢��ʱ�䳣��Td
    ΢��ʱ�䳣��Tdһ�㲻���趨��Ϊ0���ɣ���ʱPID����ת��ΪPI���ڡ������Ҫ�趨������ȷ��Kp�ķ�����ͬ��ȡ����ʱ��ֵ��30����
(4)ϵͳ���ء���������
    ��PID��������΢����ֱ����������Ҫ��

	����1������Ϊʲô��key1==1��Ч�أ�
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
#include<stdlib.h>//���������


#define Kp      0.15   //��������
#define Ti 		0.017   //���ֳ���
#define Td 		0.015   //΢�ֳ���
#define T  		0.02 //��������
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
 u16 period_TIM1= 0;//����
 u16 duty_TIM1 = 0;//ռ�ձ�
 u8 CollectFlag_TIM1= 1 ;	//2��									  

u16 period_TIM4= 0;
u16 duty_TIM4 = 0;
u8 CollectFlag_TIM4 = 1; // 1��

u16 period_TIM2= 0;
 u16 duty_TIM2= 0 ;
u8 CollectFlag_TIM2 = 1;   //3��

 u16 period_TIM5= 0;
 u16 duty_TIM5= 0 ;
u8 CollectFlag_TIM5= 1 ;   //4��
    u32 frequency1;
	u32 frequency2;
	u32 frequency3;
	u32 frequency4;	 
 u16 pwm1=0,pwm2=0,pwm3=0,pwm4=0;        //     

 u8 flag_lcd=0;//Һ����Ļ���±�־
 u8 flag_bluetooth =0;//������֤״̬      1���ѷ�����֤��Ϣ  0��δ������֤��Ϣ
 u8 status_bluetooth=0;//��������״̬λ   1��������			 0��δ����
 int main(void)
 {
	//u16 zhankongbi=0;
	//u8 i=0;
	u8 len ,t;             //         ����len���ַ����Ȳ���
 	SystemInit();
	delay_init(72);	     		//��ʱ��ʼ��
	NVIC_Configuration();  	//�ж�����  �жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ     
	 init_LCD_IO() ;				   //��ʼ��LCD�������� PG4 5
	uart_init(9600);				//���ڳ�ʼ��

     
   	  printf("KPP=%f\r\n",Kpp);                     //  ����   ��         ���������ǰPID����
		  printf("Ki=%f\r\n",Ki);
		  printf("Kd=%f\r\n",Kd);
   lcd_init();					   //LCD��ʾ
   LED_GPIO_Config();             // led ��ʼ��
   	LED1=0;// LIGHT ON
   MOTOR_INIT();
   	Dac1_Init();				//DAC��ʼ��
   	incPIDinit();                  //PID��ʼ�� ����
    KEY_Init();
	EXTIX_Init();		 	//�ⲿ�жϳ�ʼ��
    
	TIM3_PWM_Init(100-1,72-1);   //PWM���  Ƶ�ʣ�10kHZ  ���ڣ�100us	ԭ���� 1000-1 72-1����Ƶ�ʱ���ˣ����ڱ���ˣ������ͱ�С��
//	TIM_SetCompare1(TIM3,40);		//����ռ�ձ�	   PC6
//	TIM_SetCompare2(TIM3,100);		//����ռ�ձ�	   PC7
	//TIM_SetCompare3(TIM3,60);		//����ռ�ձ�	   PC8
	//TIM_SetCompare4(TIM3,800);		//����ռ�ձ�	   PC9
	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽             ����1m��Ƶ������Ӧ������Ӧ����1us����ȴ��65536us
	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽            ����������һ����ʱ������ĸ���ʱ������
	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
    MOTOR_OUT(1,0,1,0,1,0,1,0);//ת��ȫΪ�����ٶȶ���0
	//PID_set(Kpp,Ki,Kd);
	PID_set(Kpp,Ki,Kd);					                            //���趨PID����
	//0.24 Kp�ٽ磬ȡ0.15 Ki 0.011�ٽ磬ȡ 0.17	  Kd 0.05�ٽ� ��ȡ0.015

//  PID_setpoint(&PID1,500);       //����ʱ�����ӳ�ʼ�ٶ�
//	PID_setpoint(&PID2,300);
//	PID_setpoint(&PID3,300);
//	PID_setpoint(&PID4,300);

	PIDperiodinit(40,36000-1);                                  
		  //�趨PID�������� T=20ms	  72000 000/36 000 = 2 KHz	  �� T ��Ӧ
   //set_speed(3,3,3,3);
   TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx ����PID����
   
   while(1)                                               //           
   {

   	if(USART_RX_STA&0x8000)	  //������һ�ν���                    
		{	
		   TIM_Cmd(TIM6, DISABLE);  //	 �ر�PID����
		   stop(); //PID��ز������㣬����С��ֹͣ�˶�				   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���,������ϵͳӦ��len==15�����˶�ѧ��������  ���� 2��������ģ���������
	//	printf("len=%d\r\n",len);
			printf("MCU_GET:");
			for(t=0;t<len;t++) //����������ֵ                 
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���             �������ã���
			}
			printf("\r\n");//���뻻��
			if(len==15)
		  {
	        //���ĸ�ʽ ��x+������+x�ٶ�+y+������+y�ٶ�+w+������+w�ٶ�
            //����     ��x+123y+000w+000����ʾֻ��x�����ٶȵģ������Ϊ���
			table1[3]=USART_RX_BUF[1];table1[4]=USART_RX_BUF[2]; table1[5]=USART_RX_BUF[3];table1[6]=USART_RX_BUF[4];	//���µ�LCD
			table2[3]=USART_RX_BUF[6];table2[4]=USART_RX_BUF[7]; table2[5]=USART_RX_BUF[8];table2[6]=USART_RX_BUF[9];
			table3[3]=USART_RX_BUF[11];table3[4]=USART_RX_BUF[12]; table3[5]=USART_RX_BUF[13];table3[6]=USART_RX_BUF[14];
			lcd_init();					   //LCD��ʾ

			X=(USART_RX_BUF[2]-0x30)*100+(USART_RX_BUF[3]-0x30)*10+(USART_RX_BUF[4]-0x30) ;	  //�õ�Vx
			if(USART_RX_BUF[1]=='-')X=-X;
						
			Y=(USART_RX_BUF[7]-0x30)*100+(USART_RX_BUF[8]-0x30)*10+(USART_RX_BUF[9]-0x30) ;	 //�õ�Xy
		    if(USART_RX_BUF[6]=='-')Y=-Y;
					
			W=(USART_RX_BUF[12]-0x30)*100+(USART_RX_BUF[13]-0x30)*10+(USART_RX_BUF[14]-0x30) ; //�õ�w
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
		//	LED0=!LED0;		  //LED��ת
             set_speed(W1,W2,W3,W4);
   			 TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx ����PID              
		   }

		   if(len==2)
		   {
		   if ((USART_RX_BUF[0]=='B') && (USART_RX_BUF[1]=='L') && (flag_bluetooth==1))
				 {  status_bluetooth=1 ;
				   flag_bluetooth=0;
				   printf("blue_OK\r\n");
				  }
		   }
	   USART_RX_STA=0;	  //���ݴ�����ϣ����״̬�Ĵ�����׼���������ݽ���
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
			//	printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//��ӡռ�ձ�
				//printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//Ƶ��   1000us/����= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //��ӡ����
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
//TIM_SetCompare1(TIM3,zhankongbi);		//����ռ�ձ�	   PC6
		   for(i=0;i<20;)
		   {
 		  if(!CollectFlag_TIM4)
			{
		//	speed4=4090/period_TIM4;
				printf("zhan= %d \r\n",zhankongbi);				//��ӡռ�ձ�
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//Ƶ��   1000us/����= f
				printf("fre_TIM4 	= %dKHz\r\n",frequency1);
				printf("period_TIM4 = %dus\r\n",period_TIM4);  //��ӡ����
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
				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//��ӡռ�ձ�
				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//Ƶ��   1000us/����= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //��ӡ����
				CollectFlag_TIM1 = 1;
			}
						delay_ms(100);
 		  if(!CollectFlag_TIM2)
			{
				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//��ӡռ�ձ�
				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//Ƶ��   1000us/����= f
				printf("period_TIM2 = %dus\r\n",period_TIM2);    //��ӡ����
				CollectFlag_TIM2 = 1;
			}
		      delay_ms(100);
 		  if(!CollectFlag_TIM5)
			{
				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//��ӡռ�ձ�
				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//Ƶ��   1000us/����= f
				printf("period_TIM5 = %dus\r\n",period_TIM5);    //��ӡ����
				CollectFlag_TIM5 = 1;
			}
   			   */
//   }	    
//
// }
	
//	SystemInit();
//	delay_init(72);	     		//��ʱ��ʼ��
//	NVIC_Configuration();  	//�ж�����  �жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
//	uart_init(9600);				//���ڳ�ʼ��
//	incPIDinit();                  //PID��ʼ��
//	TIM3_PWM_Init(1000-1,72-1);   //PWM���  1KHZ����
//	TIM_SetCompare1(TIM3,200);		//����ռ�ձ�
//	TIM_SetCompare2(TIM3,400);		//����ռ�ձ�
//	TIM_SetCompare3(TIM3,600);		//����ռ�ձ�
//	TIM_SetCompare4(TIM3,800);		//����ռ�ձ�
//	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
//	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
//	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
//	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwm�����ʼ����1M��Ƶ�ʲ�׽
//
////	PWM_Init(900,0);	 //����Ƶ��PWMƵ��=72000/900=8Khz
//		while(1)
//	{	 
//			delay_ms(100);
// 		  if(!CollectFlag_TIM4)
//			{
//				printf("duty_TIM4  	= %d%% \r\n",duty_TIM4*100/period_TIM4);				//��ӡռ�ձ�
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//��ӡ������һ�ֽз�
//				printf("period_TIM4 = %dus\r\n",period_TIM4);    //��ӡ����
//				CollectFlag_TIM4 = 1;
//			}
//			delay_ms(100);
// 		  if(!CollectFlag_TIM1)
//			{
//				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//��ӡռ�ձ�
//				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//��ӡ������һ�ֽз�
//				printf("period_TIM1 = %dus\r\n",period_TIM1);    //��ӡ����
//				CollectFlag_TIM1 = 1;
//			}
//						delay_ms(100);
// 		  if(!CollectFlag_TIM2)
//			{
//				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//��ӡռ�ձ�
//				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//��ӡ������һ�ֽз�
//				printf("period_TIM2 = %dus\r\n",period_TIM2);    //��ӡ����
//				CollectFlag_TIM2 = 1;
//			}
//		      delay_ms(100);
// 		  if(!CollectFlag_TIM5)
//			{
//				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//��ӡռ�ձ�
//				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//��ӡ������һ�ֽз�
//				printf("period_TIM5 = %dus\r\n",period_TIM5);    //��ӡ����
//				CollectFlag_TIM5 = 1;
//			}
//		  
//
//	 }
//		 
 


