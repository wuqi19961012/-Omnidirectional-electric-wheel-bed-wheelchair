/*
  PIDµ÷½Ú¾­Ñé×Ü½á£¨ËÑË÷×ÊÔ´£©
    PID¿ØÖÆÆ÷²ÎÊýÑ¡ÔñµÄ·½·¨ºÜ¶à£¬ÀýÈçÊÔ´Õ·¨¡¢ÁÙ½ç±ÈÀý¶È·¨¡¢À©³äÁÙ½ç±ÈÀý¶È·¨µÈ¡£µ«ÊÇ£¬¶ÔÓÚPID¿ØÖÆ¶øÑÔ£¬²ÎÊýµÄÑ¡ÔñÊ¼ÖÕÊÇÒ»¼þ·Ç³£·³ÔÓµÄ¹¤×÷£¬
	ÐèÒª¾­¹ý²»¶ÏµÄµ÷Õû²ÅÄÜµÃµ½½ÏÎªÂúÒâµÄ¿ØÖÆÐ§¹û¡£ÒÀ¾Ý¾­Ñé£¬Ò»°ãPID²ÎÊýÈ·¶¨µÄ²½ÖèÈçÏÂ£º
(1)È·¶¨±ÈÀýÏµÊýKp
    È·¶¨±ÈÀýÏµÊýKpÊ±£¬Ê×ÏÈÈ¥µôPIDµÄ»ý·ÖÏîºÍÎ¢·ÖÏî£¬¿ÉÒÔÁîTi=0¡¢Td=0£¬Ê¹Ö®³ÉÎª´¿±ÈÀýµ÷½Ú¡£ÊäÈëÉè¶¨ÎªÏµÍ³ÔÊÐíÊä³ö×î´óÖµµÄ60£¥¡«70£¥£¬
	±ÈÀýÏµÊýKpÓÉ0¿ªÊ¼Öð½¥Ôö´ó£¬Ö±ÖÁÏµÍ³³öÏÖÕñµ´£»ÔÙ·´¹ýÀ´£¬´Ó´ËÊ±µÄ±ÈÀýÏµÊýKpÖð½¥¼õÐ¡£¬Ö±ÖÁÏµÍ³Õñµ´ÏûÊ§¡£¼ÇÂ¼´ËÊ±µÄ±ÈÀýÏµÊýKp£¬
	Éè¶¨PIDµÄ±ÈÀýÏµÊýKpÎªµ±Ç°ÖµµÄ60£¥¡«70£¥¡£
(2)È·¶¨»ý·ÖÊ±¼ä³£ÊýTi
    ±ÈÀýÏµÊýKpÈ·¶¨Ö®ºó£¬Éè¶¨Ò»¸ö½Ï´óµÄ»ý·ÖÊ±¼ä³£ÊýTi£¬È»ºóÖð½¥¼õÐ¡Ti£¬Ö±ÖÁÏµÍ³³öÏÖÕñµ´£¬È»ºóÔÙ·´¹ýÀ´£¬Öð½¥Ôö´óTi£¬Ö±ÖÁÏµÍ³Õñµ´ÏûÊ§¡£
	¼ÇÂ¼´ËÊ±µÄTi£¬Éè¶¨PIDµÄ»ý·ÖÊ±¼ä³£ÊýTiÎªµ±Ç°ÖµµÄ 150£¥¡«180£¥¡£
(3)È·¶¨Î¢·ÖÊ±¼ä³£ÊýTd
    Î¢·ÖÊ±¼ä³£ÊýTdÒ»°ã²»ÓÃÉè¶¨£¬Îª0¼´¿É£¬´ËÊ±PIDµ÷½Ú×ª»»ÎªPIµ÷½Ú¡£Èç¹ûÐèÒªÉè¶¨£¬ÔòÓëÈ·¶¨KpµÄ·½·¨ÏàÍ¬£¬È¡²»Õñµ´Ê±ÆäÖµµÄ30£¥¡£
(4)ÏµÍ³¿ÕÔØ¡¢´øÔØÁªµ÷
    ¶ÔPID²ÎÊý½øÐÐÎ¢µ÷£¬Ö±µ½Âú×ãÐÔÄÜÒªÇó¡£

	ÒÉÎÊ1£»°´¼üÎªÊ²Ã´ÊÇkey1==1ÓÐÐ§ÄØ£¿
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
#include<stdlib.h>//²úÉúËæ»úÊý


#define Kp      0.15   //±ÈÀý³£Êý
#define Ti 		0.017   //»ý·Ö³£Êý
#define Td 		0.015   //Î¢·Ö³£Êý
#define T  		0.02 //²ÉÑùÖÜÆÚ
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
 u16 period_TIM1= 0;//ÖÜÆÚ
 u16 duty_TIM1 = 0;//Õ¼¿Õ±È
 u8 CollectFlag_TIM1= 1 ;	//2ÂÖ									  

u16 period_TIM4= 0;
u16 duty_TIM4 = 0;
u8 CollectFlag_TIM4 = 1; // 1ÂÖ

u16 period_TIM2= 0;
 u16 duty_TIM2= 0 ;
u8 CollectFlag_TIM2 = 1;   //3ÂÖ

 u16 period_TIM5= 0;
 u16 duty_TIM5= 0 ;
u8 CollectFlag_TIM5= 1 ;   //4ÂÖ
    u32 frequency1;
	u32 frequency2;
	u32 frequency3;
	u32 frequency4;	 
 u16 pwm1=0,pwm2=0,pwm3=0,pwm4=0;        //     

 u8 flag_lcd=0;//Òº¾§ÆÁÄ»¸üÐÂ±êÖ¾
 u8 flag_bluetooth =0;//À¶ÑÀÑéÖ¤×´Ì¬      1£ºÒÑ·¢³öÑéÖ¤ÐÅÏ¢  0£ºÎ´·¢³öÑéÖ¤ÐÅÏ¢
 u8 status_bluetooth=0;//À¶ÑÀÁ¬½Ó×´Ì¬Î»   1£ºÒÑÁ¬½Ó			 0£ºÎ´Á¬½Ó
 int main(void)
 {
	//u16 zhankongbi=0;
	//u8 i=0;
	u8 len ,t;             //         ¡¾¡¾len¼´×Ö·û³¤¶È²ÎÊý
 	SystemInit();
	delay_init(72);	     		//ÑÓÊ±³õÊ¼»¯
	NVIC_Configuration();  	//ÖÐ¶ÏÅäÖÃ  ÖÐ¶Ï·Ö×é2:2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼     
	 init_LCD_IO() ;				   //³õÊ¼»¯LCD¿ØÖÆÒý½Å PG4 5
	uart_init(9600);				//´®¿Ú³õÊ¼»¯

     
   	  printf("KPP=%f\r\n",Kpp);                     //  ¡¾¡¾   Ïò         ´®¿ÚÊä³öµ±Ç°PID²ÎÊý
		  printf("Ki=%f\r\n",Ki);
		  printf("Kd=%f\r\n",Kd);
   lcd_init();					   //LCDÏÔÊ¾
   LED_GPIO_Config();             // led ³õÊ¼»¯
   	LED1=0;// LIGHT ON
   MOTOR_INIT();
   	Dac1_Init();				//DAC³õÊ¼»¯
   	incPIDinit();                  //PID³õÊ¼»¯ ÖÃÁã
    KEY_Init();
	EXTIX_Init();		 	//Íâ²¿ÖÐ¶Ï³õÊ¼»¯
    
	TIM3_PWM_Init(100-1,72-1);   //PWMÊä³ö  ÆµÂÊ£º10kHZ  ÖÜÆÚ£º100us	Ô­À´ÊÇ 1000-1 72-1¡¾¡¾ÆµÂÊ±ä¸ßÁË£¬ÖÜÆÚ±ä¶ÌÁË£¬ÔëÉù¾Í±äÐ¡ÁË
//	TIM_SetCompare1(TIM3,40);		//ÉèÖÃÕ¼¿Õ±È	   PC6
//	TIM_SetCompare2(TIM3,100);		//ÉèÖÃÕ¼¿Õ±È	   PC7
	//TIM_SetCompare3(TIM3,60);		//ÉèÖÃÕ¼¿Õ±È	   PC8
	//TIM_SetCompare4(TIM3,800);		//ÉèÖÃÕ¼¿Õ±È	   PC9
	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½             ¡¾¡¾1mµÄÆµÂÊËù¶ÔÓ¦µÄÖÜÆÚÓ¦¸ÃÊÇ1usÕâÀïÈ´ÊÇ65536us
	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½            ¡¾¡¾ÕâÀïÓÃÒ»¸ö¼ÆÊ±Æ÷Êä³öËÄ¸ö¼ÆÊ±Æ÷ÊäÈë
	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
    MOTOR_OUT(1,0,1,0,1,0,1,0);//×ªËÙÈ«ÎªÕý£¬ËÙ¶È¶¼ÊÇ0
	//PID_set(Kpp,Ki,Kd);
	PID_set(Kpp,Ki,Kd);					                            //¡¾Éè¶¨PID²ÎÊý
	//0.24 KpÁÙ½ç£¬È¡0.15 Ki 0.011ÁÙ½ç£¬È¡ 0.17	  Kd 0.05ÁÙ½ç £¬È¡0.015

//  PID_setpoint(&PID1,500);       //µ÷ÊÔÊ±¸øÂÖ×Ó³õÊ¼ËÙ¶È
//	PID_setpoint(&PID2,300);
//	PID_setpoint(&PID3,300);
//	PID_setpoint(&PID4,300);

	PIDperiodinit(40,36000-1);                                  
		  //Éè¶¨PID²ÉÑùÖÜÆÚ T=20ms	  72000 000/36 000 = 2 KHz	  ºÍ T ¶ÔÓ¦
   //set_speed(3,3,3,3);
   TIM_Cmd(TIM6, ENABLE);  //Ê¹ÄÜTIMx ¿ªÆôPID´¦Àí
   
   while(1)                                               //           
   {

   	if(USART_RX_STA&0x8000)	  //Èç¹ûÍê³ÉÒ»´Î½ÓÊÕ                    
		{	
		   TIM_Cmd(TIM6, DISABLE);  //	 ¹Ø±ÕPIDÔËËã
		   stop(); //PIDÏà¹Ø²ÎÊýÇåÁã£¬²¢ÇÒÐ¡³µÍ£Ö¹ÔË¶¯				   
			len=USART_RX_STA&0x3fff;//µÃµ½´Ë´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È,±¾¿ØÖÆÏµÍ³Ó¦¸Ãlen==15¡¾¡¾ÔË¶¯Ñ§²ÎÊý¡¿¡¿  »òÕß 2¡¾¡¾À¶ÑÀÄ£¿é²ÎÊý¡¿¡¿
	//	printf("len=%d\r\n",len);
			printf("MCU_GET:");
			for(t=0;t<len;t++) //·µ»ØËùÒÔÊýÖµ                 
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//Ïò´®¿Ú1·¢ËÍÊý¾Ý
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//µÈ´ý·¢ËÍ½áÊø             ¡¾¡¾×÷ÓÃ£¿£¿
			}
			printf("\r\n");//²åÈë»»ÐÐ
			if(len==15)
		  {
	        //±¨ÎÄ¸ñÊ½ £ºx+Õý¸ººÅ+xËÙ¶È+y+Õý¸ººÅ+yËÙ¶È+w+Õý¸ººÅ+wËÙ¶È
            //ÀýÈç     £ºx+123y+000w+000£¬±íÊ¾Ö»ÓÐx·½ÏòËÙ¶ÈµÄ£¬ÆäÓà¾ùÎªÁãµÄ
			table1[3]=USART_RX_BUF[1];table1[4]=USART_RX_BUF[2]; table1[5]=USART_RX_BUF[3];table1[6]=USART_RX_BUF[4];	//¸üÐÂµ½LCD
			table2[3]=USART_RX_BUF[6];table2[4]=USART_RX_BUF[7]; table2[5]=USART_RX_BUF[8];table2[6]=USART_RX_BUF[9];
			table3[3]=USART_RX_BUF[11];table3[4]=USART_RX_BUF[12]; table3[5]=USART_RX_BUF[13];table3[6]=USART_RX_BUF[14];
			lcd_init();					   //LCDÏÔÊ¾

			X=(USART_RX_BUF[2]-0x30)*100+(USART_RX_BUF[3]-0x30)*10+(USART_RX_BUF[4]-0x30) ;	  //µÃµ½Vx
			if(USART_RX_BUF[1]=='-')X=-X;
						
			Y=(USART_RX_BUF[7]-0x30)*100+(USART_RX_BUF[8]-0x30)*10+(USART_RX_BUF[9]-0x30) ;	 //µÃµ½Xy
		    if(USART_RX_BUF[6]=='-')Y=-Y;
					
			W=(USART_RX_BUF[12]-0x30)*100+(USART_RX_BUF[13]-0x30)*10+(USART_RX_BUF[14]-0x30) ; //µÃµ½w
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
		//	LED0=!LED0;		  //LED·­×ª
             set_speed(W1,W2,W3,W4);
   			 TIM_Cmd(TIM6, ENABLE);  //Ê¹ÄÜTIMx ¿ªÆôPID              
		   }

		   if(len==2)
		   {
		   if ((USART_RX_BUF[0]=='B') && (USART_RX_BUF[1]=='L') && (flag_bluetooth==1))
				 {  status_bluetooth=1 ;
				   flag_bluetooth=0;
				   printf("blue_OK\r\n");
				  }
		   }
	   USART_RX_STA=0;	  //Êý¾Ý´¦ÀíÍê±Ï£¬Çå³ý×´Ì¬¼Ä´æÆ÷£¬×¼±¸ÏÂ×éÊý¾Ý½ÓÊÕ
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
			//	printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//´òÓ¡Õ¼¿Õ±È
				//printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//ÆµÂÊ   1000us/ÖÜÆÚ= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //´òÓ¡ÖÜÆÚ
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
//TIM_SetCompare1(TIM3,zhankongbi);		//ÉèÖÃÕ¼¿Õ±È	   PC6
		   for(i=0;i<20;)
		   {
 		  if(!CollectFlag_TIM4)
			{
		//	speed4=4090/period_TIM4;
				printf("zhan= %d \r\n",zhankongbi);				//´òÓ¡Õ¼¿Õ±È
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//ÆµÂÊ   1000us/ÖÜÆÚ= f
				printf("fre_TIM4 	= %dKHz\r\n",frequency1);
				printf("period_TIM4 = %dus\r\n",period_TIM4);  //´òÓ¡ÖÜÆÚ
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
				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//´òÓ¡Õ¼¿Õ±È
				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//ÆµÂÊ   1000us/ÖÜÆÚ= f
				printf("period_TIM1 = %dus\r\n",period_TIM1);    //´òÓ¡ÖÜÆÚ
				CollectFlag_TIM1 = 1;
			}
						delay_ms(100);
 		  if(!CollectFlag_TIM2)
			{
				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//´òÓ¡Õ¼¿Õ±È
				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//ÆµÂÊ   1000us/ÖÜÆÚ= f
				printf("period_TIM2 = %dus\r\n",period_TIM2);    //´òÓ¡ÖÜÆÚ
				CollectFlag_TIM2 = 1;
			}
		      delay_ms(100);
 		  if(!CollectFlag_TIM5)
			{
				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//´òÓ¡Õ¼¿Õ±È
				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//ÆµÂÊ   1000us/ÖÜÆÚ= f
				printf("period_TIM5 = %dus\r\n",period_TIM5);    //´òÓ¡ÖÜÆÚ
				CollectFlag_TIM5 = 1;
			}
   			   */
//   }	    
//
// }
	
//	SystemInit();
//	delay_init(72);	     		//ÑÓÊ±³õÊ¼»¯
//	NVIC_Configuration();  	//ÖÐ¶ÏÅäÖÃ  ÖÐ¶Ï·Ö×é2:2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼¶
//	uart_init(9600);				//´®¿Ú³õÊ¼»¯
//	incPIDinit();                  //PID³õÊ¼»¯
//	TIM3_PWM_Init(1000-1,72-1);   //PWMÊä³ö  1KHZÖÜÆÚ
//	TIM_SetCompare1(TIM3,200);		//ÉèÖÃÕ¼¿Õ±È
//	TIM_SetCompare2(TIM3,400);		//ÉèÖÃÕ¼¿Õ±È
//	TIM_SetCompare3(TIM3,600);		//ÉèÖÃÕ¼¿Õ±È
//	TIM_SetCompare4(TIM3,800);		//ÉèÖÃÕ¼¿Õ±È
//	TIM4_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
//	TIM1_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
//	TIM2_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
//	TIM5_PWMINPUT_INIT(0xffff,72-1);   //pwmÊäÈë³õÊ¼»¯ÒÔ1MµÄÆµÂÊ²¶×½
//
////	PWM_Init(900,0);	 //²»·ÖÆµ¡£PWMÆµÂÊ=72000/900=8Khz
//		while(1)
//	{	 
//			delay_ms(100);
// 		  if(!CollectFlag_TIM4)
//			{
//				printf("duty_TIM4  	= %d%% \r\n",duty_TIM4*100/period_TIM4);				//´òÓ¡Õ¼¿Õ±È
//				printf("cycle_TIM4 	= %dKHz\r\n",1000/period_TIM4);//´òÓ¡ÖÜÆÚÁíÒ»ÖÖ½Ð·¨
//				printf("period_TIM4 = %dus\r\n",period_TIM4);    //´òÓ¡ÖÜÆÚ
//				CollectFlag_TIM4 = 1;
//			}
//			delay_ms(100);
// 		  if(!CollectFlag_TIM1)
//			{
//				printf("duty_TIM1  	= %d%% \r\n",duty_TIM1*100/period_TIM1);				//´òÓ¡Õ¼¿Õ±È
//				printf("cycle_TIM1 	= %dKHz\r\n",1000/period_TIM1);//´òÓ¡ÖÜÆÚÁíÒ»ÖÖ½Ð·¨
//				printf("period_TIM1 = %dus\r\n",period_TIM1);    //´òÓ¡ÖÜÆÚ
//				CollectFlag_TIM1 = 1;
//			}
//						delay_ms(100);
// 		  if(!CollectFlag_TIM2)
//			{
//				printf("duty_TIM2  	= %d%% \r\n",duty_TIM2*100/period_TIM2);				//´òÓ¡Õ¼¿Õ±È
//				printf("cycle_TIM2 	= %dKHz\r\n",1000/period_TIM2);//´òÓ¡ÖÜÆÚÁíÒ»ÖÖ½Ð·¨
//				printf("period_TIM2 = %dus\r\n",period_TIM2);    //´òÓ¡ÖÜÆÚ
//				CollectFlag_TIM2 = 1;
//			}
//		      delay_ms(100);
// 		  if(!CollectFlag_TIM5)
//			{
//				printf("duty_TIM5  	= %d%% \r\n",duty_TIM5*100/period_TIM5);				//´òÓ¡Õ¼¿Õ±È
//				printf("cycle_TIM5 	= %dKHz\r\n",1000/period_TIM5);//´òÓ¡ÖÜÆÚÁíÒ»ÖÖ½Ð·¨
//				printf("period_TIM5 = %dus\r\n",period_TIM5);    //´òÓ¡ÖÜÆÚ
//				CollectFlag_TIM5 = 1;
//			}
//		  
//
//	 }
//		 
 


