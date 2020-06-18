#include "lcd12864.h"
#include "delay.h"
//#include<intrins.h>
#define uchar unsigned char
#define uint unsigned int
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
//uchar  table[16]=				 //80
//"Mecanum  Monitor";							  //һ������ռ��2���ֽ�
//uchar   table1[16]=			  //90
//"Sys Condition:OK";
//uchar   table2[16]=			 //88
//"V1:       V2:   ";
//uchar   table3[16]=			   //98
//"V3:       V4:   ";
uchar  table[16]=				 //80
"�����ķ�����ƶ�";							  //һ������ռ��2���ֽ�
uchar   table1[16]=			  //90
"Vx:       mm/s  ";
uchar   table2[16]=			 //88
"Vy:       mm/s  ";
uchar   table3[16]=			   //98
"Wz:     E-2rad/s";

 void init_LCD_IO()
 {
	GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);  //ʹ��GPIO����

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4 | GPIO_Pin_5 ; //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

}
void sendbyte(unsigned char bbyte) //����һ���ֽ�
{
 unsigned char i;
 for(i=0;i<8;i++)
   {
  // LCD_SID=bbyte&0x80; //ȡ�����λ
  if(bbyte&0x80)
	 LCD_SID=1;
   else 
	 LCD_SID=0;

   //delay_ms(1); 	 //��Ӳ����Ӧ������
   LCD_SCLK=1;
   //delay_ms(1); 	 //��Ӳ����Ӧ������
   LCD_SCLK=0;
   bbyte<<=1; //����
   }  
}

void write(unsigned char start, unsigned char ddata) //дָ�������
{
  unsigned char start_data,Hdata,Ldata;
//  LCD_CS=1;
  if(start==0x00) 
    start_data=0xf8;  //дָ��
  else       
	start_data=0xfa;  //д����
  
  Hdata=ddata&0xf0;    //ȡ����λ
  Ldata=(ddata<<4)&0xf0;  //ȡ����λ
  sendbyte(start_data);   //������ʼ�ź�
  delay_ms(5); //��ʱ�Ǳ����
  sendbyte(Hdata);       //���͸���λ
  delay_ms(1);  //��ʱ�Ǳ����
  sendbyte(Ldata);    //���͵���λ
  delay_ms(1);  //��ʱ�Ǳ����
}

void lcd_init()
{
	uchar i;
	write(0,0x30);  //8 λ���棬����ָ�
	write(0,0x0c);  //��ʾ�򿪣����أ����׹�
	write(0,0x01);  //��������DDRAM�ĵ�ַ����������  
	delay_ms(10);

	write(0,0x80); 
	for(i=0;i<16;i++)  
	write(1,table[i]);

	write(0,0x90); 
	for(i=0;i<16;i++) 
	write(1,table1[i]);

	write(0,0x88); 
	for(i=0;i<16;i++)  
	write(1,table2[i]);

	write(0,0x98); 
	for(i=0;i<16;i++)  
	write(1,table3[i]);
}

void lcd_speed_update()
{
    uchar i;
	
	frequency1=1000000/period_TIM4	;
table2[7]=(period_TIM4%10)+0x30;
table2[6]=period_TIM4/10%10+0x30;
table2[5]=period_TIM4/100%10+0x30;
table2[4]=period_TIM4/1000%10+0x30;
//	speed
    
	write(0,0x88); 
	for(i=0;i<16;i++)  
	write(1,table2[i]);

	write(0,0x98); 
	for(i=0;i<16;i++)  
	write(1,table3[i]);

}

