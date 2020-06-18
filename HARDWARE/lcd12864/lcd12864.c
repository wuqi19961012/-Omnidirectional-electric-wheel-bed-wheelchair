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
//"Mecanum  Monitor";							  //一个汉字占用2个字节
//uchar   table1[16]=			  //90
//"Sys Condition:OK";
//uchar   table2[16]=			 //88
//"V1:       V2:   ";
//uchar   table3[16]=			   //98
//"V3:       V4:   ";
uchar  table[16]=				 //80
"麦克纳姆万向移动";							  //一个汉字占用2个字节
uchar   table1[16]=			  //90
"Vx:       mm/s  ";
uchar   table2[16]=			 //88
"Vy:       mm/s  ";
uchar   table3[16]=			   //98
"Wz:     E-2rad/s";

 void init_LCD_IO()
 {
	GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);  //使能GPIO外设

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4 | GPIO_Pin_5 ; //初始化GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

}
void sendbyte(unsigned char bbyte) //发送一个字节
{
 unsigned char i;
 for(i=0;i<8;i++)
   {
  // LCD_SID=bbyte&0x80; //取出最高位
  if(bbyte&0x80)
	 LCD_SID=1;
   else 
	 LCD_SID=0;

   //delay_ms(1); 	 //怕硬件反应不过来
   LCD_SCLK=1;
   //delay_ms(1); 	 //怕硬件反应不过来
   LCD_SCLK=0;
   bbyte<<=1; //左移
   }  
}

void write(unsigned char start, unsigned char ddata) //写指令或数据
{
  unsigned char start_data,Hdata,Ldata;
//  LCD_CS=1;
  if(start==0x00) 
    start_data=0xf8;  //写指令
  else       
	start_data=0xfa;  //写数据
  
  Hdata=ddata&0xf0;    //取高四位
  Ldata=(ddata<<4)&0xf0;  //取低四位
  sendbyte(start_data);   //发送起始信号
  delay_ms(5); //延时是必须的
  sendbyte(Hdata);       //发送高四位
  delay_ms(1);  //延时是必须的
  sendbyte(Ldata);    //发送低四位
  delay_ms(1);  //延时是必须的
}

void lcd_init()
{
	uchar i;
	write(0,0x30);  //8 位介面，基本指令集
	write(0,0x0c);  //显示打开，光标关，反白关
	write(0,0x01);  //清屏，将DDRAM的地址计数器归零  
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

