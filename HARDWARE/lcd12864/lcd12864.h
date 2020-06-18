#ifndef __LCD12864_H
#define __LCD12864_H
#include "sys.h"

/******************************************************************/
/*                    ����ӿ���Ϣ                                */
/******************************************************************/
 //#define    LCD_CS   PCout(8)							//�󿪷���ֱ���P2�ڵ�01235������ΪP0��
 #define    LCD_SID  PGout(4)	//p06	RW					//С������ֱ���P0�ڵ�76543������ΪP2��
 #define    LCD_SCLK PGout(5)   //p05	E					//STM32 �ֱ�ΪPF8~10

/******************************************************************/
/*                    ������                                */
/******************************************************************/
void init_LCD_IO(void);
void sendbyte(unsigned char bbyte) ;//����һ���ֽ�
void write(unsigned char start, unsigned char ddata) ;//дָ�������
void lcd_init(void);
void lcd_speed_update(void);//�ٶ���ʾ�ĸ���
#endif

