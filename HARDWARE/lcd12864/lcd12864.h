#ifndef __LCD12864_H
#define __LCD12864_H
#include "sys.h"

/******************************************************************/
/*                    定义接口信息                                */
/******************************************************************/
 //#define    LCD_CS   PCout(8)							//大开发板分别是P2口的01235，数据为P0口
 #define    LCD_SID  PGout(4)	//p06	RW					//小开发板分别是P0口的76543，数据为P2口
 #define    LCD_SCLK PGout(5)   //p05	E					//STM32 分别为PF8~10

/******************************************************************/
/*                    函数区                                */
/******************************************************************/
void init_LCD_IO(void);
void sendbyte(unsigned char bbyte) ;//发送一个字节
void write(unsigned char start, unsigned char ddata) ;//写指令或数据
void lcd_init(void);
void lcd_speed_update(void);//速度显示的更新
#endif

