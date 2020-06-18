#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"
#include"usart.h"
#include "math.h"
typedef struct 
{
 int setpoint;//设定目标
 int sum_error;//误差累计
 float proportion ;//比例常数
 float integral ;//积分常数
 float derivative;//微分常数
 int last_error;//e[-1]
 int prev_error;//e[-2]
}PIDtypedef;

void PIDperiodinit(u16 arr,u16 psc);				//PID 采样定时器设定
void incPIDinit(void);								//初始化，默认清零
int incPIDcalc(PIDtypedef*PIDx,u16 nextpoint);	   //PID计算
void PID_setpoint(PIDtypedef*PIDx,u16 setvalue);  //设定 定值
void PID_set(float pp,float ii,float dd);		   //
void set_speed(float W1,float W2,float W3,float W4);
void stop(void);
#endif
