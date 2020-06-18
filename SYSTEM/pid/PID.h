#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"
#include"usart.h"
#include "math.h"
typedef struct 
{
 int setpoint;//�趨Ŀ��
 int sum_error;//����ۼ�
 float proportion ;//��������
 float integral ;//���ֳ���
 float derivative;//΢�ֳ���
 int last_error;//e[-1]
 int prev_error;//e[-2]
}PIDtypedef;

void PIDperiodinit(u16 arr,u16 psc);				//PID ������ʱ���趨
void incPIDinit(void);								//��ʼ����Ĭ������
int incPIDcalc(PIDtypedef*PIDx,u16 nextpoint);	   //PID����
void PID_setpoint(PIDtypedef*PIDx,u16 setvalue);  //�趨 ��ֵ
void PID_set(float pp,float ii,float dd);		   //
void set_speed(float W1,float W2,float W3,float W4);
void stop(void);
#endif
