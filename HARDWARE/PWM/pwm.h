#ifndef __PWM_H
#define __PWM_H
#include "sys.h"
#define motor1_out0 PFout(0)
#define motor1_out1 PFout(2)

#define motor2_out0 PFout(4)
#define motor2_out1 PFout(6)

#define motor3_out0 PFout(1)
#define motor3_out1 PFout(3)

#define motor4_out0 PFout(5)
#define motor4_out1 PFout(7)
//
//u16 period_TIM1 ;
//u16 duty_TIM1  ;
//u8 CollectFlag_TIM1 ;
//  
//u16 period_TIM2;
//u16 duty_TIM2  ;
//u8 CollectFlag_TIM2 ;
//
//u16 period_TIM4 ;
//u16 duty_TIM4  ;
//u8 CollectFlag_TIM4 ;
//
//u16 period_TIM5;
//u16 duty_TIM5  ;
//u8 CollectFlag_TIM5 ;
void PWM_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWMINPUT_INIT(u16 arr,u16 psc);
void TIM1_PWMINPUT_INIT(u16 arr,u16 psc);
void TIM2_PWMINPUT_INIT(u16 arr,u16 psc);
void TIM5_PWMINPUT_INIT(u16 arr,u16 psc);
void MOTOR_OUT(u8 direct1,u16 pwm1,u8 direct2,u16 pwm2,u8 direct3,u16 pwm3,u8 direct4,u16 pwm4);
void MOTOR_INIT(void);
void TIM_SetCompare(u16 pwm1,u16 pwm2,u16 pwm3,u16 pwm4);
#endif
