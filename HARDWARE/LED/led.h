#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"
#define LED1 PEout(15)// PE15
/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
//#define ON  0
//#define OFF 1
//
//#define LED(a)	if (a)	\
//					GPIO_SetBits(GPIOC,GPIO_Pin_13);\
//					else		\
//					GPIO_ResetBits(GPIOC,GPIO_Pin_13)
//
//

void LED_GPIO_Config(void);

#endif /* __LED_H */
