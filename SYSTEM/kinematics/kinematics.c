  #include"kinematics.h"
  #define rhub    30.0  //���Ӱ뾶
#define L		125.0//��ʽ�е�  L����λ��mm
#define l        125.0//��������������������l����λ��mm
 int X,Y;//�����ٶ�		mm/s
 float W;//���ٶ�		 rad/s
 float W1,W2,W3,W4;//�ĸ����ӵ�ת��  rad/s
 void kinematics(int xx,int yy,float ww,float *ww1,float *ww2,float *ww3,float *ww4)
 {
 
  *ww1=(xx-yy-(L+l)*ww)/rhub;
  *ww2=(xx+yy+(L+l)*ww)/rhub;
  *ww3=(xx+yy-(L+l)*ww)/rhub;
  *ww4=(xx-yy+(L+l)*ww)/rhub;
 
 }

/*
 �趨���ֵļ���ת��n= 1r/s����w=2pi rad/s����ʱVx=188.5
 ���Բ����趨Vx��Vy���Ϊ200 �����Ƶ�����w���ֵΪw=6.67 rad/s=2.122 pi/s
 ���������������ת�٣����������ת�ټ���ʽ�Ĳ�����ww�����ww=1.11 rad/s
 ע�⣺������̺� L l�����й�ϵ�ģ�Ӧ�ð��վ����С�������ٴ��趨
*/

