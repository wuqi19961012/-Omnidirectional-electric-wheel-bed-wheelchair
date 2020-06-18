  #include"kinematics.h"
  #define rhub    30.0  //轮子半径
#define L		125.0//公式中的  L，单位：mm
#define l        125.0//··········l，单位：mm
 int X,Y;//两个速度		mm/s
 float W;//角速度		 rad/s
 float W1,W2,W3,W4;//四个轮子的转速  rad/s
 void kinematics(int xx,int yy,float ww,float *ww1,float *ww2,float *ww3,float *ww4)
 {
 
  *ww1=(xx-yy-(L+l)*ww)/rhub;
  *ww2=(xx+yy+(L+l)*ww)/rhub;
  *ww3=(xx+yy-(L+l)*ww)/rhub;
  *ww4=(xx-yy+(L+l)*ww)/rhub;
 
 }

/*
 设定车轮的极限转速n= 1r/s，即w=2pi rad/s，此时Vx=188.5
 所以不妨设定Vx，Vy最大为200 ，反推得轮子w最大值为w=6.67 rad/s=2.122 pi/s
 按照这个极限轮子转速，车体的中心转速即上式的参数：ww最大有ww=1.11 rad/s
 注意：上面过程和 L l都是有关系的，应该按照具体的小车参数再次设定
*/

