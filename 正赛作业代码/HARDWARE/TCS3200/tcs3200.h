#ifndef __TCS3200_H
#define __TCS3200_H
#include "sys.h"

void TCS_GPIO_Init(void);
void filter1(int s2,int s3) ;
void filter2(int s21,int s31);
void TSC_WB_1(int s2, int s3 );
void TSC_WB_2(int s21, int s31 );
int color_choice(int R,int G,int B);
//S0-----PA4,S1-----PA5,S2-----PA6,S3-----PA7,LED-----PC5,OUT-----PA0
#define S0 PAout(4)
#define S1 PAout(5)
#define S2 PAout(6)
#define S3 PAout(7)
#define OUT PAin(0)

#define S01 PCout(0)
#define S11 PCout(1)
#define S21 PCout(2)
#define S31 PCout(3)
#define OUT1 PDin(2)

#define YELLOW 1
#define RED    2
#define NONE   3
#endif


