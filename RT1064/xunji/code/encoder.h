#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "zf_common_headfile.h"

extern int16 Motor1_Speed;
extern int16 Motor2_Speed;
extern int16 Motor3_Speed;
extern int16 Motor4_Speed;
extern int16 Motor1_Position;
extern int16 Motor2_Position;//右后方电机
extern int16 Motor3_Position;//前方电机
extern int16 Motor4_Position;//左后方电机

extern float SUM_X;
extern float SUM_Y;
extern float CORRECT_X;
extern float CORRECT_Y;
extern float SPEED;//由按键设置速度

void Encoder_Init(void);
void get_corrd(void);
void position_zero(void);
#endif