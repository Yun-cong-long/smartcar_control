#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "zf_common_headfile.h"
#include "PIDController.h"


#define MOTOR1_DIR               (C10 )                                    
#define MOTOR1_PWM               (PWM2_MODULE2_CHB_C11)

#define MOTOR2_DIR               (D2 )
#define MOTOR2_PWM               (PWM2_MODULE3_CHB_D3)                   //右后方电机

#define MOTOR3_DIR               (C7 )
#define MOTOR3_PWM               (PWM2_MODULE0_CHA_C6)                  //前方电机

#define MOTOR4_DIR               (C9 )
#define MOTOR4_PWM               (PWM2_MODULE1_CHA_C8)                 //左后方电机

extern PID_t Motor1_SpeedPID;
extern PID_t Motor2_SpeedPID;
extern PID_t Motor3_SpeedPID;
extern PID_t Motor4_SpeedPID;
extern PID_t angle_PID;
//extern PID_t angle1_PID;
extern PID_t angle2_PID;
extern PID_t angle3_PID;
extern PID_t angle4_PID;

extern int position_get;   //是否到达坐标的标志位,1为到达，-1未开始,0为进行中
extern int correction_flag;    //是否完成微调的标志位,1为到达，-1未开始,0为进行中

extern float angle_set;//车模和速度方向
extern int8 start;//开始标志，由按键置1

void Motor_Init(void);
void Motor_Clear(void);
void Motor_Test(void);

void MotorPID_DefaultInit(void);
void anglePID_DefaultInit(void);

void Motor_Speed_PID(void);
void SetMotorCurrent(uint8_t num, float current);

void Circumferential(float speed, float radius);
void tracking(float speed,float angle,float distance);

void arrive_corrd(float x, float y, float speed, float angle);
void correct(float x, float y, float speed, float angle, float range);
void speed_and_angle(float speed, float angle);
void speed(float speed);
void speed_and_turn(float derection, float speed);
void speed_and_angle_and_turn(float derection, float speed ,float angle);
void correct_position(float x, float y, float speed);
void xunjixunji(float speed, float angle1);
void angle_controll(int num,float target);
void left_or_right(float speed);
void close(float position);
void left_forward(float speed);
void right_forward(float speed);
int motor2_position(float position);
int motor3_position(float position);
int motor4_position(float position);
void pushbox_left(void);
void move_control(float move_speed, float turn_angle);
void set_motor_speed(float motor2_speed, float motor4_speed, float motor3_speed);
void Motor_Speed_PID(void);
void speed_and_angle(float speed, float angle);
#endif
