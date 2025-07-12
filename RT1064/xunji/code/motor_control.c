#include "motor_control.h"
#include "Encoder.h"
#include "math.h"
#include "imu963ra.h"
#include "gyro.h"

//PID_t Motor1_SpeedPID;		
PID_t Motor2_SpeedPID;		//右后方电机        //左前方
PID_t Motor3_SpeedPID;		//前方电机          //后方
PID_t Motor4_SpeedPID;		//左后方电机        //右前方

PID_t angle_PID;					//角度
//PID_t angle1_PID;					//角度
PID_t angle2_PID;					//角度
PID_t angle3_PID;					//角度
PID_t angle4_PID;					//角度


int position_get = -1;   //是否到达坐标的标志位,1为到达，-1未开始,0为进行中
int correction_flag = -1;   //是否完成微调的标志位,1为到达，-1未开始,0为进行中


float x_next=0,y_next=0;   //用于斜线行驶中，下个需要跑的坐标与当前坐标的差值
float angle_set=PI*0;		//设定合速度方向
int8 start = 0;

void Motor_Init(void){

	
	system_delay_ms(100);           //等待主板其他外设上电完成
    
//    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高（顺时针）
//    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高（顺时针）
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高（顺时针）
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高（顺时针）
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
}

void Motor_Clear(void){
//	SetMotorCurrent(1,0);
	SetMotorCurrent(2,0);
	SetMotorCurrent(3,0);
	SetMotorCurrent(4,0);	
//	encoder_clear_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER2);
//	Motor1_Speed = 0;
  Motor2_Speed = 0;//右后方电机
  Motor3_Speed = 0;//前方电机
  Motor4_Speed = 0;//左后方电机
//	PID_Clear(&Motor1_SpeedPID);
	PID_Clear(&Motor2_SpeedPID);
  PID_Clear(&Motor3_SpeedPID);
  PID_Clear(&Motor4_SpeedPID);
}

void Motor_Test(void){	
//	SetMotorCurrent(1,-2000);
	SetMotorCurrent(2,1000);   //800临界
	SetMotorCurrent(3,1000);
  SetMotorCurrent(4,1000);
}

void anglePID_DefaultInit(void){				//角度PID参数初始化
	PID_DefaultInit(&angle_PID);
	angle_PID.Kp1 = 0.49f;				//5 // 0.3
	angle_PID.Ki1 = 0.0f;			//  // 0.002
	angle_PID.Kd1 = 25.0f;  // 
//	angle_PID.RampTartgetStep = 2;
	angle_PID.PID_OutMax = 4000;	
//	angle_PID.RampCountTime=99;
	angle_PID.PID_ErrAllMax = 2300;
	angle_PID.PID_OutStep = 2000; //500
//	angle_PID.State_RampOrNormal = Normal_e;
	PID_SetTargetWithNormal(&angle_PID,0.0);
	
//	PID_DefaultInit(&angle1_PID);
//	angle1_PID.Kp1 = 0.3f;				//5
//	angle1_PID.Ki1 = 0.002f;			//
//	angle1_PID.Kd1 = 0.0f;  //
////	angle_PID.RampTartgetStep = 2;
//	angle1_PID.PID_OutMax = 4000;	
////	angle_PID.RampCountTime=99;
//	angle1_PID.PID_ErrAllMax = 2300;
//	angle1_PID.PID_OutStep = 2000; //500
////	angle_PID.State_RampOrNormal = Normal_e;
//	PID_SetTargetWithNormal(&angle1_PID,0.0);
	
	PID_DefaultInit(&angle2_PID);
	angle2_PID.Kp1 = 30.0f;				//5
	angle2_PID.Ki1 = 2.0f;			//
	angle2_PID.Kd1 = 2.8f;  //
//	angle_PID.RampTartgetStep = 2;
	angle2_PID.PID_OutMax = 8000;	
//	angle_PID.RampCountTime=99;
	angle2_PID.PID_ErrAllMax = 2300;
	angle2_PID.PID_OutStep = 2000; //500
//	angle_PID.State_RampOrNormal = Normal_e;
	PID_SetTargetWithNormal(&angle2_PID,0.0);
	
	PID_DefaultInit(&angle3_PID);
	angle3_PID.Kp1 = 30.0f;				//100
	angle3_PID.Ki1 = 2.0f;			//
	angle3_PID.Kd1 = 2.8f;  //
//	angle_PID.RampTartgetStep = 2;
	angle3_PID.PID_OutMax = 8000;	
//	angle_PID.RampCountTime=99;
	angle3_PID.PID_ErrAllMax = 2300;
	angle3_PID.PID_OutStep = 2000; //500
//	angle_PID.State_RampOrNormal = Normal_e;
	PID_SetTargetWithNormal(&angle3_PID,0.0);
	
	PID_DefaultInit(&angle4_PID);
	angle4_PID.Kp1 = 30.0f;				// 原来 40 4 0.8
	angle4_PID.Ki1 = 2.0f;			//
	angle4_PID.Kd1 = 2.8f;  //
//	angle_PID.RampTartgetStep = 2;
	angle4_PID.PID_OutMax = 8000;	
//	angle_PID.RampCountTime=99;
	angle4_PID.PID_ErrAllMax = 2300;
	angle4_PID.PID_OutStep = 2000; //500
//	angle_PID.State_RampOrNormal = Normal_e;
	PID_SetTargetWithNormal(&angle4_PID,0.0);
}

void MotorPID_DefaultInit(void){				//电机PID参数初始化
//	PID_DefaultInit(&Motor1_SpeedPID);
//	Motor1_SpeedPID.Kp1 = 150.0f;								
//	Motor1_SpeedPID.Ki1 = 10.0f; 
//	Motor1_SpeedPID.Kd1 = 2.0f;
//	Motor1_SpeedPID.RampTartgetStep = 1;
//	Motor1_SpeedPID.PID_OutMax = 8000;
//	Motor1_SpeedPID.RampCountTime=99;
//	Motor1_SpeedPID.PID_ErrAllMax =1300;
//	Motor1_SpeedPID.PID_OutStep = 1000; //500
//	Motor1_SpeedPID.State_RampOrNormal = Ramp_e;   //工作在斜坡方式
//	PID_SetTargetWithRamp(&Motor1_SpeedPID,0.0);	 //没用这个引脚
	
	PID_DefaultInit(&Motor2_SpeedPID);
	Motor2_SpeedPID.Kp1 = 20.0f;         
	Motor2_SpeedPID.Ki1 = 0.5f; 
	Motor2_SpeedPID.Kd1 = 1;
//	Motor2_SpeedPID.RampTartgetStep = 1;      //坡道目标值
	Motor2_SpeedPID.PID_OutMax = 8000;
//	Motor2_SpeedPID.RampCountTime=99;         //坡道时间
	Motor2_SpeedPID.PID_ErrAllMax = 5300;
	Motor2_SpeedPID.PID_OutStep = 5000; //500
//	Motor2_SpeedPID.State_RampOrNormal = Ramp_e;    //坡道状态
	PID_SetTargetWithNormal(&Motor2_SpeedPID,0.0);
	
	PID_DefaultInit(&Motor3_SpeedPID);
	Motor3_SpeedPID.Kp1 = 20.0f;
	Motor3_SpeedPID.Ki1 = 0.5f; 
	Motor3_SpeedPID.Kd1 = 1;
//	Motor3_SpeedPID.RampTartgetStep = 1;
	Motor3_SpeedPID.PID_OutMax = 8000;
//	Motor3_SpeedPID.RampCountTime=99;
	Motor3_SpeedPID.PID_ErrAllMax = 5300;
	Motor3_SpeedPID.PID_OutStep = 5000; //500
//	Motor3_SpeedPID.State_RampOrNormal = Ramp_e;
	PID_SetTargetWithNormal(&Motor3_SpeedPID,0.0);
	
	
	PID_DefaultInit(&Motor4_SpeedPID);
	Motor4_SpeedPID.Kp1 = 20.0f;			
	Motor4_SpeedPID.Ki1 = 0.5f; 	//或0.2
	Motor4_SpeedPID.Kd1 = 1;				
//	Motor4_SpeedPID.RampTartgetStep = 1;
	Motor4_SpeedPID.PID_OutMax = 8000;
//	Motor4_SpeedPID.RampCountTime=99;
	Motor4_SpeedPID.PID_ErrAllMax = 5300;
	Motor4_SpeedPID.PID_OutStep = 5000; //500
//	Motor4_SpeedPID.State_RampOrNormal = Ramp_e;
	PID_SetTargetWithNormal(&Motor4_SpeedPID,0.0);
}

void SetMotorCurrent(uint8_t num, float current){			//设置电机电流（PWM），控制电机转速
//	//输出限幅
	if(current > 9000)
			current = 9000;
	else if(current < -9000)
		current = -9000;
//	if (current > 0 && current < 1200)
//		current = 1200;
//	if (current > -1200 && current < 0)
//		current = -1200;
   switch(num)
	 {
//		 case 1:
//			 if (current>=0){
//				 gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                         // DIR输出高电平（顺时针）
//         pwm_set_duty(MOTOR1_PWM, current);                   // 计算占空比
//			 }				 
//			 else{
//				 gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                          // DIR输出低电平（逆时针）
//           pwm_set_duty(MOTOR1_PWM, -current);                // 计算占空比
//			 }				 
//		 break;
		 
		 case 2:
			 if (current>=0){
				  gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                         // DIR输出高电平（顺时针）
         pwm_set_duty(MOTOR2_PWM, current);                   // 计算占空比
			 }
			 else{
				 gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                          // DIR输出低电平（逆时针）
           pwm_set_duty(MOTOR2_PWM, -current);                // 计算占空比
			 }
		 break; 
	 
		 case 3:
			  if (current>=0){
				  gpio_set_level(MOTOR3_DIR, GPIO_LOW);                                         // DIR输出高电平（顺时针）
         pwm_set_duty(MOTOR3_PWM, current);                   // 计算占空比
			 }
			 else{
				 gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                          // DIR输出低电平（逆时针）
           pwm_set_duty(MOTOR3_PWM, -current);                // 计算占空比
			 }
		 break; 
			 
		 case 4:
			  if (current>=0){
				  gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                         // DIR输出高电平（顺时针）
         pwm_set_duty(MOTOR4_PWM, current);                   // 计算占空比
			 }
			 else{
				 gpio_set_level(MOTOR4_DIR, GPIO_LOW);                                          // DIR输出低电平（逆时针）
           pwm_set_duty(MOTOR4_PWM, -current);                // 计算占空比
			 }
		 break;  
	 } 
}


void xunjixunji(float speed, float angle1)
{
	get_corrd();              //读取当前电机编码器数值       
	PID_SetTargetWithNormal(&angle_PID, 6500*angle1/360);     //设置目标值
	PID_Update(&angle_PID, 6500*angle1/360);                  //更新PID参数
	PID_GetPositionPID(&angle_PID);                   //更新PIDOUT（位置式）
//	SetMotorCurrent(2,Motor2_Speed+angle_PID.PID_Out);       //驱动电机
//	SetMotorCurrent(3,Motor3_Speed+angle_PID.PID_Out+1000);       //驱动电机
//	SetMotorCurrent(4,Motor4_Speed+angle_PID.PID_Out);       //驱动电机
	if (angle1>0)          //车右转
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed-angle_PID.PID_Out);   //设置目标值
		PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed-angle_PID.PID_Out);   //设置目标值
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed-angle_PID.PID_Out);
	}
	else if (angle1<0)      //车左转
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed-angle_PID.PID_Out);   //设置目标值
		PID_SetTargetWithNormal(&Motor3_SpeedPID,speed-angle_PID.PID_Out);   //设置目标值
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed-angle_PID.PID_Out);
	}
	else                //车直走
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed);   //设置目标值
		PID_SetTargetWithNormal(&Motor3_SpeedPID,0);   //设置目标值
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed);
	}
	PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //更新PID参数
	PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //更新PID参数
	PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //更新PID参数
	PID_GetIncrementalPID(&Motor2_SpeedPID);         //由参数更新PID-OUT
	PID_GetIncrementalPID(&Motor3_SpeedPID);         //由参数更新PID-OUT
	PID_GetIncrementalPID(&Motor4_SpeedPID);         //由参数更新PID-OUT
	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //驱动电机
	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //驱动电机
	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //驱动电机
//	ips200_show_int(0, 200, Motor2_Speed,4);                 //6500
//						ips200_show_int(0, 216,  Motor2_SpeedPID.PID_Out,4);
//				ips200_show_int(0, 232,  Motor2_SpeedPID.PID_Target,4);
//	 printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // 输出编码器计数信息  
//	 printf("ENCODER_3 counter \t%d .\r\n", Motor3_Speed);                 // 输出编码器计数信息  
//	 printf("ENCODER_4 counter \t%d .\r\n", Motor4_Speed);
	// 输出编码器计数信息  
//	 printf("Motor2_SpeedPID.PID_Out \t%d .\r\n", Motor2_SpeedPID.PID_Out);                 // 输出编码器计数信息  
	
}

void left_or_right(float speed)             //正值往右，负值往左
{
//	  get_corrd();              //读取当前电机编码器数值
	  PID_SetTargetWithNormal(&Motor2_SpeedPID,speed);   //设置目标值
		PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed*2);   //设置目标值
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,speed);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //更新PID参数
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //更新PID参数
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //更新PID参数
	  PID_GetIncrementalPID(&Motor2_SpeedPID);         //由参数更新PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //由参数更新PID-OUT
  	PID_GetIncrementalPID(&Motor4_SpeedPID);         //由参数更新PID-OUT
  	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //驱动电机
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //驱动电机
  	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //驱动电机
}

void close(float position)
{
	position_zero();
	int flag=0;
	while (flag != 1)
	{
	get_corrd();              //读取当前电机编码器数值
	if (Motor2_Position > position - 50 && Motor2_Position < position + 50 && Motor4_Position > position - 50 && Motor4_Position < position + 50)
	{
		flag=1;
	}
	PID_SetTargetWithNormal(&angle2_PID, position);     //设置目标值
	PID_SetTargetWithNormal(&angle4_PID, -position);     //设置目标值
	PID_Update(&angle2_PID, Motor2_Position);                  //更新PID参数
	PID_Update(&angle4_PID, Motor4_Position);                  //更新PID参数
	PID_GetPositionPID(&angle2_PID);                   //更新PIDOUT（位置式）
	PID_GetPositionPID(&angle4_PID);                   //更新PIDOUT（位置式）
	SetMotorCurrent(2,angle2_PID.PID_Out);   //设置目标值
	SetMotorCurrent(4,angle4_PID.PID_Out);   //设置目标值
	}
}

void left_forward(float speed)
{
	  get_corrd();              //读取当前电机编码器数值
	  PID_SetTargetWithNormal(&Motor2_SpeedPID,0);   //设置目标值
		PID_SetTargetWithNormal(&Motor3_SpeedPID,speed);   //设置目标值
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //更新PID参数
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //更新PID参数
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //更新PID参数
	  PID_GetIncrementalPID(&Motor2_SpeedPID);         //由参数更新PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //由参数更新PID-OUT
  	PID_GetIncrementalPID(&Motor4_SpeedPID);         //由参数更新PID-OUT
	  SetMotorCurrent(2,0);   //驱动电机
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //驱动电机
  	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //驱动电机
}

void right_forward(float speed)
{
	  get_corrd();              //读取当前电机编码器数值
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed+20.0);   //设置目标值
	  PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed);
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,0);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //更新PID参数
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //更新PID参数
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //更新PID参数
  	PID_GetIncrementalPID(&Motor2_SpeedPID);         //由参数更新PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //由参数更新PID-OUT
	  PID_GetIncrementalPID(&Motor4_SpeedPID);         //由参数更新PID-OUT
  	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //驱动电机
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //驱动电机
	  SetMotorCurrent(4,0);   //驱动电机
}

int motor2_position(float position)
{
	get_corrd();              //读取当前电机编码器数值
	PID_SetTargetWithNormal(&angle2_PID, position);     //设置目标值
	PID_Update(&angle2_PID, Motor2_Position);                  //更新PID参数
	PID_GetPositionPID(&angle2_PID);                   //更新PIDOUT（位置式）
	SetMotorCurrent(2,angle2_PID.PID_Out);   //设置目标值
	if (angle2_PID.PID_Err_now < 50 && angle2_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

int motor3_position(float position)
{
	get_corrd();              //读取当前电机编码器数值
	PID_SetTargetWithNormal(&angle3_PID, position);     //设置目标值
	PID_Update(&angle3_PID, Motor3_Position);                  //更新PID参数
	PID_GetPositionPID(&angle3_PID);                   //更新PIDOUT（位置式）
	SetMotorCurrent(3,angle3_PID.PID_Out);   //设置目标值
	if (angle3_PID.PID_Err_now < 50 && angle3_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

int motor4_position(float position)
{
	get_corrd();              //读取当前电机编码器数值
	PID_SetTargetWithNormal(&angle4_PID, position);     //设置目标值
	PID_Update(&angle4_PID, Motor4_Position);                  //更新PID参数
	PID_GetPositionPID(&angle4_PID);                   //更新PIDOUT（位置式）
	SetMotorCurrent(4,angle4_PID.PID_Out);   //设置目标值
	if (angle4_PID.PID_Err_now < 50 && angle4_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

void move_control(float move_speed, float turn_angle)
{
	float z_speed;
	float motor2_speed, motor3_speed, motor4_speed;
	float z_p = 5.0;  // 1.8在80原  0.3在80现  3.0在100/120现  5.0在150现
	z_speed =  - turn_angle;
	if (z_speed > 180)
		z_speed = z_speed - 360;
	if (z_speed < -180)
		z_speed = z_speed + 360;
	motor2_speed = -move_speed + z_speed*z_p;
	motor4_speed = move_speed + z_speed*z_p;
	motor3_speed = z_speed*z_p;
	set_motor_speed(motor2_speed, motor3_speed, motor4_speed);
//	text111(motor2_speed, motor3_speed, motor4_speed);
}


void set_motor_speed(float motor2_speed, float motor3_speed, float motor4_speed)
{
	
  PID_SetTargetWithNormal(&Motor2_SpeedPID,motor2_speed);   //设置目标值
	PID_SetTargetWithNormal(&Motor3_SpeedPID,motor3_speed);
	PID_SetTargetWithNormal(&Motor4_SpeedPID,motor4_speed);
	PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //更新PID参数
  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //更新PID参数
  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //更新PID参数
 	PID_GetIncrementalPID(&Motor2_SpeedPID);         //由参数更新PID-OUT
	PID_GetIncrementalPID(&Motor3_SpeedPID);         //由参数更新PID-OUT
  PID_GetIncrementalPID(&Motor4_SpeedPID);         //由参数更新PID-OUT
 	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //驱动电机
 	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //驱动电机
  SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //驱动电机
//	ips200_show_int(0, 200, Motor2_SpeedPID.PID_Out,8);   
//	ips200_show_int(0, 216, Motor3_SpeedPID.PID_Out,8);  
//	ips200_show_int(0, 232, Motor4_SpeedPID.PID_Out,8); 
}

void Motor_Speed_PID(void)
{							//速度环PID控制
//	PID_Update(&Motor1_SpeedPID,Motor1_Speed);//更新pid
//	PID_GetPositionPID(&Motor1_SpeedPID);//更改PIDout
//	SetMotorCurrent(1,Motor1_SpeedPID.PID_Out);
	
	PID_Update(&angle2_PID,Motor2_Speed);
	PID_GetPositionPID(&angle2_PID);
	SetMotorCurrent(2,angle2_PID.PID_Out*2);
		
	PID_Update(&angle3_PID,Motor3_Speed);
	PID_GetPositionPID(&angle3_PID);
	SetMotorCurrent(3,angle3_PID.PID_Out);
	
	PID_Update(&angle4_PID,Motor4_Speed);
	PID_GetPositionPID(&angle4_PID);
	SetMotorCurrent(4,angle4_PID.PID_Out*2);
}

void speed_and_angle(float speed, float angle)	
{
//	PID_SetTargetWithNormal(&angle_PID, angle);
//	PID_Update(&angle_PID, Angle_z);
//	PID_GetPositionPID(&angle_PID);
//	
//	PID_SetTargetWithNormal(&Motor2_SpeedPID,speed - angle_PID.PID_Out);
//	PID_SetTargetWithNormal(&Motor3_SpeedPID,speed - angle_PID.PID_Out);
//	PID_SetTargetWithNormal(&Motor4_SpeedPID,speed + angle_PID.PID_Out);	
////	PID_SetTargetWithNormal(&Motor1_SpeedPID,speed + angle_PID.PID_Out);
////	if (mode == 0)   //左右
////	{
////	PID_SetTargetWithNormal(&angle2_PID,speed);  //60
////	PID_SetTargetWithNormal(&angle3_PID,-2*speed);  //-300
////	PID_SetTargetWithNormal(&angle4_PID,speed);  //60
////	}
////	if (mode == 1)  //前后
////	{
////		PID_SetTargetWithNormal(&angle2_PID,speed);  //60
////	  PID_SetTargetWithNormal(&angle3_PID,0);  //-300
////	  PID_SetTargetWithNormal(&angle4_PID,-speed);  //60
////	}
//	Motor_Speed_PID();
    float motor2_speed, motor3_speed, motor4_speed;
		motor2_speed = -speed + ANGLE_TO_RAD(angle - Angle_z)*5 + imu660ra_gyro_z/20;
		motor3_speed = 2*speed + ANGLE_TO_RAD(angle - Angle_z)*5 + imu660ra_gyro_z/20;
		motor4_speed = -speed + ANGLE_TO_RAD(angle - Angle_z)*5 + imu660ra_gyro_z/20;
//		motor2_speed = -speed;
//		motor3_speed = 2*speed;
//		motor4_speed = -speed;
//	  PID_SetTargetWithNormal(&angle2_PID,motor2_speed);
//	  PID_SetTargetWithNormal(&angle3_PID,motor3_speed);
//	  PID_SetTargetWithNormal(&angle4_PID,motor4_speed);
//		Motor_Speed_PID();
		text111(motor2_speed, motor3_speed, motor4_speed);
}

void text111(float motor2_speed, float motor3_speed, float motor4_speed)
{
//	int turn;
//	turn = 90 - angle;
	PID_SetTargetWithNormal(&angle2_PID,motor2_speed);   //设置目标值
	PID_SetTargetWithNormal(&angle3_PID,motor3_speed);
	PID_SetTargetWithNormal(&angle4_PID,motor4_speed);
	PID_Update(&angle2_PID,Motor2_Speed);    //更新PID参数
  PID_Update(&angle3_PID,Motor3_Speed);    //更新PID参数
  PID_Update(&angle4_PID,Motor4_Speed);    //更新PID参数
 	PID_GetIncrementalPID(&angle2_PID);         //由参数更新PID-OUT
	PID_GetIncrementalPID(&angle3_PID);         //由参数更新PID-OUT
  PID_GetIncrementalPID(&angle4_PID);         //由参数更新PID-OUT
 	SetMotorCurrent(2,angle2_PID.PID_Out);   //驱动电机
 	SetMotorCurrent(3,angle3_PID.PID_Out);   //驱动电机
  SetMotorCurrent(4,angle4_PID.PID_Out);   //驱动电机
//	ips200_show_int(0, 200, angle2_PID.PID_Out,8);   
//	ips200_show_int(0, 216, angle3_PID.PID_Out,8);  
//	ips200_show_int(0, 232, angle4_PID.PID_Out,8); 
}