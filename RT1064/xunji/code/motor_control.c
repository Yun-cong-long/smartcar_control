#include "motor_control.h"
#include "Encoder.h"
#include "math.h"
#include "imu963ra.h"
#include "gyro.h"

//PID_t Motor1_SpeedPID;		
PID_t Motor2_SpeedPID;		//�Һ󷽵��        //��ǰ��
PID_t Motor3_SpeedPID;		//ǰ�����          //��
PID_t Motor4_SpeedPID;		//��󷽵��        //��ǰ��

PID_t angle_PID;					//�Ƕ�
//PID_t angle1_PID;					//�Ƕ�
PID_t angle2_PID;					//�Ƕ�
PID_t angle3_PID;					//�Ƕ�
PID_t angle4_PID;					//�Ƕ�


int position_get = -1;   //�Ƿ񵽴�����ı�־λ,1Ϊ���-1δ��ʼ,0Ϊ������
int correction_flag = -1;   //�Ƿ����΢���ı�־λ,1Ϊ���-1δ��ʼ,0Ϊ������


float x_next=0,y_next=0;   //����б����ʻ�У��¸���Ҫ�ܵ������뵱ǰ����Ĳ�ֵ
float angle_set=PI*0;		//�趨���ٶȷ���
int8 start = 0;

void Motor_Init(void){

	
	system_delay_ms(100);           //�ȴ��������������ϵ����
    
//    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ����������ߣ�˳ʱ�룩
//    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ����������ߣ�˳ʱ�룩
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ����������ߣ�˳ʱ�룩
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ����������ߣ�˳ʱ�룩
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
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
  Motor2_Speed = 0;//�Һ󷽵��
  Motor3_Speed = 0;//ǰ�����
  Motor4_Speed = 0;//��󷽵��
//	PID_Clear(&Motor1_SpeedPID);
	PID_Clear(&Motor2_SpeedPID);
  PID_Clear(&Motor3_SpeedPID);
  PID_Clear(&Motor4_SpeedPID);
}

void Motor_Test(void){	
//	SetMotorCurrent(1,-2000);
	SetMotorCurrent(2,1000);   //800�ٽ�
	SetMotorCurrent(3,1000);
  SetMotorCurrent(4,1000);
}

void anglePID_DefaultInit(void){				//�Ƕ�PID������ʼ��
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
	angle4_PID.Kp1 = 30.0f;				// ԭ�� 40 4 0.8
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

void MotorPID_DefaultInit(void){				//���PID������ʼ��
//	PID_DefaultInit(&Motor1_SpeedPID);
//	Motor1_SpeedPID.Kp1 = 150.0f;								
//	Motor1_SpeedPID.Ki1 = 10.0f; 
//	Motor1_SpeedPID.Kd1 = 2.0f;
//	Motor1_SpeedPID.RampTartgetStep = 1;
//	Motor1_SpeedPID.PID_OutMax = 8000;
//	Motor1_SpeedPID.RampCountTime=99;
//	Motor1_SpeedPID.PID_ErrAllMax =1300;
//	Motor1_SpeedPID.PID_OutStep = 1000; //500
//	Motor1_SpeedPID.State_RampOrNormal = Ramp_e;   //������б�·�ʽ
//	PID_SetTargetWithRamp(&Motor1_SpeedPID,0.0);	 //û���������
	
	PID_DefaultInit(&Motor2_SpeedPID);
	Motor2_SpeedPID.Kp1 = 20.0f;         
	Motor2_SpeedPID.Ki1 = 0.5f; 
	Motor2_SpeedPID.Kd1 = 1;
//	Motor2_SpeedPID.RampTartgetStep = 1;      //�µ�Ŀ��ֵ
	Motor2_SpeedPID.PID_OutMax = 8000;
//	Motor2_SpeedPID.RampCountTime=99;         //�µ�ʱ��
	Motor2_SpeedPID.PID_ErrAllMax = 5300;
	Motor2_SpeedPID.PID_OutStep = 5000; //500
//	Motor2_SpeedPID.State_RampOrNormal = Ramp_e;    //�µ�״̬
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
	Motor4_SpeedPID.Ki1 = 0.5f; 	//��0.2
	Motor4_SpeedPID.Kd1 = 1;				
//	Motor4_SpeedPID.RampTartgetStep = 1;
	Motor4_SpeedPID.PID_OutMax = 8000;
//	Motor4_SpeedPID.RampCountTime=99;
	Motor4_SpeedPID.PID_ErrAllMax = 5300;
	Motor4_SpeedPID.PID_OutStep = 5000; //500
//	Motor4_SpeedPID.State_RampOrNormal = Ramp_e;
	PID_SetTargetWithNormal(&Motor4_SpeedPID,0.0);
}

void SetMotorCurrent(uint8_t num, float current){			//���õ��������PWM�������Ƶ��ת��
//	//����޷�
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
//				 gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ��˳ʱ�룩
//         pwm_set_duty(MOTOR1_PWM, current);                   // ����ռ�ձ�
//			 }				 
//			 else{
//				 gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                          // DIR����͵�ƽ����ʱ�룩
//           pwm_set_duty(MOTOR1_PWM, -current);                // ����ռ�ձ�
//			 }				 
//		 break;
		 
		 case 2:
			 if (current>=0){
				  gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                         // DIR����ߵ�ƽ��˳ʱ�룩
         pwm_set_duty(MOTOR2_PWM, current);                   // ����ռ�ձ�
			 }
			 else{
				 gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                          // DIR����͵�ƽ����ʱ�룩
           pwm_set_duty(MOTOR2_PWM, -current);                // ����ռ�ձ�
			 }
		 break; 
	 
		 case 3:
			  if (current>=0){
				  gpio_set_level(MOTOR3_DIR, GPIO_LOW);                                         // DIR����ߵ�ƽ��˳ʱ�룩
         pwm_set_duty(MOTOR3_PWM, current);                   // ����ռ�ձ�
			 }
			 else{
				 gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                          // DIR����͵�ƽ����ʱ�룩
           pwm_set_duty(MOTOR3_PWM, -current);                // ����ռ�ձ�
			 }
		 break; 
			 
		 case 4:
			  if (current>=0){
				  gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ��˳ʱ�룩
         pwm_set_duty(MOTOR4_PWM, current);                   // ����ռ�ձ�
			 }
			 else{
				 gpio_set_level(MOTOR4_DIR, GPIO_LOW);                                          // DIR����͵�ƽ����ʱ�룩
           pwm_set_duty(MOTOR4_PWM, -current);                // ����ռ�ձ�
			 }
		 break;  
	 } 
}


void xunjixunji(float speed, float angle1)
{
	get_corrd();              //��ȡ��ǰ�����������ֵ       
	PID_SetTargetWithNormal(&angle_PID, 6500*angle1/360);     //����Ŀ��ֵ
	PID_Update(&angle_PID, 6500*angle1/360);                  //����PID����
	PID_GetPositionPID(&angle_PID);                   //����PIDOUT��λ��ʽ��
//	SetMotorCurrent(2,Motor2_Speed+angle_PID.PID_Out);       //�������
//	SetMotorCurrent(3,Motor3_Speed+angle_PID.PID_Out+1000);       //�������
//	SetMotorCurrent(4,Motor4_Speed+angle_PID.PID_Out);       //�������
	if (angle1>0)          //����ת
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed-angle_PID.PID_Out);   //����Ŀ��ֵ
		PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed-angle_PID.PID_Out);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed-angle_PID.PID_Out);
	}
	else if (angle1<0)      //����ת
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed-angle_PID.PID_Out);   //����Ŀ��ֵ
		PID_SetTargetWithNormal(&Motor3_SpeedPID,speed-angle_PID.PID_Out);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed-angle_PID.PID_Out);
	}
	else                //��ֱ��
	{
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed);   //����Ŀ��ֵ
		PID_SetTargetWithNormal(&Motor3_SpeedPID,0);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed);
	}
	PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //����PID����
	PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //����PID����
	PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //����PID����
	PID_GetIncrementalPID(&Motor2_SpeedPID);         //�ɲ�������PID-OUT
	PID_GetIncrementalPID(&Motor3_SpeedPID);         //�ɲ�������PID-OUT
	PID_GetIncrementalPID(&Motor4_SpeedPID);         //�ɲ�������PID-OUT
	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //�������
	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //�������
	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //�������
//	ips200_show_int(0, 200, Motor2_Speed,4);                 //6500
//						ips200_show_int(0, 216,  Motor2_SpeedPID.PID_Out,4);
//				ips200_show_int(0, 232,  Motor2_SpeedPID.PID_Target,4);
//	 printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // ���������������Ϣ  
//	 printf("ENCODER_3 counter \t%d .\r\n", Motor3_Speed);                 // ���������������Ϣ  
//	 printf("ENCODER_4 counter \t%d .\r\n", Motor4_Speed);
	// ���������������Ϣ  
//	 printf("Motor2_SpeedPID.PID_Out \t%d .\r\n", Motor2_SpeedPID.PID_Out);                 // ���������������Ϣ  
	
}

void left_or_right(float speed)             //��ֵ���ң���ֵ����
{
//	  get_corrd();              //��ȡ��ǰ�����������ֵ
	  PID_SetTargetWithNormal(&Motor2_SpeedPID,speed);   //����Ŀ��ֵ
		PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed*2);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,speed);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //����PID����
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //����PID����
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //����PID����
	  PID_GetIncrementalPID(&Motor2_SpeedPID);         //�ɲ�������PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //�ɲ�������PID-OUT
  	PID_GetIncrementalPID(&Motor4_SpeedPID);         //�ɲ�������PID-OUT
  	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //�������
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //�������
  	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //�������
}

void close(float position)
{
	position_zero();
	int flag=0;
	while (flag != 1)
	{
	get_corrd();              //��ȡ��ǰ�����������ֵ
	if (Motor2_Position > position - 50 && Motor2_Position < position + 50 && Motor4_Position > position - 50 && Motor4_Position < position + 50)
	{
		flag=1;
	}
	PID_SetTargetWithNormal(&angle2_PID, position);     //����Ŀ��ֵ
	PID_SetTargetWithNormal(&angle4_PID, -position);     //����Ŀ��ֵ
	PID_Update(&angle2_PID, Motor2_Position);                  //����PID����
	PID_Update(&angle4_PID, Motor4_Position);                  //����PID����
	PID_GetPositionPID(&angle2_PID);                   //����PIDOUT��λ��ʽ��
	PID_GetPositionPID(&angle4_PID);                   //����PIDOUT��λ��ʽ��
	SetMotorCurrent(2,angle2_PID.PID_Out);   //����Ŀ��ֵ
	SetMotorCurrent(4,angle4_PID.PID_Out);   //����Ŀ��ֵ
	}
}

void left_forward(float speed)
{
	  get_corrd();              //��ȡ��ǰ�����������ֵ
	  PID_SetTargetWithNormal(&Motor2_SpeedPID,0);   //����Ŀ��ֵ
		PID_SetTargetWithNormal(&Motor3_SpeedPID,speed);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,-speed);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //����PID����
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //����PID����
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //����PID����
	  PID_GetIncrementalPID(&Motor2_SpeedPID);         //�ɲ�������PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //�ɲ�������PID-OUT
  	PID_GetIncrementalPID(&Motor4_SpeedPID);         //�ɲ�������PID-OUT
	  SetMotorCurrent(2,0);   //�������
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //�������
  	SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //�������
}

void right_forward(float speed)
{
	  get_corrd();              //��ȡ��ǰ�����������ֵ
		PID_SetTargetWithNormal(&Motor2_SpeedPID,speed+20.0);   //����Ŀ��ֵ
	  PID_SetTargetWithNormal(&Motor3_SpeedPID,-speed);
	  PID_SetTargetWithNormal(&Motor4_SpeedPID,0);
	  PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //����PID����
	  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //����PID����
	  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //����PID����
  	PID_GetIncrementalPID(&Motor2_SpeedPID);         //�ɲ�������PID-OUT
  	PID_GetIncrementalPID(&Motor3_SpeedPID);         //�ɲ�������PID-OUT
	  PID_GetIncrementalPID(&Motor4_SpeedPID);         //�ɲ�������PID-OUT
  	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //�������
  	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //�������
	  SetMotorCurrent(4,0);   //�������
}

int motor2_position(float position)
{
	get_corrd();              //��ȡ��ǰ�����������ֵ
	PID_SetTargetWithNormal(&angle2_PID, position);     //����Ŀ��ֵ
	PID_Update(&angle2_PID, Motor2_Position);                  //����PID����
	PID_GetPositionPID(&angle2_PID);                   //����PIDOUT��λ��ʽ��
	SetMotorCurrent(2,angle2_PID.PID_Out);   //����Ŀ��ֵ
	if (angle2_PID.PID_Err_now < 50 && angle2_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

int motor3_position(float position)
{
	get_corrd();              //��ȡ��ǰ�����������ֵ
	PID_SetTargetWithNormal(&angle3_PID, position);     //����Ŀ��ֵ
	PID_Update(&angle3_PID, Motor3_Position);                  //����PID����
	PID_GetPositionPID(&angle3_PID);                   //����PIDOUT��λ��ʽ��
	SetMotorCurrent(3,angle3_PID.PID_Out);   //����Ŀ��ֵ
	if (angle3_PID.PID_Err_now < 50 && angle3_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

int motor4_position(float position)
{
	get_corrd();              //��ȡ��ǰ�����������ֵ
	PID_SetTargetWithNormal(&angle4_PID, position);     //����Ŀ��ֵ
	PID_Update(&angle4_PID, Motor4_Position);                  //����PID����
	PID_GetPositionPID(&angle4_PID);                   //����PIDOUT��λ��ʽ��
	SetMotorCurrent(4,angle4_PID.PID_Out);   //����Ŀ��ֵ
	if (angle4_PID.PID_Err_now < 50 && angle4_PID.PID_Err_now > -50)
		return 1;
	else 
		return 0;
}

void move_control(float move_speed, float turn_angle)
{
	float z_speed;
	float motor2_speed, motor3_speed, motor4_speed;
	float z_p = 5.0;  // 1.8��80ԭ  0.3��80��  3.0��100/120��  5.0��150��
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
	
  PID_SetTargetWithNormal(&Motor2_SpeedPID,motor2_speed);   //����Ŀ��ֵ
	PID_SetTargetWithNormal(&Motor3_SpeedPID,motor3_speed);
	PID_SetTargetWithNormal(&Motor4_SpeedPID,motor4_speed);
	PID_Update(&Motor2_SpeedPID,Motor2_Speed);    //����PID����
  PID_Update(&Motor3_SpeedPID,Motor3_Speed);    //����PID����
  PID_Update(&Motor4_SpeedPID,Motor4_Speed);    //����PID����
 	PID_GetIncrementalPID(&Motor2_SpeedPID);         //�ɲ�������PID-OUT
	PID_GetIncrementalPID(&Motor3_SpeedPID);         //�ɲ�������PID-OUT
  PID_GetIncrementalPID(&Motor4_SpeedPID);         //�ɲ�������PID-OUT
 	SetMotorCurrent(2,Motor2_SpeedPID.PID_Out);   //�������
 	SetMotorCurrent(3,Motor3_SpeedPID.PID_Out);   //�������
  SetMotorCurrent(4,Motor4_SpeedPID.PID_Out);   //�������
//	ips200_show_int(0, 200, Motor2_SpeedPID.PID_Out,8);   
//	ips200_show_int(0, 216, Motor3_SpeedPID.PID_Out,8);  
//	ips200_show_int(0, 232, Motor4_SpeedPID.PID_Out,8); 
}

void Motor_Speed_PID(void)
{							//�ٶȻ�PID����
//	PID_Update(&Motor1_SpeedPID,Motor1_Speed);//����pid
//	PID_GetPositionPID(&Motor1_SpeedPID);//����PIDout
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
////	if (mode == 0)   //����
////	{
////	PID_SetTargetWithNormal(&angle2_PID,speed);  //60
////	PID_SetTargetWithNormal(&angle3_PID,-2*speed);  //-300
////	PID_SetTargetWithNormal(&angle4_PID,speed);  //60
////	}
////	if (mode == 1)  //ǰ��
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
	PID_SetTargetWithNormal(&angle2_PID,motor2_speed);   //����Ŀ��ֵ
	PID_SetTargetWithNormal(&angle3_PID,motor3_speed);
	PID_SetTargetWithNormal(&angle4_PID,motor4_speed);
	PID_Update(&angle2_PID,Motor2_Speed);    //����PID����
  PID_Update(&angle3_PID,Motor3_Speed);    //����PID����
  PID_Update(&angle4_PID,Motor4_Speed);    //����PID����
 	PID_GetIncrementalPID(&angle2_PID);         //�ɲ�������PID-OUT
	PID_GetIncrementalPID(&angle3_PID);         //�ɲ�������PID-OUT
  PID_GetIncrementalPID(&angle4_PID);         //�ɲ�������PID-OUT
 	SetMotorCurrent(2,angle2_PID.PID_Out);   //�������
 	SetMotorCurrent(3,angle3_PID.PID_Out);   //�������
  SetMotorCurrent(4,angle4_PID.PID_Out);   //�������
//	ips200_show_int(0, 200, angle2_PID.PID_Out,8);   
//	ips200_show_int(0, 216, angle3_PID.PID_Out,8);  
//	ips200_show_int(0, 232, angle4_PID.PID_Out,8); 
}