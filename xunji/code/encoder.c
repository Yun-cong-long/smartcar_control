#include "encoder.h"
#include "motor_control.h"
#include "math.h"
#include "imu963ra.h"
#include "gyro.h"
//#include "zf_driver_encoder.h"
//#include "Path.h"
//#include "Carry.h"




#define vx_slow 110.76
#define vy_slow 121.91
#define vx_5 125.9
#define vy_5 121.5
#define vx_10 127.4
#define vy_10 117.5
#define vx_15 128.6
#define vy_15 104.1966

int16 Motor1_Speed = 0;
int16 Motor2_Speed = 0;//�Һ󷽵��
int16 Motor3_Speed = 0;//ǰ�����
int16 Motor4_Speed = 0;//��󷽵��
int16 Motor1_Position = 0;
int16 Motor2_Position = 0;//�Һ󷽵��
int16 Motor3_Position = 0;//ǰ�����
int16 Motor4_Position = 0;//��󷽵��

float SUM_X = 0.0;
float SUM_Y = 0.0;
float CORRECT_X = 0.0;
float CORRECT_Y = 0.0;
float V_X = 0, V_Y = 0;
float SPEED = 7.0;//�ɰ��������ٶ�
uint8 i=0;//����������˵���һ�α����������ֵ

void Encoder_Init(void){
		//һ��QTIMER���� ����������������
    //������Ҫע��һ�£�����Ǵ���������ı���������������LSB����Ӧ����A������ DIR����Ӧ����B������ ���ɽ���
    //��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER0_C0 B��ʹ��QTIMER1_TIMER1_C1
   encoder_dir_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);    
    //��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER2_C2 B��ʹ��QTIMER1_TIMER3_C24
   encoder_dir_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);    
   encoder_dir_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C4);
   encoder_dir_init(QTIMER2_ENCODER2,QTIMER2_ENCODER2_CH1_C5,QTIMER2_ENCODER2_CH2_C25);
//	 qtimer_quad_init(QTIMER_4,QTIMER4_TIMER0_C9,QTIMER4_TIMER1_C10);
}

void get_corrd(void)
{

	if(i>=1)
	{
//		Motor1_Speed = encoder_get_count(QTIMER1_ENCODER1); //������Ҫע��ڶ������������дA������
		Motor2_Speed = encoder_get_count(QTIMER1_ENCODER2);
		Motor3_Speed = encoder_get_count(QTIMER1_ENCODER1);
		Motor4_Speed = encoder_get_count(QTIMER2_ENCODER1);
		Motor2_Position += encoder_get_count(QTIMER1_ENCODER2);
		Motor3_Position += encoder_get_count(QTIMER1_ENCODER1);
		Motor4_Position += encoder_get_count(QTIMER2_ENCODER1);
//				printf("ENCODER_1 counter \t%d .\r\n", Motor1_Speed);                 // ���������������Ϣ
//        printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // ���������������Ϣ  
//        printf("ENCODER_3 counter \t%d .\r\n", Motor3_Speed);                 // ���������������Ϣ
//        printf("ENCODER_4 counter \t%d .\r\n", Motor4_Speed);                 // ���������������Ϣ  
	}
	i++;
	if(i>=100) i=100;
	
	encoder_clear_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER2);
	
//	V_X = 0.333f*(0.5f*Motor2_Speed - Motor3_Speed + 0.5f*Motor4_Speed);
//	V_Y = 0.333f*(0.866f*Motor2_Speed  - 0.866f*Motor4_Speed);
}

void position_zero(void)
{
	Motor2_Position = 0;//�Һ󷽵��
  Motor3_Position = 0;//ǰ�����
  Motor4_Position = 0;//��󷽵��
}

