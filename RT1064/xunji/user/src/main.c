#include "zf_common_headfile.h"
#include "isr.h"
#include "motor_control.h"
#include "PIDController.h"
#include "encoder.h"
#include "stdio.h"
#include "imageprocess.h"
#include "imu963ra.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// *************************** ����Ӳ������˵�� ***************************
// ���������Ҷ���������ͷ ��Ӧ��������ͷ�ӿ� ��ע������
//      ģ��ܽ�            ��Ƭ���ܽ�
//      TXD                 �鿴 zf_device_mt9v03x.h �� MT9V03X_COF_UART_TX �궨��
//      RXD                 �鿴 zf_device_mt9v03x.h �� MT9V03X_COF_UART_RX �궨��
//      PCLK                �鿴 zf_device_mt9v03x.h �� MT9V03X_PCLK_PIN �궨��
//      VSY                 �鿴 zf_device_mt9v03x.h �� MT9V03X_VSYNC_PIN �궨��
//      D0-D7               �鿴 zf_device_mt9v03x.h �� MT9V03X_DATA_PIN �궨�� �Ӹö��忪ʼ�������˸�����
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ
// 
// ����2��IPSģ��
//      ˫������ ���������� Ӳ������
//      RD                  �鿴 zf_device_ips200.h �� IPS200_RD_PIN_PARALLEL8     �궨�� B0 
//      WR                  �鿴 zf_device_ips200.h �� IPS200_WR_PIN_PARALLEL8     �궨�� B1 
//      RS                  �鿴 zf_device_ips200.h �� IPS200_RS_PIN_PARALLEL8     �궨�� B2 
//      RST                 �鿴 zf_device_ips200.h �� IPS200_RST_PIN_PARALLEL8    �궨�� C19
//      CS                  �鿴 zf_device_ips200.h �� IPS200_CS_PIN_PARALLEL8     �궨�� B3 
//      BL                  �鿴 zf_device_ips200.h �� IPS200_BL_PIN_PARALLEL8     �궨�� C18
//      D0-D7               �鿴 zf_device_ips200.h �� IPS200_Dx_PIN_PARALLEL8     �궨�� B16/B17/B18/B19/D12/D13/D14/D15
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ
//      �������� SPI ������ Ӳ������
//      SCL                 �鿴 zf_device_ips200.h �� IPS200_SCL_PIN_SPI  �궨��  B0
//      SDA                 �鿴 zf_device_ips200.h �� IPS200_SDA_PIN_SPI  �궨��  B1
//      RST                 �鿴 zf_device_ips200.h �� IPS200_RST_PIN_SPI  �궨��  B2
//      DC                  �鿴 zf_device_ips200.h �� IPS200_DC_PIN_SPI   �궨��  C19
//      CS                  �鿴 zf_device_ips200.h �� IPS200_CS_PIN_SPI   �궨��  B3 
//      BL                  �鿴 zf_device_ips200.h �� IPS200_BLk_PIN_SPI  �궨��  C18
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ



// **************************** �������� ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // ˫������ ���������� ����궨����д IPS200_TYPE_PARALLEL8

#define UART1_INDEX              (UART_1   )                           // Ĭ�� UART_1
#define UART1_BAUDRATE           (115200)                           // Ĭ�� 115200
#define UART1_TX_PIN             (UART1_TX_B12  )                           // Ĭ�� UART1_TX_B12
#define UART1_RX_PIN             (UART1_RX_B13  )                           // Ĭ�� UART1_RX_B13
#define UART1_PRIORITY           (LPUART1_IRQn)                         // ��Ӧ�����жϵ��жϱ�� �� MIMXRT1064.h ͷ�ļ��в鿴 IRQn_Type ö����

#define UART4_INDEX              (UART_4   )                           // Ĭ�� UART_4
#define UART4_BAUDRATE           (115200)                           // Ĭ�� 115200
#define UART4_TX_PIN             (UART4_TX_C16  )                           // Ĭ�� UART1_TX_A9
#define UART4_RX_PIN             (UART4_RX_C17  )                           // Ĭ�� UART1_RX_A10
#define UART4_PRIORITY           (LPUART4_IRQn)                                  // ��Ӧ�����жϵ��жϱ�� �� MIMXRT1064.h ͷ�ļ��в鿴 IRQn_Type ö����

#define TRIG_PIN B9                //���峬�����������Ŷ˿�
#define ECHO_PIN B10               //���峬�����������Ŷ˿�
#define HUI_PIN D17

extern int found_letter_position;
extern int flag_cross;                 //ʮ�ֵ�ʶ���־
extern int flag_crossring;				//ʮ�ֻ���ʶ���־

extern uint8 Enter_Rings_Process;
extern uint8 Enter_Rings_Flag_1;
extern uint8 Enter_Rings_Flag_2;
extern uint8 Enter_Rings_Flag_3;
extern bool flag_stop_car;
extern int save_coord98;

extern int unload_times;

extern int state__2_state3_if_found;
extern int state93;
extern int varian;
extern int show_uart_data;


bool ready=0;
bool is_roundabout = 1;//0����1�ڻ�����
//uint8 flag_stop_Car = 0;
float angle=0.0;          
int8 duty = 10;
int angle_turn=0;       //��Ҫת�ĽǶ�
float move_angle = 90.0f;  //����ʻʱ�ĽǶ�
uint8 shibiehfk_state=0;
int roundabout_type=0;//1����ʮ�֣�2������Բ����3������Բ��
int state=0;    //1:����ѭ�� 2:�����췽�� 3:ʶ��ͼƬ���� 4,5:�������� 6,7:��������
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;        //����



uint8 uart_get_data[64];                                                        // ���ڽ������ݻ�����
uint8 fifo_get_data[64];                                                        // fifo �������������
uint8 get_data = 0;                                                             // �������ݱ���
uint32 fifo_data_count = 0;                                                     // fifo ���ݸ���
fifo_struct uart_data_fifo;
uint8 gpio_status;

int findbox = 0;
float bili = 0.0;
int bias_center = 0;
int UltrasonicGetLength(void);
int juli = 0;
int juli_flag = 0;
float shijian = 0.0;
uint8 get_uart1_data;
uint8 get_uart4_data;
uint32 go_time=0;
int time_flag = 0;
uint8 faajishu = 0;
int kongxian = 0;
int jishu = 0;
void zhouqipit_init(void);
void jieguoxianshi(int n,int count);
void text(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    system_delay_ms(100);           //�ȴ��������������ϵ����
    ips200_init(IPS200_TYPE);       //��ʾ����ʼ��
    ips200_show_string(0, 0, "mt9v03x init.");
	  system_delay_ms(100);
	  
//	//��ʼ��IO�˿�Ϊ���ģʽ������       ��ഫ������ʼ��
//    gpio_init(TRIG_PIN, GPO, 0, GPO_PUSH_PULL);
//	//��ʼ��IO�˿�Ϊ���ģʽ������
//    gpio_init(ECHO_PIN, GPI, 0, GPI_PULL_UP);
	
    timer_init(GPT_TIM_1, TIMER_US);                                            // ��ʱ��ʹ�� TIM_1 ʹ�ú��뼶����
	
	  system_delay_ms(100);	//��ʱ100ms���ȴ��������������ϵ�ɹ�
    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(100);                                                   // ����ʱ�������Ʊ�ʾ�쳣
    }
    ips200_show_string(0, 16, "init success.");
    system_delay_ms(100);
//		ips200_clear();
//		mt9v03x_set_exposure_time(60);      //�����ع�ʱ��
//    interrupt_global_enable(0);
    Motor_Init();    //�����ʼ��
		Encoder_Init();      //��������ʼ��
		MotorPID_DefaultInit();      //���PID��ʼ��
	  anglePID_DefaultInit();      //�Ƕ�PID��ʼ��
		imu660ra_init();             //�����ǳ�ʼ��
		gyroOffset_init();//��������Ư��ʼ��
		zhouqipit_init();     //�����жϳ�ʼ��
		gpio_init(B11, GPO, 0, GPO_PUSH_PULL);//������
		state = 1;    //��ʼѭ��
		int flag_motor2=0;
		int flag_motor3=0;
		int flag_motor4=0;
		int jieshu = 0;
		int jiaodujj = 0;
		int jiaodunow = 0;
		
    while(1)
    {
				if (pit0_state == 1)
					{
						Get_angle();              //��ȡ�Ƕ�
			      get_corrd();              //��ȡ��ǰ�����������ֵ
					  if(mt9v03x_finish_flag)
            {
			          angle_turn = Image_Process();
				    }
//							  text();
//							ips200_show_int(0, 248, state, 4);
//							move_control(50,Angle_z,angle_turn);  //80
//							ips200_show_int(0, 248, Angle_z, 10);
//							if (flag_stop_Car == 1)
//								set_motor_speed(0,0,0);
//							move_control(50,angle_turn);  //80
//							ips200_show_int(0, 200, angle_turn, 10);
//							Motor_Test();
//					uart_write_byte (UART_4, 'A');
//					system_delay_ms(2000);
//					fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
//					ips200_show_int(0, 200, fifo_data_count, 10);		
//          if(fifo_data_count != 0)                                                // ��ȡ��������
//          {
//              fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//              
//						  for (int i=0;i< fifo_data_count;i++)
//						  ips200_show_int(0, 216+16*i, fifo_get_data[i], 10);
//          }
//						if (flag_motor2 == 0 || flag_motor3 == 0 || flag_motor4 == 0)
//						{
//							flag_motor2=motor2_position(3000);
//							flag_motor3=motor3_position(-6000);
//							flag_motor4=motor4_position(3000);
//						}
//						else
//						{
//							motor3_position(9000);
//							motor4_position(-9000);
//						}
////						uart_write_byte (UART_1, 'A');
              if (state == -1)
							{
								move_control(100,0);  //80
								jieshu++;
//								ips200_show_int(0, 264, jieshu, 10);
								if (jieshu > 160)
								{
									state = 0;
									Motor_Clear();
								}
							}
				    	if (state == 1)     //����ѭ��
				    	{
								kongxian++;    //����ʱ��
								  if (flag_zebra == 1 && findbox == 1)  //ʶ�𵽰�������ûʶ�𵽺췽��
									{
										state = -1;
										jieshu = 0;
									}
									findbox = get_uart1_data;
								  ips200_show_int(0, 200, findbox, 2);
									jiaodunow = Angle_z;
									if (findbox == 1 || kongxian <100)  //ûʶ�𵽺췽��
									{
										move_control(150,angle_turn);
									}
									else
									{
										Motor_Clear();
//										system_delay_ms(100);
//										Motor_Clear();
										kongxian = 0;
										state = 2;
									}
				  		    
			        }
							if (state == 2)    //����췽��
							{
								findbox=get_uart1_data;
								ips200_show_int(0, 200, findbox, 2);
								if (Angle_z - jiaodunow < 10 && Angle_z - jiaodunow > -10)
								{
									if (findbox == 2)      //�������
	              {
		              speed_and_angle(10,jiaodunow);
                }
								if (findbox == 3)     //�����ұ�
                {
                  speed_and_angle(-10,jiaodunow);
                }
	              if (findbox == 4)    //������
	              {
//		              set_motor_speed(-30,0,30);
									text111(-12,0,12);
	              }
//								if (findbox == 1)
//								{
//									text111(10,0,-10);
//								}
	              if (findbox == 5)   
	              {
		              Motor_Clear();
//									system_delay_ms(100);
//									Motor_Clear();
									state = 3;
	              }
							  }
								else
								{
									text111(-Angle_z + jiaodunow,-Angle_z + jiaodunow,-Angle_z + jiaodunow);
								}
							}
							if (state == 3)     //�ж�ͼƬ����
					    {
								 system_delay_ms(1000);
						     uart_write_byte (UART_4, 'A');
								 faajishu++;
					       system_delay_ms(1000);
								 if (faajishu == 1)
				  	     {
									 fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
						       if(fifo_data_count == 3)                                                // ��ȡ��������
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//                     ips200_show_int(0, 200, fifo_data_count, 3);
							       int numsss = 0;
							       numsss = fifo_get_data[2] % 2;
				  		       if (numsss == 1)
									     state = 4;           //��������
								     if (numsss == 0)
									     state = 8;        //��������
                   }
                   if(fifo_data_count == 4)                                                // ��ȡ��������
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//                     ips200_show_int(0, 200, fifo_data_count, 3);
				  		       if (fifo_get_data[2] == 48 && fifo_get_data[3] <= 57)
									     state = 8;
								     else
							  		   state = 4;
                   }
								 }
								 else
								 {
									 fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
						       if(fifo_data_count == 2)                                                // ��ȡ��������
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//                     ips200_show_int(0, 200, fifo_data_count, 3);
							       int numsss = 0;
							       numsss = fifo_get_data[1] % 2;
				  		       if (numsss == 1)
									     state = 4;           //��������
								     if (numsss == 0)
									     state = 8;        //��������
                   }
                   if(fifo_data_count ==3)                                                // ��ȡ��������
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//                     ips200_show_int(0, 200, fifo_data_count, 3);
				  		       if (fifo_get_data[1] == 48 && fifo_get_data[2] <= 56)
									     state = 8;
								     else
							  		   state = 4;
                   }
								 }
								 jieguoxianshi(faajishu, fifo_data_count);
							  	 ips200_show_int(0, 248, state, 10);
//								 system_delay_ms(100);
								 jiaodunow = Angle_z;
								 jiaodujj = angle_turn;
								 Motor_Clear();
					     }
							if (state == 4)        //��������
							{
										 if (Angle_z - jiaodunow >= 70 - jiaodujj*0.7)
//								     if (Angle_z - 90 >= 70)      //������
		                 {
			                 Motor_Clear();
											 state = 5;
		                 }
		                 else
		                 {
			                 set_motor_speed(0,70,-15);
		                 }
							}
							if (state == 5)      //ǰ���Ƴ�����
							{
								if (time_flag == 0)
						     {
							     timer_start(GPT_TIM_1);           // ������ʱ 
							     time_flag = 1;
						     }
								go_time = timer_get(GPT_TIM_1); //��ȡ��ʱ��ʱ��
								set_motor_speed(-80,0,80);
								if (go_time >= 1000000 && gpio_get_level(HUI_PIN)==1)  //��һ��ʱ�䳬������
								{
									Motor_Clear();
									shijian = go_time;
									timer_stop(GPT_TIM_1);                      // ֹͣ��ʱ��
							    timer_clear(GPT_TIM_1);   		// ��ʱֵʹ����Ϻ�ǵ���������⵼���´μ�ʱ����0��ʼ
							    go_time = 0;
							    time_flag = 0;
								  position_zero();
									state = 6;
								}
							}
							if (state == 6)          //�ص�����
							{
								if (time_flag == 0)
						     {
							     timer_start(GPT_TIM_1);           // ������ʱ 
							     time_flag = 1;
						     }
								go_time = timer_get(GPT_TIM_1); //��ȡ��ʱ��ʱ��
								if (go_time <= shijian*0.6)
								{
									set_motor_speed(80,0,-80);
								}
								else
								{
									Motor_Clear();
									timer_stop(GPT_TIM_1);                      // ֹͣ��ʱ��
							    timer_clear(GPT_TIM_1);   		// ��ʱֵʹ����Ϻ�ǵ���������⵼���´μ�ʱ����0��ʼ
							    go_time = 0;
							    time_flag = 0;
								  position_zero();
									state = 7;
								}
							}
							if (state == 7)       //��ԭ�Ƕ�
							{
			          set_motor_speed(-30,-30,-30);
								if (Angle_z - jiaodunow <= 15)
//								if (Angle_z - 90 <= 40)
		            {
//									if (angle_turn <= 5 && angle_turn >= -5)
//									{
			               Motor_Clear();
									   state = 1;
//									}
		            }
							}
							if (state == 8)          //��������
							{
								if (Angle_z - jiaodunow <= -70 - jiaodujj*0.7)
//								if (Angle_z - 90 <= -70)         //������
		            {
			            Motor_Clear();
								  state = 9;
		            }
		            else
		            {
			            set_motor_speed(15,-60,0);
		            }
						  }
							if (state == 9)            //ǰ��
							{
								if (time_flag == 0)
						     {
							     timer_start(GPT_TIM_1);           // ������ʱ 
							     time_flag = 1;
						     }
								go_time = timer_get(GPT_TIM_1); //��ȡ��ʱ��ʱ��
								set_motor_speed(-80,0,80);
								if (go_time >= 1000000 && gpio_get_level(HUI_PIN)==1)  //��һ��ʱ�䳬������
								{
									Motor_Clear();
									shijian = go_time;
									timer_stop(GPT_TIM_1);                      // ֹͣ��ʱ��
							    timer_clear(GPT_TIM_1);   		// ��ʱֵʹ����Ϻ�ǵ���������⵼���´μ�ʱ����0��ʼ
							    go_time = 0;
							    time_flag = 0;
								  position_zero();
									state = 10;
								}
							}
							if (state == 10)         //����
							{
								if (time_flag == 0)
						     {
							     timer_start(GPT_TIM_1);           // ������ʱ 
							     time_flag = 1;
						     }
								go_time = timer_get(GPT_TIM_1); //��ȡ��ʱ��ʱ��
								if (go_time <= shijian*0.6)
								{
									set_motor_speed(80,0,-80);
								}
								else
								{
									Motor_Clear();
									timer_stop(GPT_TIM_1);                      // ֹͣ��ʱ��
							    timer_clear(GPT_TIM_1);   		// ��ʱֵʹ����Ϻ�ǵ���������⵼���´μ�ʱ����0��ʼ
							    go_time = 0;
							    time_flag = 0;
								  position_zero();
									state = 11;
								}
							}
							if (state == 11)
							{
								set_motor_speed(30,30,30);
								if (Angle_z - jiaodunow >= -15)
//								if (Angle_z - jiaodunow >= -40)
		            {
//									if (angle_turn <= 5 && angle_turn >= -5)
//									{
			               Motor_Clear();
									   state = 1;
//									}
		            }
							}
					  pit0_state = 0;                   // ��������жϴ�����־λ
			  	}
    }
}

//int UltrasonicGetLength(void)
//{
//	    int lengthtemp = 0;
//	    int sum = 0;
//	    int distance = 0;
//	    long wait_time = 0;                //��ʱ������
//      uint32 distance_time = 0;          //����ʱ�����

//	for (wait_time=0;wait_time<3;wait_time++)
//	{
//	      gpio_set_level(TRIG_PIN,1);                           //������������ߵ�ƽ
//        system_delay_ms(15);                           //��ʱ10us    
//        gpio_set_level(TRIG_PIN,0);
//				
//        while(gpio_get_level(ECHO_PIN)==0);                     //��⵽��������Ϊ�ߵ�ƽ��ʼ��ʱ
//        timer_start(GPT_TIM_1);                                                     // ������ʱ 
//        while(gpio_get_level(ECHO_PIN)==1);                       //��⵽��������Ϊ�͵�ƽ�������ʱ
//        timer_stop(GPT_TIM_1);                      // ֹͣ��ʱ��
//        distance_time = timer_get(GPT_TIM_1); //��ȡ��ʱ��ʱ��
//		    distance = distance_time*340/2/1000;            //�������  ��λ����
//		    sum = distance + sum;
//	    	timer_clear(GPT_TIM_1);   		// ��ʱֵʹ����Ϻ�ǵ���������⵼���´μ�ʱ����0��ʼ
//	}
//        lengthtemp = sum/3;
//        ips200_show_int(0, 300, lengthtemp, 10);
//	      return lengthtemp;
//}
void uart_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // �������� while �ȴ�ʽ ���������ж�ʹ��
    uart_query_byte(UART4_INDEX, &get_data);                                     // �������� ��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // ������д�� fifo ��
}

void zhouqipit_init(void)
{
      pit_ms_init(PIT_CH0,5);  	//��ʼ��pitͨ��0 ����5ms
      pit_ms_init(PIT_CH1,1);  	//��ʼ��pitͨ��1 ����1ms
//		pit_ms_init(PIT_CH2,1); 	//��ʼ��pitͨ��2 ����1ms
//  	pit_ms_init(PIT_CH3,20); 	//��ʼ��pitͨ��3 ����500ms 

		NVIC_SetPriority(PIT_IRQn,0);  	//�����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ�� ��·PIT����һ��PIT�жϺ���
}

void jieguoxianshi(int n, int count)
{

	if (n == 1)
	{
		ips200_clear();
		fifo_get_data[0]=fifo_get_data[1];
		fifo_get_data[1]=fifo_get_data[2];
		fifo_get_data[2]=fifo_get_data[3];
		count --;
	}

//	if (n >= 1)
//	{
		if (count == 3)
		{
			fifo_get_data[0]=fifo_get_data[0]-48;
			fifo_get_data[1]=fifo_get_data[1]-48;
			fifo_get_data[2]=fifo_get_data[2]-48;
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[0] == 1)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "wrench");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 2)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	   		ips200_show_string(120, n*16, "soldering_iron");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 3)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16,  "electrodrill");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 4)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "tape_measure");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 5)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
		  	ips200_show_string(120, n*16, "screwdriver");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 6)
  		{
  			ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "pliers");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 7)
	  	{
		  	ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "oscillograph");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 8)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120,  n*16, "multimeter");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 0 && fifo_get_data[2] == 9)
  		{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16,  "printer");
		  }
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 0)
	  	{
		  	ips200_show_int(0, n*16, n, 3);
		  	ips200_show_string(120,  n*16, "keyboard");
		  }
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 1)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16,  "mobile_phone");
	   	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 2)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16,  "mouse");
	  	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 3)
	  	{
		  	ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "headphones");
	   	}
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 4)
	  	{
	  		ips200_show_int(0, n*16, n, 3);
	  		ips200_show_string(120, n*16, "monitor");
		  }
	  	if (fifo_get_data[0] == 1 && fifo_get_data[1] == 1 && fifo_get_data[2] == 5)
		  {
			  ips200_show_int(0, n*16, n, 3);
		   	ips200_show_string(120, n*16, "speaker");
		  }
	  }
		if (count == 2)
		{
			ips200_show_int(0, n*16, n, 3);
			ips200_show_int(120, n*16, fifo_get_data[0]-48, 1);
			ips200_show_int(136, n*16, fifo_get_data[1]-48, 1);
		}
//	}
}

void text(void)
{
//	//�������
//	SetMotorCurrent(2,-1700);   //800�ٽ�
//	SetMotorCurrent(3,2550);  //Ŀǰ������
//  SetMotorCurrent(4,-1700);
//	  set_motor_speed(-30,60,-30);
//	speed_and_angle(10,90);
//	text111(10,0,-10);
//	text111(0,30,0);
	  speed_and_angle(10,90);
//	set_motor_speed(0,70,-15);
	
	//����������
//	get_corrd();              //��ȡ��ǰ�����������ֵ
//	set_motor_speed(20,40,0);
//	speed_and_angle(60,0);
//	char speed_str[16];  // ����һ���㹻����ַ��������洢ת������ַ���
//  sprintf(speed_str, "%d", Motor3_Speed);  // ������ת��Ϊ�ַ���
//	uart_write_string (UART_4, speed_str);
////	printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // ���������������Ϣ  
////	ips200_show_int(0, 200, Motor2_Speed,4);   
//	ips200_show_int(0, 216, Motor3_Speed,4);  
//	ips200_show_int(0, 232, Motor4_Speed,4);  
//	ips200_show_int(0, 200, Motor2_Position,8);   
//	ips200_show_int(0, 216, Motor3_Position,8);  
//	ips200_show_int(0, 232, Motor4_Position,8); 
//  printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // ���������������Ϣ  
//	 printf("ENCODER_3 counter \t%d .\r\n", Motor3_Speed);                 // ���������������Ϣ  
//	 printf("ENCODER_4 counter \t%d .\r\n", Motor4_Speed);	
	
//	//�����ǲ���
//	Get_angle();
//	ips200_show_int(0, 200, Angle_z, 10);
	
//	//MCX����ͷ����
//	findbox=get_uart1_data;
//	ips200_show_int(0, 200, findbox, 10);
	
//	//openart����
//	uart_write_byte (UART_4, 'A');
//	system_delay_ms(2000);
////	ips200_clear();
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
//	ips200_show_int(0, 200, fifo_data_count, 10);		
//  if(fifo_data_count != 0)                                                // ��ȡ��������
//  {
//     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//              
//		 for (int i=0;i< fifo_data_count;i++)
//		ips200_show_int(0, 216+16*i, fifo_get_data[i], 10);
//  }
	
//	//�������
//	uart_write_byte (UART_4, 'A');
//	faajishu++;
//	system_delay_ms(2000);
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
//	fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
//	jieguoxianshi(faajishu, fifo_data_count);


//    //��ʾ������
//	uart_write_byte (UART_4, 'A');
//	faajishu++;
//	system_delay_ms(2000);
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
//	ips200_show_int(0, faajishu*16, faajishu, 3);
//	ips200_show_string(120, faajishu*16, "monitor");

//  //�췽�����
//	findbox=get_uart1_data;
//	ips200_show_int(0, 200, findbox, 2);
//	if (findbox == 1)
//	{
//		Motor_Clear();
//	}
//  if (findbox == 2)      //�������
//	{
////    speed_and_angle(50,0);  //80
//		speed_and_angle(8,0);
//    //									right_forward(25);
//  }
//  if (findbox == 3)     //�����ұ�
//  {
//    speed_and_angle(-8,0);
//    //left_forward(25);
//   }
//	 if (findbox == 4)    //������
//	 {
//		 text111(-10,0,10);
//	 }
//	 if (findbox == 5)
//	 {
//		 Motor_Clear();
//	 }	
//     jishu++;
//		 if (jishu == 5)
//		 {
//			 
//			 jishu = 0;
//		 

//     //�����Ӳ���
//		 if (Angle_z - 90 >= 35)
//		 {
//			 Motor_Clear();
//		 }
//		 else
//		 {
//			 text111(0,30,0);
//		 }
       
//			 //ƽ�Ʋ���
//			 jishu++;
//			 if (jishu <= 300)
//			   speed_and_angle(5,90);
//			 else
//				 speed_and_angle(-5,90);

//       //�Ҷȴ���������
//			 if (gpio_get_level(HUI_PIN)==1)    //ʶ�𵽰�Ϊ0������Ϊ1
//				 text111(0,30,0);
//			 else
//				 text111(0,0,0);

//      //����������
//      gpio_set_level(B11, 1);   //����������
//			gpio_set_level(B11, 0);   //�رշ�����
}