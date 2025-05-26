#include "zf_common_headfile.h"
#include "isr.h"
#include "motor_control.h"
#include "PIDController.h"
#include "encoder.h"
#include "stdio.h"
#include "imageprocess.h"
#include "imu963ra.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 接入总钻风灰度数字摄像头 对应主板摄像头接口 请注意线序
//      模块管脚            单片机管脚
//      TXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义
//      RXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义
//      PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN 宏定义
//      VSY                 查看 zf_device_mt9v03x.h 中 MT9V03X_VSYNC_PIN 宏定义
//      D0-D7               查看 zf_device_mt9v03x.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 
// 接入2寸IPS模块
//      双排排针 并口两寸屏 硬件引脚
//      RD                  查看 zf_device_ips200.h 中 IPS200_RD_PIN_PARALLEL8     宏定义 B0 
//      WR                  查看 zf_device_ips200.h 中 IPS200_WR_PIN_PARALLEL8     宏定义 B1 
//      RS                  查看 zf_device_ips200.h 中 IPS200_RS_PIN_PARALLEL8     宏定义 B2 
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_PARALLEL8    宏定义 C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_PARALLEL8     宏定义 B3 
//      BL                  查看 zf_device_ips200.h 中 IPS200_BL_PIN_PARALLEL8     宏定义 C18
//      D0-D7               查看 zf_device_ips200.h 中 IPS200_Dx_PIN_PARALLEL8     宏定义 B16/B17/B18/B19/D12/D13/D14/D15
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//      单排排针 SPI 两寸屏 硬件引脚
//      SCL                 查看 zf_device_ips200.h 中 IPS200_SCL_PIN_SPI  宏定义  B0
//      SDA                 查看 zf_device_ips200.h 中 IPS200_SDA_PIN_SPI  宏定义  B1
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_SPI  宏定义  B2
//      DC                  查看 zf_device_ips200.h 中 IPS200_DC_PIN_SPI   宏定义  C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_SPI   宏定义  B3 
//      BL                  查看 zf_device_ips200.h 中 IPS200_BLk_PIN_SPI  宏定义  C18
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源



// **************************** 代码区域 ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 双排排针 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8

#define UART1_INDEX              (UART_1   )                           // 默认 UART_1
#define UART1_BAUDRATE           (115200)                           // 默认 115200
#define UART1_TX_PIN             (UART1_TX_B12  )                           // 默认 UART1_TX_B12
#define UART1_RX_PIN             (UART1_RX_B13  )                           // 默认 UART1_RX_B13
#define UART1_PRIORITY           (LPUART1_IRQn)                         // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体

#define UART4_INDEX              (UART_4   )                           // 默认 UART_4
#define UART4_BAUDRATE           (115200)                           // 默认 115200
#define UART4_TX_PIN             (UART4_TX_C16  )                           // 默认 UART1_TX_A9
#define UART4_RX_PIN             (UART4_RX_C17  )                           // 默认 UART1_RX_A10
#define UART4_PRIORITY           (LPUART4_IRQn)                                  // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体

#define TRIG_PIN B9                //定义超声波触发引脚端口
#define ECHO_PIN B10               //定义超声波触发引脚端口

extern int found_letter_position;
extern int flag_cross;                 //十字的识别标志
extern int flag_crossring;				//十字环的识别标志

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
bool is_roundabout = 1;//0不在1在环岛内
//uint8 flag_stop_Car = 0;
float angle=0;          
int8 duty = 10;
int angle_turn=0;       //车要转的角度
float move_angle = 90.0f;  //车行驶时的角度
uint8 shibiehfk_state=0;
int roundabout_type=0;//1代表十字，2代表左圆环，3代表右圆环
int state=0;    //1:正常循迹 2:靠近红方块 3:识别图片类型 4,5:左推箱子 6,7:右推箱子
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;        //调试



uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区
uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_data_fifo;
uint8 gpio_status;

int findbox=0;
float bili = 0.0;
int bias_center = 0;
int UltrasonicGetLength(void);
int juli = 0;
int juli_flag = 0;
uint8 get_uart1_data;
uint8 get_uart4_data;
uint32 go_time=0;
int time_flag = 0;
uint8 faajishu = 0;
int kongxian = 0;
void zhouqipit_init(void);
void jieguoxianshi(int n,int count);
void text(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化
	  zhouqipit_init();     //周期中断初始化
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    system_delay_ms(100);           //等待主板其他外设上电完成
    ips200_init(IPS200_TYPE);       //显示屏初始化
    ips200_show_string(0, 0, "mt9v03x init.");
	  system_delay_ms(100);
	  
//	//初始化IO端口为输出模式且上拉       测距传感器初始化
//    gpio_init(TRIG_PIN, GPO, 0, GPO_PUSH_PULL);
//	//初始化IO端口为输出模式且上拉
//    gpio_init(ECHO_PIN, GPI, 0, GPI_PULL_UP);
	
    timer_init(GPT_TIM_1, TIMER_US);                                            // 定时器使用 TIM_1 使用毫秒级计数
	
	  system_delay_ms(100);	//延时100ms，等待主板其他外设上电成功
    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(100);                                                   // 短延时快速闪灯表示异常
    }
    ips200_show_string(0, 16, "init success.");
    system_delay_ms(100);
//		ips200_clear();
//		mt9v03x_set_exposure_time(60);      //设置曝光时间
//    interrupt_global_enable(0);
    Motor_Init();    //电机初始化
		Encoder_Init();      //编码器初始化
		MotorPID_DefaultInit();      //电机PID初始化
	  anglePID_DefaultInit();      //角度PID初始化
		imu660ra_init();             //陀螺仪初始化
		gyroOffset_init();//陀螺仪零漂初始化
		state = 0;    //开始循迹
		
		int flag_motor2=0;
		int flag_motor3=0;
		int flag_motor4=0;
		int jieshu = 0;
		int jiaodujj = 0;
		int jiaodunow = 0;
    while(1)
    {
        if(mt9v03x_finish_flag)
        {
					  if (pit0_state == 1)
						{
							Get_angle();
							get_corrd();              //读取当前电机编码器数值
							text();
						  angle_turn = Image_Process();
							ips200_show_int(0, 248, state, 4);
							
//							ips200_show_int(0, 248, Angle_z, 10);
//							if (flag_stop_Car == 1)
//								set_motor_speed(0,0,0);
//							move_control(50,angle_turn);  //80
//							ips200_show_int(0, 200, angle_turn, 10);
//							Motor_Test();
//					uart_write_byte (UART_4, 'A');
//					system_delay_ms(5000);
//							ips200_clear();
//					fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
//					ips200_show_int(0, 200, fifo_data_count, 10);		
//          if(fifo_data_count != 0)                                                // 读取到数据了
//          {
//              fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
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
								move_control(60,angle_turn);  //80
								jieshu++;
//								ips200_show_int(0, 264, jieshu, 10);
								if (jieshu > 160)
								{
									state = 0;
									Motor_Clear();
								}
							}
				    	if (state == 1)     //正常循迹
				    	{
								kongxian++;
//								if (pit1_state == 1)
//                {
								  if (flag_zebra == 1 && flag_Straight == 1)
									{
										state = -1;
										jieshu = 0;
									}
									findbox=get_uart1_data;
//				    		  ips200_show_int(0, 200, findbox, 10);
//				    	  	if (findbox == 1 && flag_left_ring == 0 && flag_right_ring == 0)    //没找到红方块
//				  	 	    {
//				  		  		move_control(60,angle_turn);  //80
//				  		    }
									if (flag_Straight == 1)
									{
									if (findbox == 1 ||  kongxian < 150)
									{
											move_control(90,angle_turn);  //80
									}
			  			    if (findbox == 2 && kongxian > 150)      //车在左边
			  			    {
			  			  	  move_control(30,6);  //80
//									right_forward(25);
				  		    }
				  		    if (findbox == 3 && kongxian > 150)
				  		    {
				  		  	  move_control(30,-6);  //80
//									left_forward(25);
				  		    }
								}
									if (flag_Straight == 0)
									{
										if (findbox == 1 ||  kongxian < 150)
									  {
											move_control(60,angle_turn);  //80
								  	}
			  			      if (findbox == 2 && kongxian > 150)      //车在左边
			  			      {
			  			    	  move_control(30,10);  //80
//									right_forward(25);
				  		      }
				  		      if (findbox == 3 && kongxian > 150)
				  		      {
				  		    	  move_control(30,-10);  //80
//							  		left_forward(25);
				  		      }
									}
				  		    if (findbox == 4)
				  		    {
				  		  	  state=3;
										position_zero();
										kongxian=0;
//				  			    set_motor_speed(0,0,0);
//				  		  	  position_zero();
//										PID_Clear(&Motor2_SpeedPID);
//										PID_Clear(&Motor3_SpeedPID);
//										PID_Clear(&Motor4_SpeedPID);
										Motor_Clear();
										system_delay_ms(500);
										Motor_Clear();
										jiaodujj = angle_turn;
										jiaodunow = Angle_z;
			  			    }
//									pit1_state = 0;
//							  }
			        }
//			 		    if (state == 2)   //靠近红方块
//					    {
//						    
//						    if (juli_flag == 0)
//				        {
//							    juli=UltrasonicGetLength();	
//							    juli_flag=1;
//						    }
//						    if (time_flag == 0)
//						    {
//							    timer_start(GPT_TIM_1);           // 启动定时 
//							    time_flag = 1;
//						    }
//						    go_time = timer_get(GPT_TIM_1); //获取定时的时间
//								if (go_time < 5000000)
//				        {
//									if (flag_motor2 == 0  || flag_motor4 == 0)
////									if (flag_motor2 == 0 || flag_motor3 == 0 || flag_motor4 == 0)
//						      {
//							      flag_motor2=motor2_position((juli-47)*10);
////							      flag_motor3=motor3_position(0);
//							      flag_motor4=motor4_position(-(juli-47)*10);
//									}  				
//						      else
//						      {
//							      state = 3;
//							      timer_stop(GPT_TIM_1);                      // 停止定时器
//						        timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//						  	    go_time = 0;
//							      time_flag = 0;
//							      juli_flag = 0;
//										SetMotorCurrent(2,0);
//										SetMotorCurrent(3,0);
//										SetMotorCurrent(4,0);
//						      }
//								}
//								else
//							    {
//								    state = 3;
//							      timer_stop(GPT_TIM_1);                      // 停止定时器
//						  	    timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							      go_time = 0;
//								    time_flag = 0;
//							      juli_flag = 0;
//										SetMotorCurrent(2,0);
//										SetMotorCurrent(3,0);
//										SetMotorCurrent(4,0);
//							    }			
//							
//								ips200_show_int(0, 216, state, 10);
//					    }
   				    if (state == 3)     //判断图片类型
					    {
						     uart_write_byte (UART_4, 'A');
								 faajishu++;
					       system_delay_ms(1000);
								 if (faajishu == 1)
				  	     {
									 fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
						       if(fifo_data_count == 3)                                                // 读取到数据了
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//                     ips200_show_int(0, 200, fifo_data_count, 3);
							       int numsss = 0;
							       numsss = fifo_get_data[2] % 2;
				  		       if (numsss == 1)
									     state = 4;           //左推箱子
								     if (numsss == 0)
									     state = 6;        //右推箱子
                   }
                   if(fifo_data_count == 4)                                                // 读取到数据了
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//                     ips200_show_int(0, 200, fifo_data_count, 3);
				  		       if (fifo_get_data[2] == 48 && fifo_get_data[3] <= 57)
									     state = 6;
								     else
							  		   state = 4;
                   }
								 }
								 else
								 {
									 fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
						       if(fifo_data_count == 2)                                                // 读取到数据了
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//                     ips200_show_int(0, 200, fifo_data_count, 3);
							       int numsss = 0;
							       numsss = fifo_get_data[1] % 2;
				  		       if (numsss == 1)
									     state = 4;           //左推箱子
								     if (numsss == 0)
									     state = 6;        //右推箱子
                   }
                   if(fifo_data_count ==3)                                                // 读取到数据了
                   {
                     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//                     ips200_show_int(0, 200, fifo_data_count, 3);
				  		       if (fifo_get_data[1] == 48 && fifo_get_data[2] <= 56)
									     state = 6;
								     else
							  		   state = 4;
                   }
								 }
								 jieguoxianshi(faajishu, fifo_data_count);
							  	 ips200_show_int(0, 248, state, 10);
								 system_delay_ms(100);
					     }
////					
					     if (state == 4)              //左推箱子
					     {
					     	 if (time_flag == 0)
						     {
							     timer_start(GPT_TIM_1);           // 启动定时 
							     time_flag = 1;
						     }
//						       set_motor_speed(45,-50,35);
								   set_motor_speed(50,-40,5);						 
//								   SetMotorCurrent(2,1850);
//									 SetMotorCurrent(3,-1600);
//									 SetMotorCurrent(4,-1000);
//								   move_control(50,10);  //80
						       go_time = timer_get(GPT_TIM_1); //获取定时的时间
								 if (angle_turn > 20)
								 {
						     if (go_time >2000000 && Motor2_Position > 3500)
						     {
						       state = 5;
							     timer_stop(GPT_TIM_1);                      // 停止定时器
							     timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							     go_time = 0;
							     time_flag = 0;
									 position_zero();
//									 SetMotorCurrent(2,0);
//									 SetMotorCurrent(3,0);
//									 SetMotorCurrent(4,0);
//									 PID_Clear(&Motor2_SpeedPID);
//									 PID_Clear(&Motor3_SpeedPID);
//									 PID_Clear(&Motor4_SpeedPID);
									 Motor_Clear();
							     system_delay_ms(100);
									 Motor_Clear();
						      }
								  }
								  else
									{
										if (go_time >1800000 && Motor2_Position > 3500)
						        {
						          state = 5;
							        timer_stop(GPT_TIM_1);                      // 停止定时器
							        timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							        go_time = 0;
							        time_flag = 0;
								    	position_zero();
//									 SetMotorCurrent(2,0);
//									 SetMotorCurrent(3,0);
//									 SetMotorCurrent(4,0);
//									 PID_Clear(&Motor2_SpeedPID);
//									 PID_Clear(&Motor3_SpeedPID);
//									 PID_Clear(&Motor4_SpeedPID);
									    Motor_Clear();
							        system_delay_ms(100);
									    Motor_Clear();
						        }
									}
					      }
				 	      if (state == 5)
					      {
						      if (time_flag == 0)
						      {
							      timer_start(GPT_TIM_1);           // 启动定时 
							      time_flag = 1;
						      }
									set_motor_speed(7,80,-64);
//						      left_forward(40);
//									SetMotorCurrent(3,2000);
//									SetMotorCurrent(4,-2000);
						      go_time = timer_get(GPT_TIM_1); //获取定时的时间
//									if (angle_turn > 40)
//									 {
//										 state = 1;
//						       	 timer_stop(GPT_TIM_1);                      // 停止定时器
//							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							       go_time = 0;
//						  	     time_flag = 0;
////										 SetMotorCurrent(2,0);
////										 SetMotorCurrent(3,0);
////										 SetMotorCurrent(4,0);
//										 Motor_Clear();
//						   	     system_delay_ms(2000);
//									 }
									
						      if (go_time > 1200000)
									{
						        if (go_time > 2500000 || flag_out == 1)
						        {
											if (Motor3_Position > 3000)
											{
							        state = 8;
							        timer_stop(GPT_TIM_1);                      // 停止定时器
							        timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							        go_time = 0;
							        time_flag = 0;
							  			kongxian = 0;
											position_zero();
//										set_motor_speed(0,0,0);
//										SetMotorCurrent(2,0);
//									  SetMotorCurrent(3,0);
//									  SetMotorCurrent(4,0);
//										PID_Clear(&Motor2_SpeedPID);
//									  PID_Clear(&Motor3_SpeedPID);
//									  PID_Clear(&Motor4_SpeedPID);
								  		Motor_Clear();
						 	        system_delay_ms(100);
							  			Motor_Clear();
											}
						        }
							  	}
					      }
////					
					       if (state == 6)            //右推箱子
					       {
						       if (time_flag == 0)
						       {
							       timer_start(GPT_TIM_1);           // 启动定时 
							       time_flag = 1;
						       }
//						       left_forward(40);
//									 SetMotorCurrent(4,-1900);
//									 SetMotorCurrent(3,1600);
//									 set_motor_speed(-35,50,-45);
									 set_motor_speed(-5,40,-50);
						       go_time = timer_get(GPT_TIM_1); //获取定时的时间
//									 if (go_time > 800000 && angle_turn < -10)
//									 {
//										 state = 7;
//							       timer_stop(GPT_TIM_1);                      // 停止定时器
//							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							       go_time = 0;
//							       time_flag = 0;
////										 SetMotorCurrent(2,0);
////										 SetMotorCurrent(3,0);
////										 SetMotorCurrent(4,0);
//										 Motor_Clear();
//							       system_delay_ms(2000);
//									 }
									 if (angle_turn < -20)
									 {
										 if (go_time > 2000000 && Motor4_Position < -3500)
						         {
							       state = 7;
							       timer_stop(GPT_TIM_1);                      // 停止定时器
							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							       go_time = 0;
							       time_flag = 0;
										 position_zero();
//										 SetMotorCurrent(2,0);
//										 SetMotorCurrent(3,0);
//										 SetMotorCurrent(4,0);
										 Motor_Clear();
							       system_delay_ms(100);
										 Motor_Clear();
						         }
									 }
									 else
									 {
										if (go_time > 1800000 && Motor4_Position < -3500)
						        {
							       state = 7;
							       timer_stop(GPT_TIM_1);                      // 停止定时器
							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							       go_time = 0;
							       time_flag = 0;
										 position_zero();
//										 SetMotorCurrent(2,0);
//										 SetMotorCurrent(3,0);
//										 SetMotorCurrent(4,0);
										 Motor_Clear();
							       system_delay_ms(100);
										 Motor_Clear();
						        }
								   }
				         }
					       if (state == 7)
					       {
						       if (time_flag == 0)
						       {
							       timer_start(GPT_TIM_1);           // 启动定时 
							       time_flag = 9;
						       }
//						       right_forward(40);
									 set_motor_speed(64,-80,-7);
						       go_time = timer_get(GPT_TIM_1); //获取定时的时间
//									 if (angle_turn < -40)
//									 {
//										 state = 1;
//						       	 timer_stop(GPT_TIM_1);                      // 停止定时器
//							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							       go_time = 0;
//						  	     time_flag = 0;
////										 SetMotorCurrent(2,0);
////										 SetMotorCurrent(3,0);
////										 SetMotorCurrent(4,0);
//										 Motor_Clear();
//						   	     system_delay_ms(2000);
//									 }
						       if (go_time > 1200000)
									 {
						         if (go_time > 2500000 || flag_out == 1)
						         {
											 if (Motor3_Position < -3000)
											 {
							         state = 9;
						         	 timer_stop(GPT_TIM_1);                      // 停止定时器
							         timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							         go_time = 0;
						  	       time_flag = 0;
								  		 kongxian = 0;
											 position_zero();
//							 			 SetMotorCurrent(2,0);
//										 SetMotorCurrent(3,0);
//										 SetMotorCurrent(4,0);
									  	 Motor_Clear();
						   	       system_delay_ms(100);
								 	  	 Motor_Clear();
											 }
						         }
							  	 }
					       }
								 if (state == 8)
					       {
						       if (time_flag == 0)
						       {
							       timer_start(GPT_TIM_1);           // 启动定时 
							       time_flag = 1;
						       }
//						       right_forward(40);
									 set_motor_speed(-7,-80,64);
						       go_time = timer_get(GPT_TIM_1); //获取定时的时间
//									 if (angle_turn < -40)
//									 {
//										 state = 1;
//						       	 timer_stop(GPT_TIM_1);                      // 停止定时器
//							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							       go_time = 0;
//						  	     time_flag = 0;
////										 SetMotorCurrent(2,0);
////										 SetMotorCurrent(3,0);
////										 SetMotorCurrent(4,0);
//										 Motor_Clear();
//						   	     system_delay_ms(2000);
//									 }
						       if (go_time > 1500000)
									 {
							         state = 1;
						         	 timer_stop(GPT_TIM_1);                      // 停止定时器
							         timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							         go_time = 0;
						  	       time_flag = 0;
								  		 kongxian = 0;
											 position_zero();
//							 			 SetMotorCurrent(2,0);
//										 SetMotorCurrent(3,0);
//										 SetMotorCurrent(4,0);
									  	 Motor_Clear();
						   	       system_delay_ms(100);
								 	  	 Motor_Clear();
							  	 }
					       }
								 if (state == 9)
					      {
						      if (time_flag == 0)
						      {
							      timer_start(GPT_TIM_1);           // 启动定时 
							      time_flag = 1;
						      }
									set_motor_speed(-64,80,7);
//						      left_forward(40);
//									SetMotorCurrent(3,2000);
//									SetMotorCurrent(4,-2000);
						      go_time = timer_get(GPT_TIM_1); //获取定时的时间
//									if (angle_turn > 40)
//									 {
//										 state = 1;
//						       	 timer_stop(GPT_TIM_1);                      // 停止定时器
//							       timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							       go_time = 0;
//						  	     time_flag = 0;
////										 SetMotorCurrent(2,0);
////										 SetMotorCurrent(3,0);
////										 SetMotorCurrent(4,0);
//										 Motor_Clear();
//						   	     system_delay_ms(2000);
//									 }
									
						      if (go_time > 1500000)
									{
							        state = 1;
							        timer_stop(GPT_TIM_1);                      // 停止定时器
							        timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
							        go_time = 0;
							        time_flag = 0;
							  			kongxian = 0;
											position_zero();
//										set_motor_speed(0,0,0);
//										SetMotorCurrent(2,0);
//									  SetMotorCurrent(3,0);
//									  SetMotorCurrent(4,0);
//										PID_Clear(&Motor2_SpeedPID);
//									  PID_Clear(&Motor3_SpeedPID);
//									  PID_Clear(&Motor4_SpeedPID);
								  		Motor_Clear();
						 	        system_delay_ms(100);
							  			Motor_Clear();
							  	}
								}
					
//					if (state == 3)
//					{
//						 uart_write_byte (UART_4, 'A');
//					   system_delay_ms(3000);
//				  	fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
//            if(fifo_data_count != 0)                                                // 读取到数据了
//            {
//                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//                ips200_show_int(0, 200, fifo_data_count, 10);
//				  		  if (fifo_get_data[1] == 48 && fifo_get_data[2] >= 57)
//									state = 4;
//								if (fifo_get_data[1] == 49)
//									state = 8;
//            }
//					}
//					
//					if (state == 4)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 5;
//						ips200_show_int(0, 216, state, 10);
//					}
//					if (state == 5)
//					{
//						left_or_right(20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 1000000)
//						{
//							state = 6;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
//					if (state == 6)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 7;
//					}
//					if (state == 7)
//					{
//						left_forward(20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 5000000)
//						{
//							state = 1;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
//					
//					if (state == 8)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 9;
//						ips200_show_int(0, 216, state, 10);
//					}
//					if (state == 9)
//					{
//						left_or_right(-20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 1000000)
//						{
//							state = 10;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
//					if (state == 10)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 11;
//					}
//					if (state == 11)
//					{
//						right_forward(20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 5000000)
//						{
//							state = 1;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
					
//					if (state == 3)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 4;
//						ips200_show_int(0, 216, state, 10);
//					}
//					if (state == 4)
//					{
//						left_or_right(20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 1000000)
//						{
//							state = 5;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
//					if (state == 5)
//					{
//						timer_start(GPT_TIM_1);           // 启动定时 
//						state = 6;
//					}
//					if (state == 6)
//					{
//						left_forward(20);
//						go_time = timer_get(GPT_TIM_1); //获取定时的时间
//						if (go_time > 5000000)
//						{
//							state = 1;
//							pit0_state = 0;
//							timer_stop(GPT_TIM_1);                      // 停止定时器
//							timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//							go_time = 0;
//							system_delay_ms(500);
//						}
//					}
					  pit0_state = 0;                   // 清空周期中断触发标志位
			  	}
        }
    }
}

//int UltrasonicGetLength(void)
//{
//	    int lengthtemp = 0;
//	    int sum = 0;
//	    int distance = 0;
//	    long wait_time = 0;                //超时检测变量
//      uint32 distance_time = 0;          //测距的时间变量

//	for (wait_time=0;wait_time<3;wait_time++)
//	{
//	      gpio_set_level(TRIG_PIN,1);                           //触发引脚输出高电平
//        system_delay_ms(15);                           //延时10us    
//        gpio_set_level(TRIG_PIN,0);
//				
//        while(gpio_get_level(ECHO_PIN)==0);                     //检测到接收引脚为高电平则开始计时
//        timer_start(GPT_TIM_1);                                                     // 启动定时 
//        while(gpio_get_level(ECHO_PIN)==1);                       //检测到接收引脚为低电平则结束计时
//        timer_stop(GPT_TIM_1);                      // 停止定时器
//        distance_time = timer_get(GPT_TIM_1); //获取定时的时间
//		    distance = distance_time*340/2/1000;            //计算距离  单位毫米
//		    sum = distance + sum;
//	    	timer_clear(GPT_TIM_1);   		// 计时值使用完毕后记得清除，避免导致下次计时不从0开始
//	}
//        lengthtemp = sum/3;
//        ips200_show_int(0, 300, lengthtemp, 10);
//	      return lengthtemp;
//}
void uart_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART4_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}

void zhouqipit_init(void)
{
      pit_ms_init(PIT_CH0,5);  	//初始化pit通道0 周期5ms
      pit_ms_init(PIT_CH1,1);  	//初始化pit通道1 周期1ms
//		pit_ms_init(PIT_CH2,1); 	//初始化pit通道2 周期1ms
//  	pit_ms_init(PIT_CH3,20); 	//初始化pit通道3 周期500ms 

		NVIC_SetPriority(PIT_IRQn,0);  	//设置中断优先级 范围0-15 越小优先级越高 四路PIT共用一个PIT中断函数
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
////	//电机测试
//	SetMotorCurrent(2,1500);   //800临界
//	SetMotorCurrent(3,1500);
//  SetMotorCurrent(4,1500);
//	  set_motor_speed(10,20,-10);
	
	//编码器测试
//	get_corrd();              //读取当前电机编码器数值
//	set_motor_speed(20,40,0);
//	speed_and_angle(60,0);
//	char speed_str[16];  // 定义一个足够大的字符数组来存储转换后的字符串
//  sprintf(speed_str, "%d", Motor3_Speed);  // 将整数转换为字符串
//	uart_write_string (UART_4, speed_str);
////	printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // 输出编码器计数信息  
////	ips200_show_int(0, 200, Motor2_Speed,4);   
//	ips200_show_int(0, 216, Motor3_Speed,4);  
//	ips200_show_int(0, 232, Motor4_Speed,4);  
//	ips200_show_int(0, 200, Motor2_Position,8);   
//	ips200_show_int(0, 216, Motor3_Position,8);  
//	ips200_show_int(0, 232, Motor4_Position,8); 
//  printf("ENCODER_2 counter \t%d .\r\n", Motor2_Speed);                 // 输出编码器计数信息  
//	 printf("ENCODER_3 counter \t%d .\r\n", Motor3_Speed);                 // 输出编码器计数信息  
//	 printf("ENCODER_4 counter \t%d .\r\n", Motor4_Speed);	
	
//	//陀螺仪测试
//	Get_angle();
//	ips200_show_int(0, 200, Angle_z, 10);
	
//	//MCX摄像头测试
//	findbox=get_uart1_data;
//	ips200_show_int(0, 200, findbox, 10);
	
//	//openart测试
//	uart_write_byte (UART_4, 'A');
//	system_delay_ms(2000);
////	ips200_clear();
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
//	ips200_show_int(0, 200, fifo_data_count, 10);		
//  if(fifo_data_count != 0)                                                // 读取到数据了
//  {
//     fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//              
//		 for (int i=0;i< fifo_data_count;i++)
//		ips200_show_int(0, 216+16*i, fifo_get_data[i], 10);
//  }
	
//	//结果测试
//	uart_write_byte (UART_4, 'A');
//	faajishu++;
//	system_delay_ms(2000);
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
//	fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//	jieguoxianshi(faajishu, fifo_data_count);


//    //显示屏测试
//	uart_write_byte (UART_4, 'A');
//	faajishu++;
//	system_delay_ms(2000);
//	fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
//	ips200_show_int(0, faajishu*16, faajishu, 3);
//	ips200_show_string(120, faajishu*16, "monitor");

  //红方块测试
	findbox=get_uart1_data;
	ips200_show_int(0, 200, findbox, 2);
	if (findbox == 1)
	{
		move_control(60,angle_turn);  //80
	}
  if (findbox == 2)      //车在左边
	{
//    speed_and_angle(50,0);  //80
		set_motor_speed(30,-30,0);
    //									right_forward(25);
  }
  if (findbox == 3)     //车在右边
  {
    set_motor_speed(0,43,-30);  //80
    //left_forward(25);
   }
	 if (findbox == 4)    //车靠后
	 {
		 set_motor_speed(30,0,-30);  //80
	 }
	 if (findbox == 5)   
	 {
		 set_motor_speed(0,0,0);
	 }
}