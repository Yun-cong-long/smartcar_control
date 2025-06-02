/*********************************************************************************************************************
* MCX Vision Opensourec Library 即（MCX Vision 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2024 SEEKFREE 逐飞科技
* 
* 本文件是 MCX Vision 开源库的一部分
* 
* MCX Vision 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.38a
* 适用平台          MCX Vision
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-04-21        ZSY            first version
********************************************************************************************************************/
#include "zf_model_process.h"
#if defined(__cplusplus)
extern "C" // mian文件是C++文件，如果需要包含C语言的头文件，就需要使用extern "C"
{
#endif /* __cplusplus */ 
#include "zf_common_headfile.h"
#include "color_tracer.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 核心板下载完代码需要手动复位！！！
// 核心板下载完代码需要手动复位！！！
// 核心板下载完代码需要手动复位！！！.
    
// 如果程序运行后，模块上的Core灯闪烁，说明程序进入了HardFault，就需要检测是否有数组越界，外设没有初始化，外设时钟没有设置等情况
    
// *************************** 例程测试说明 ***************************
// 1.使用DAP下载器连接模块,将IPS2.0屏幕插到模块上，注意引脚方向
// 
// 2.MCX Vision下载本例程，
// 
// 3.复位核心板（需要手动复位） 
// 
// 4.屏幕会显示实时图像并框出色块目标，通过按键可以根据屏幕中心颜色切换识别色彩

// **************************** 代码区域 ****************************
    
// LED状态枚举
enum 
{
    LED_ON = 0,
    LED_OFF = 1
}led_state_enum;


// LED控制宏定义
#define LED_RED(x)      gpio_set_level(gpio_led_red,    (uint8)x)
#define LED_GREEN(x)    gpio_set_level(gpio_led_green,  (uint8)x)
#define LED_BLUE(x)     gpio_set_level(gpio_led_blue,   (uint8)x)
#define LED_WHITE(x)    gpio_set_level(gpio_led_white,  (uint8)x)

// LED引脚定义
gpio_struct gpio_led_red =      {GPIO2, 8u};
gpio_struct gpio_led_green =    {GPIO2, 9u};
gpio_struct gpio_led_blue =     {GPIO2, 10u};
gpio_struct gpio_led_white =    {GPIO2, 11u};

// KEY引脚定义
gpio_struct gpio_key_1 =        {GPIO4, 2u};
double proportion;
int stop;   //stop signal
double stop_threshold = 0.70;  //time to stop the car
int findbox;
int deviation;
int direction;
int main(void)
{

    // 时钟和调试串口-串口4初始化
    zf_board_init();
    // 用户串口-串口5初始化
    user_uart_init();
    // 延时300ms
    system_delay_ms(10);
//    // 使用C++编译无法使用printf，可以使用zf_debug_printf和zf_user_printf替代
//    zf_debug_printf("debug_uart_init_finishok\r\n");  // 使用调试串口-串口4发送数据
//    zf_user_printf("user_uart_init_finish\r\n");    // 使用用户串口-串口5发送数据
    // LED初始化
    gpio_init(gpio_led_red, GPO, 1, PULL_UP);
    gpio_init(gpio_led_green, GPO, 1, PULL_UP);
    gpio_init(gpio_led_blue, GPO, 1, PULL_UP);
    gpio_init(gpio_led_white, GPO, 1, PULL_UP);
    
    // 初始化按键
    gpio_init(gpio_key_1, GPI, 0, PULL_UP);
    // 屏幕初始化
    ips200_init();
    // 摄像头初始化
    scc8660_init();
    scc8660_set_uniform_brightness(600);
    while (1)
    {
//			scc8660_set_brightness(50);
//			LED_WHITE(LED_ON); //open white LED
        if(scc8660_finish)
        {
            scc8660_finish = 0;
            
            ips200_show_scc8660((uint16_t*)g_camera_buffer);
            if(!gpio_get_level(gpio_key_1))
            {
                // 通过图像中心数据设置色块识别阈值
                set_color_target_condi((*((uint16*)g_camera_buffer + SCC8660_H/2 * SCC8660_W + SCC8660_W/2)), &target_color_condi);
            }
            
            if(color_trace(&target_color_condi, &target_pos_out))
            {
							findbox = 1;
                ips200_draw_line((target_pos_out.x - target_pos_out.w/2), (target_pos_out.y - target_pos_out.h/2), (target_pos_out.x + target_pos_out.w/2), (target_pos_out.y - target_pos_out.h/2), 0xffff);
                ips200_draw_line((target_pos_out.x - target_pos_out.w/2), (target_pos_out.y - target_pos_out.h/2), (target_pos_out.x - target_pos_out.w/2), (target_pos_out.y + target_pos_out.h/2), 0xffff);
                ips200_draw_line((target_pos_out.x - target_pos_out.w/2), (target_pos_out.y + target_pos_out.h/2), (target_pos_out.x + target_pos_out.w/2), (target_pos_out.y + target_pos_out.h/2), 0xffff);
                ips200_draw_line((target_pos_out.x + target_pos_out.w/2), (target_pos_out.y - target_pos_out.h/2), (target_pos_out.x + target_pos_out.w/2), (target_pos_out.y + target_pos_out.h/2), 0xffff);
						  proportion = ((double)(target_pos_out.w*target_pos_out.h))/((double)(320*240));
							deviation = 159 - target_pos_out.x;
							if (deviation < 10)        //车在左边
							{
								direction=2;
								LED_RED(LED_ON);
								LED_GREEN(LED_OFF);
								LED_BLUE(LED_OFF);
							}
							if (deviation > -10)     //车在右边
							{
								direction=3;
								LED_RED(LED_OFF);
								LED_GREEN(LED_OFF);
								LED_BLUE(LED_ON);
							}
							if (deviation > -10 && deviation <10 && proportion > 0.15)
							{
								direction=4;
								LED_RED(LED_OFF);
								LED_GREEN(LED_ON);
								LED_BLUE(LED_OFF);
							}
            }
						else
						{
							findbox = 0;
							stop = 0;
							deviation = 0;
							direction=1;
						}
//						zf_debug_printf("%d" , deviation);  //send deviation to RT1064
//					  zf_user_printf("%d" , deviation);
						 zf_debug_printf("%c" , direction);
			      	zf_user_printf("%c" , direction);
					  system_delay_ms(50);
        }
    }
}

void zf_debug_uart_callback(uint8 c)
{
	  if (c == 'A')
		{
			  zf_debug_printf("%d" , findbox);
				zf_user_printf("%d" , findbox);
		}
		if (c == 'B')
		{
    zf_debug_printf("%d" , direction);
	  zf_user_printf("%d" , direction);
		}
}

// **************************** 代码区域 ****************************
#if defined(__cplusplus)
}
#endif /* __cplusplus */