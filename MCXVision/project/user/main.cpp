/*********************************************************************************************************************
* MCX Vision Opensourec Library ����MCX Vision ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2024 SEEKFREE ��ɿƼ�
* 
* ���ļ��� MCX Vision ��Դ���һ����
* 
* MCX Vision ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MDK 5.38a
* ����ƽ̨          MCX Vision
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2024-04-21        ZSY            first version
********************************************************************************************************************/
#include "zf_model_process.h"
#if defined(__cplusplus)
extern "C" // mian�ļ���C++�ļ��������Ҫ����C���Ե�ͷ�ļ�������Ҫʹ��extern "C"
{
#endif /* __cplusplus */ 
#include "zf_common_headfile.h"
#include "color_tracer.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// ���İ������������Ҫ�ֶ���λ������
// ���İ������������Ҫ�ֶ���λ������
// ���İ������������Ҫ�ֶ���λ������.
    
// ����������к�ģ���ϵ�Core����˸��˵�����������HardFault������Ҫ����Ƿ�������Խ�磬����û�г�ʼ��������ʱ��û�����õ����
    
// *************************** ���̲���˵�� ***************************
// 1.ʹ��DAP����������ģ��,��IPS2.0��Ļ�嵽ģ���ϣ�ע�����ŷ���
// 
// 2.MCX Vision���ر����̣�
// 
// 3.��λ���İ壨��Ҫ�ֶ���λ�� 
// 
// 4.��Ļ����ʾʵʱͼ�񲢿��ɫ��Ŀ�꣬ͨ���������Ը�����Ļ������ɫ�л�ʶ��ɫ��

// **************************** �������� ****************************
    
// LED״̬ö��
enum 
{
    LED_ON = 0,
    LED_OFF = 1
}led_state_enum;


// LED���ƺ궨��
#define LED_RED(x)      gpio_set_level(gpio_led_red,    (uint8)x)
#define LED_GREEN(x)    gpio_set_level(gpio_led_green,  (uint8)x)
#define LED_BLUE(x)     gpio_set_level(gpio_led_blue,   (uint8)x)
#define LED_WHITE(x)    gpio_set_level(gpio_led_white,  (uint8)x)

// LED���Ŷ���
gpio_struct gpio_led_red =      {GPIO2, 8u};
gpio_struct gpio_led_green =    {GPIO2, 9u};
gpio_struct gpio_led_blue =     {GPIO2, 10u};
gpio_struct gpio_led_white =    {GPIO2, 11u};

// KEY���Ŷ���
gpio_struct gpio_key_1 =        {GPIO4, 2u};
double proportion;
int stop;   //stop signal
double stop_threshold = 0.70;  //time to stop the car
int findbox;
int deviation;
int direction;
int main(void)
{

    // ʱ�Ӻ͵��Դ���-����4��ʼ��
    zf_board_init();
    // �û�����-����5��ʼ��
    user_uart_init();
    // ��ʱ300ms
    system_delay_ms(10);
//    // ʹ��C++�����޷�ʹ��printf������ʹ��zf_debug_printf��zf_user_printf���
//    zf_debug_printf("debug_uart_init_finishok\r\n");  // ʹ�õ��Դ���-����4��������
//    zf_user_printf("user_uart_init_finish\r\n");    // ʹ���û�����-����5��������
    // LED��ʼ��
    gpio_init(gpio_led_red, GPO, 1, PULL_UP);
    gpio_init(gpio_led_green, GPO, 1, PULL_UP);
    gpio_init(gpio_led_blue, GPO, 1, PULL_UP);
    gpio_init(gpio_led_white, GPO, 1, PULL_UP);
    
    // ��ʼ������
    gpio_init(gpio_key_1, GPI, 0, PULL_UP);
    // ��Ļ��ʼ��
    ips200_init();
    // ����ͷ��ʼ��
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
                // ͨ��ͼ��������������ɫ��ʶ����ֵ
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
							if (deviation < 10)        //�������
							{
								direction=2;
								LED_RED(LED_ON);
								LED_GREEN(LED_OFF);
								LED_BLUE(LED_OFF);
							}
							if (deviation > -10)     //�����ұ�
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

// **************************** �������� ****************************
#if defined(__cplusplus)
}
#endif /* __cplusplus */