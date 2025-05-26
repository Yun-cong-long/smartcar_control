/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
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
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// *************************** ����Ӳ������˵�� ***************************
//      ģ��ܽ�            ��Ƭ���ܽ�
//      ���������� Ӳ������
//      RD                  �鿴 zf_device_ips200.h �� IPS200_RD_PIN_PARALLEL8     �궨�� B0 
//      WR                  �鿴 zf_device_ips200.h �� IPS200_WR_PIN_PARALLEL8     �궨�� B1 
//      RS                  �鿴 zf_device_ips200.h �� IPS200_RS_PIN_PARALLEL8     �궨�� B2 
//      RST                 �鿴 zf_device_ips200.h �� IPS200_RST_PIN_PARALLEL8    �궨�� C19
//      CS                  �鿴 zf_device_ips200.h �� IPS200_CS_PIN_PARALLEL8     �궨�� B3 
//      BL                  �鿴 zf_device_ips200.h �� IPS200_BL_PIN_PARALLEL8     �궨�� C18
//      D0-D7               �鿴 zf_device_ips200.h �� IPS200_Dx_PIN_PARALLEL8     �궨�� B16/B17/B18/B19/D12/D13/D14/D15
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ
//      SPI ������ Ӳ������
//      SCL                 �鿴 zf_device_ips200.h �� IPS200_SCL_PIN_SPI  �궨��  B0
//      SDA                 �鿴 zf_device_ips200.h �� IPS200_SDA_PIN_SPI  �궨��  B1
//      RST                 �鿴 zf_device_ips200.h �� IPS200_RST_PIN_SPI  �궨��  B2
//      DC                  �鿴 zf_device_ips200.h �� IPS200_DC_PIN_SPI   �궨��  C19
//      CS                  �鿴 zf_device_ips200.h �� IPS200_CS_PIN_SPI   �궨��  B3 
//      BL                  �鿴 zf_device_ips200.h �� IPS200_BLk_PIN_SPI  �궨��  C18
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ



// *************************** ���̲���˵�� ***************************
// 1.���İ���¼������ ���������� 2��IPS ��ʾģ������������Ļ�ӿ������� ��ע�����Ŷ�Ӧ ��Ҫ���
// 
// 2.��ع��� �ϵ�� 2��IPS ��Ļ���� ��ʾ�ַ����ָ������Ͳ���ͼ
// 
// 3.�ж���ĻΪSPI��Ļ���߲�����Ļ: �鿴��Ļ�����PCB˿ӡ���������SPI������ΪSPI��Ļ������Ϊ������Ļ
//   �����ļ������в�ͬ����2����Ļ������ʾ��ͼ "SPI�Ͳ���2����Ļ����.png"
//   �޸��·���IPS200_TYPE�궨���������Ӧ��Ļ������
//
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�


// **************************** �������� ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // ���������� ����궨����д IPS200_TYPE_PARALLEL8
                                                                                // SPI ������ ����궨����д IPS200_TYPE_SPI

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��
        
    system_delay_ms(300);           //�ȴ��������������ϵ����
    
    uint16_t data[128];
    int16_t data_index = 0;
    for( ; 64 > data_index; data_index ++)
        data[data_index] = data_index;
    for(data_index = 64; 128 > data_index; data_index ++)
        data[data_index] = 128 - data_index;

    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
    interrupt_global_enable(0);
    
    while(1)
    {
        ips200_clear();
        ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_seekfree_logo, 240, 80, 240, 80, 0);    // ��ʾһ��RGB565ɫ��ͼƬ ԭͼ240*80 ��ʾ240*80 ��λ��ǰ
        system_delay_ms(1500);

        ips200_full(RGB565_GRAY);
        ips200_show_string( 0 , 16*7,   "SEEKFREE");                            // ��ʾ�ַ���
        ips200_show_chinese(80, 16*7,   16, (const uint8 *)chinese_test, 4, RGB565_BLUE);               // ��ʾ����

        // ��ʾ�� flaot ���� �����ʾ 8bit λ���� �����ʾ 6bit λС��
        ips200_show_float(  0 , 16*8,   -13.141592,     1, 6);                  // ��ʾ float ���� 1bit ���� 6bit С�� Ӧ����ʾ -3.141592 �ܹ����� 9 ���ַ�����ʾռλ
        ips200_show_float(  80, 16*8,   13.141592,      8, 4);                  // ��ʾ float ���� 8bit ���� 4bit С�� Ӧ����ʾ 13.1415 �ܹ����� 14 ���ַ�����ʾռλ ������� 5 ���ַ��Ŀհ�ռλ

        ips200_show_int(    0 , 16*9,   -127,           2);                     // ��ʾ int8 ����
        ips200_show_uint(   80, 16*9,   255,            4);                     // ��ʾ uint8 ����

        ips200_show_int(    0 , 16*10,  -32768,         4);                     // ��ʾ int16 ����
        ips200_show_uint(   80, 16*10,  65535,          6);                     // ��ʾ uint16 ����

        ips200_show_int(    0 , 16*11,  -2147483648,    8);                     // ��ʾ int32 ���� 8bit ���� Ӧ����ʾ -47483648
        ips200_show_uint(   80, 16*11,  4294967295,     8);                     // ��ʾ uint32 ���� 10bit ���� Ӧ����ʾ 4294967295

        system_delay_ms(1000);

        ips200_full(RGB565_GRAY);
        ips200_show_wave(88, 144, data, 128, 64,  64, 32);                      // ��ʾһ�����ǲ��� ���ο�� 128 �������ֵ 64 ��ʾ��� 64 ��ʾ���ֵ 32
        system_delay_ms(1000);
        ips200_full(RGB565_GRAY);
        ips200_show_wave(56, 128, data, 128, 64, 128, 64);                      // ��ʾһ�����ǲ��� ���ο�� 128 �������ֵ 64 ��ʾ��� 128 ��ʾ���ֵ 64
        system_delay_ms(1000);

        // ʹ�û��ߺ��� �Ӷ��������ǻ�����
        ips200_clear();
        for(data_index = 0; 240 > data_index; data_index += 10)
        {
            ips200_draw_line(0, 0, data_index, 320 - 1, RGB565_66CCFF);
            system_delay_ms(20);
        }
        ips200_draw_line(0, 0, 240 - 1, 320 - 1, RGB565_66CCFF);
        for(data_index = 310; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(0, 0, 240 - 1, data_index, RGB565_66CCFF);
            system_delay_ms(20);
        }

        ips200_draw_line(240 - 1, 0, 239, 320 - 1, RGB565_66CCFF);
        for(data_index = 230; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(240 - 1, 0, data_index, 320 - 1, RGB565_66CCFF);
            system_delay_ms(20);
        }
        ips200_draw_line(240 - 1, 0, 0, 320 - 1, RGB565_66CCFF);
        for(data_index = 310; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(240 - 1, 0, 0, data_index, RGB565_66CCFF);
            system_delay_ms(20);
        }
        system_delay_ms(1000);

        ips200_full(RGB565_RED);
        system_delay_ms(500);
        ips200_full(RGB565_GREEN);
        system_delay_ms(500);
        ips200_full(RGB565_BLUE);
        system_delay_ms(500);
        ips200_full(RGB565_WHITE);
        system_delay_ms(500);
    }
}

// **************************** �������� ****************************

// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// 
// ����1����Ļ����ʾ
//      ���ʹ��������ԣ��������Ҫ�õ�ع��� �����Ļ�������ŵ�ѹ
//      �����Ļ�ǲ��ǲ��λ���� ������Ŷ�Ӧ��ϵ
//      �����Ӧ���Ŷ���ȷ ���һ���Ƿ������Ų��β��� ��Ҫ��ʾ����
//      �޷���ɲ��β�������һ��GPIO���̽���Ļ����IO��ʼ��ΪGPIO��ת��ƽ �����Ƿ��ܿ�

