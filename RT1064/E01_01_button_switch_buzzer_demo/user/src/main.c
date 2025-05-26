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
//      �����İ���������� ȷ���嵽�׺��İ������������û�з�϶����
//      �����İ���������� ȷ���嵽�׺��İ������������û�з�϶����
//      �����İ���������� ȷ���嵽�׺��İ������������û�з�϶����
// 
//      ���尴��            ��Ƭ���ܽ�
//      S1                  �鿴 zf_device_key.h �ļ��� KEY1_PIN �궨������� Ĭ�� C15
//      S2                  �鿴 zf_device_key.h �ļ��� KEY2_PIN �궨������� Ĭ�� C14
//      S3                  �鿴 zf_device_key.h �ļ��� KEY3_PIN �궨������� Ĭ�� C13
//      S4                  �鿴 zf_device_key.h �ļ��� KEY4_PIN �궨������� Ĭ�� C12
// 
//      ���岦�뿪��        ��Ƭ���ܽ�
//      S5-1                C26
//      S5-2                C27
// 
//      ���������          ��Ƭ���ܽ�
//      BEEP                B11


// *************************** ���̲���˵�� ***************************
// 1.���İ���������� ����ʹ�õ�ع��� ���ر�����
// 
// 2.��λ���İ� LED������˸���� ������Ҳ��������
// 
// 3.�̰�һ�� S1-S4 �����ⰴ�� BEEP ����һ��
// 
// 4.���� S1-S4 �����ⰴ�� BEEP ��һֱ��
// 
// 5.���� SWITCH1/SWITCH2 ���뿪�� LED1 ���л���˸����Ϩ��״̬
// 
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�

// **************************** �������� ****************************
#define SWITCH1             (C27)
#define SWITCH2             (C26)

#define LED1                (B9)

#define BEEP                (B11)

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��
    
    system_delay_ms(300);           //�ȴ��������������ϵ����
    
    uint8 count_beep = 0;
    uint8 count_led = 0;

    // key_index_enum key_index_array[KEY_NUMBER] = {KEY_1,KEY_2,KEY_3,KEY_4};

    key_init(5);

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_set_level(LED1, GPIO_LOW);                                             // LED1 ��
    gpio_set_level(BEEP, GPIO_HIGH);                                            // BEEP ��
    system_delay_ms(100);
    gpio_set_level(LED1, GPIO_HIGH);                                            // LED1 ��
    gpio_set_level(BEEP, GPIO_LOW);                                             // BEEP ͣ
    system_delay_ms(100);
    gpio_set_level(LED1, GPIO_LOW);                                             // LED1 ��
    gpio_set_level(BEEP, GPIO_HIGH);                                            // BEEP ��
    system_delay_ms(100);
    gpio_set_level(LED1, GPIO_HIGH);                                            // LED1 ��
    gpio_set_level(BEEP, GPIO_LOW);                                             // BEEP ͣ
    system_delay_ms(100);
    interrupt_global_enable(0);
    
    while(1)
    {
        if(!gpio_get_level(SWITCH1) || !gpio_get_level(SWITCH2))                // SWITCH1/SWITCH2 ON
        {
            if(25 > count_led)
            {
                gpio_set_level(LED1, GPIO_LOW);                                 // LED1 ��
            }
            else
            {
                gpio_set_level(LED1, GPIO_HIGH);                                // LED1 ��
            }
        }
        else
        {
            gpio_set_level(LED1, GPIO_HIGH);                                    // LED1 ��
        }

        count_led = ((count_led != 100) ? (count_led + 1) : (1));
        
        key_scanner();

        if( KEY_SHORT_PRESS == key_get_state(KEY_1) ||
            KEY_SHORT_PRESS == key_get_state(KEY_2) ||
            KEY_SHORT_PRESS == key_get_state(KEY_3) ||
            KEY_SHORT_PRESS == key_get_state(KEY_4))                            // ���ⰴ���̰�
        {
            // �̰��İ������ɿ�ʱ ״̬�Żᱻ key_scanner ��λΪ KEY_SHORT_PRESS
            count_beep = 40;
            // ���Ե����������״̬
            key_clear_state(KEY_1);
            key_clear_state(KEY_2);
            key_clear_state(KEY_3);
            key_clear_state(KEY_4);
        }
        else if(KEY_LONG_PRESS == key_get_state(KEY_1) ||
                KEY_LONG_PRESS == key_get_state(KEY_2) ||
                KEY_LONG_PRESS == key_get_state(KEY_3) ||
                KEY_LONG_PRESS == key_get_state(KEY_4))                         // ���ⰴ������
        {
            // �����İ����ڰ����ڼ�ᱻ key_scanner ������λΪ KEY_LONG_PRESS
            // ���Լ�ʹ����˱��ε�״̬ ���´�ɨ��ʱ���ɻ��ж�Ϊ KEY_LONG_PRESS
            count_beep = 40;
            // Ҳ����������а���״̬
            key_clear_all_state();
        }

        if(count_beep)
        {
            gpio_set_level(BEEP, GPIO_HIGH);
            count_beep --;
        }
        else
        {
            gpio_set_level(BEEP, GPIO_LOW);
        }
        
        system_delay_ms(5);
    }
}

// **************************** �������� ****************************

// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// 
// ����1��S1-S4 ���� BEEP ����
//      ���ʹ��������ԣ��������Ҫ�õ�ع���
//      �鿴�����Ƿ�������¼���Ƿ����ر���ȷ���������¸�λ����
//      ���ñ������Ӧ BEEP ���ŵ�ѹ�Ƿ�仯��������仯֤������δ���У�����仯֤�� BEEP ��·����������
//      ���ñ����Ӧ S1-S4 ���ŵ�ѹ�Ƿ������仯���Ƿ�������źŲ����������Ƿ�Ӵ��Ƿ��·��·
// 
// ����2��SWITCH1 / SWITCH2 ������ LED ��Ӧ
//      ���ʹ��������ԣ��������Ҫ�õ�ع���
//      �鿴�����Ƿ�������¼���Ƿ����ر���ȷ���������¸�λ����
//      ���ñ������Ӧ LED ���ŵ�ѹ�Ƿ�仯��������仯֤������δ���У�����仯֤�� LED ������
//      ���ñ����Ӧ SWITCH1 / SWITCH2 ���ŵ�ѹ�Ƿ������仯���Ƿ�������źŲ����������Ƿ�Ӵ��Ƿ��·��·



