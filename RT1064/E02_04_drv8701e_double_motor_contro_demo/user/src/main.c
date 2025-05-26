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
// ʹ�� RT1064 ���İ�ֱ�ӽ��߽��в���
//      ģ��ܽ�            ��Ƭ���ܽ�																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																									
//      MOTOR1_DIR                C9
//      MOTOR1_PWM                C8
//      GND                 GND   
//      MOTOR2_DIR                C7
//      MOTOR2_PWM                C6
//      GND                 GND   
//      MOTOR3_DIR                D3
//      MOTOR3_PWM                D2
//      GND                 GND   
//      MOTOR4_DIR                C11
//      MOTOR4_PWM                C10
//      GND                 GND
//      ���߶��� +          �������
//      ���߶��� -          ��ظ���
// 
// ʹ�� RT1064 ѧϰ������в���
//      ��ģ��ĵ�Դ���߶�������������������������
//      ��ģ����źŽӿ�ʹ�����׻��������������źŽӿ����� ��ע����߷��� ��ȷ��������������ñ�ȷ��һ�� ���Ųο��Ϸ����İ�����
//      �������빩������ȷ����


// *************************** ���̲���˵�� ***************************
// 1.���İ���¼��ɱ����� �����ع���
// 
// 2.������˵�� ���Կ��������������ת
// 
// 3.���û�нӵ�� ʹ�����ñ���������������������ϲ����������ѹ�仯
// 
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�



// **************************** �������� ****************************
#define MAX_DUTY            (10 )                                               // ��� MAX_DUTY% ռ�ձ�
#define MOTOR1_DIR               (C9 )                                     
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR               (C7 )
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)                    //�Һ󷽵��

#define MOTOR3_DIR               (D2 )
#define MOTOR3_PWM               (PWM2_MODULE3_CHB_D3)                  //ǰ�����

#define MOTOR4_DIR               (C10 )
#define MOTOR4_PWM               (PWM2_MODULE2_CHB_C11)                 //��󷽵��
int8 duty = 10;
bool dir = true;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��
        
    system_delay_ms(300);           //�ȴ��������������ϵ����
    
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    interrupt_global_enable(0);
    
    while(1)
    {
//        if(duty >= 0)                                                           // ��ת
//        {
//            gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ
//            pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

            gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ(˳ʱ�룩
            pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

            gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ
            pwm_set_duty(MOTOR3_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

            gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                         // DIR����ߵ�ƽ
            pwm_set_duty(MOTOR4_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�
//        }
//        else                                                                    // ��ת
//        {
////            gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                          // DIR����͵�ƽ
////            pwm_set_duty(MOTOR1_PWM, (-duty) * (PWM_DUTY_MAX / 100));                // ����ռ�ձ�
//            
////            gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                          // DIR����͵�ƽ
////            pwm_set_duty(MOTOR2_PWM, (-duty) * (PWM_DUTY_MAX / 100));                // ����ռ�ձ�
//            
//            gpio_set_level(MOTOR3_DIR, GPIO_LOW);                                          // DIR����͵�ƽ
//            pwm_set_duty(MOTOR3_PWM, (-duty) * (PWM_DUTY_MAX / 100));                // ����ռ�ձ�
//            
////            gpio_set_level(MOTOR4_DIR, GPIO_LOW);                                          // DIR����͵�ƽ
////            pwm_set_duty(MOTOR4_PWM, (-duty) * (PWM_DUTY_MAX / 100));                // ����ռ�ձ�

//        }
//        if(dir)                                                                 // ���ݷ����жϼ������� �����̽����ο�
//        {
//            duty ++;                                                            // �������
//            if(duty >= MAX_DUTY)                                                // �ﵽ���ֵ
//            dir = false;                                                        // �����������
//        }
//        else
//        {
//            duty --;                                                            // �������
//            if(duty <= -MAX_DUTY)                                               // �ﵽ��Сֵ
//            dir = true;                                                         // �����������
//        }
//        system_delay_ms(50);
    }
}

// **************************** �������� ****************************

// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// 
// ����1�������ת����ģ�������ѹ�ޱ仯
//      ���ʹ��������ԣ��������Ҫ�õ�ع���
//      ���ģ���Ƿ���ȷ���ӹ��� ����ʹ�õ�Դ�߹��� ����ʹ�öŰ���
//      �鿴�����Ƿ�������¼���Ƿ����ر���ȷ���������¸�λ����
//      ���ñ������Ӧ PWM ���ŵ�ѹ�Ƿ�仯��������仯֤������δ���У����������𻵣����߽Ӵ����� ��ϵ�����ͷ�

