#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "zf_common_headfile.h"

#define target_num 12

typedef struct{
	float X_coord;  
	float Y_coord;
	int X_confirm_flag;
	int Y_confirm_flag;
	int position_flag;
} COORD_t;   //������ṹ��
/*
typedef struct{
	int Type1;   //����
	int Type2;   //С��
	int get_flag;  
}	TYPE_t;   //��Ƭ����ṹ��
*/
typedef struct{
	int x;  
	int y;
} POSITION;

extern COORD_t correct_coord;//����������
/*
extern uint8 openart[100];
extern POSITION target_position[31];
extern TYPE_t classfication;
extern uint8 all_coord;     //���յ�����������
extern bool GetAllCoord;


extern COORD_t find_coord;//Ŀ���������

extern uint8 uart1_data;
extern uint8 uart2_data;
extern int16 art2_data;   //art2�Ľ�������*/

void openart1_uart_init(void);
void openart2_uart_init(void);
void openart3_uart_init(void);
void MT9V034_uart_init(void);
void conmmuication_init();
void find_left_ring();
#endif

