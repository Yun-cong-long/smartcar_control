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
} COORD_t;   //坐标类结构体
/*
typedef struct{
	int Type1;   //大类
	int Type2;   //小类
	int get_flag;  
}	TYPE_t;   //卡片种类结构体
*/
typedef struct{
	int x;  
	int y;
} POSITION;

extern COORD_t correct_coord;//矫正的坐标
/*
extern uint8 openart[100];
extern POSITION target_position[31];
extern TYPE_t classfication;
extern uint8 all_coord;     //接收到的坐标总数
extern bool GetAllCoord;


extern COORD_t find_coord;//目标检测的坐标

extern uint8 uart1_data;
extern uint8 uart2_data;
extern int16 art2_data;   //art2的接收数据*/

void openart1_uart_init(void);
void openart2_uart_init(void);
void openart3_uart_init(void);
void MT9V034_uart_init(void);
void conmmuication_init();
void find_left_ring();
#endif

