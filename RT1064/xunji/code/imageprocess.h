#ifndef _IMAGE_h
#define _IMAGE_h

#include  "stdio.h"
#include "zf_device_mt9v03x.h"

#define LCDH 60                              //����ͼ�����ͼ��߶ȣ��У�
#define LCDW 80                              //����ͼ�����ͼ���ȣ��У�
#define ImageSensorMid 39                    //ͼ�����Ļ�е�
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //�޷��޷�
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //�޷��޷�

extern float center_avg;
extern float idea_Med;

extern uint8 f_x,f_y;

#define BEEP_PIN   P33_10       //�������������

//ÿһ�е�����
typedef struct {
  uint8 IsRightFind;      //�ұ��б߱�־
  uint8 IsLeftFind;       //����б߱�־
  int Wide;               //�߽���
  int LeftBorder;         //��߽�
  int Left_BlackandWhite; //��߱ߵĺڰ�����
  int RightBorder;        //�ұ߽�
  int Right_BlackandWhite; //�ұߵĺڰ�����
  int Center;             //����
  int RightTemp;          //�ұ���ʱֵ
  int LeftTemp;           //�����ʱֵ
  int CenterTemp;         //������ʱֵ
} ImageDealDatatypedef;

typedef struct {
  //ͼ����Ϣ
  int OFFLine;            //ͼ�񶥱�
  uint8 OFFLine_Test;       //��ʱͼ�񶥱�
  uint8 WhiteLine;          //˫�߶�����
  uint8 Miss_Left_lines;    //���߶�ʧ
  uint8 Miss_Right_lines;   //���߶�ʧ

  uint8 Miss_Low_Right_lines; //����0-19 ����
  uint8 Miss_Middle_Right_lines; //����20-39 ����
  uint8 Miss_High_Right_lines; //����40-59 ����

  uint8 Miss_Low_Left_lines; //����0-19 ����
  uint8 Miss_Middle_Left_lines; //����20-39 ����
  uint8 Miss_High_Left_lines; //����40-59 ����

} ImageStatustypedef;

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

extern uint8 Threshold;
extern ImageDealDatatypedef ImageDeal[60];
extern uint8 Pixle_hb[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern ImageStatustypedef ImageStatus;
extern uint8 Image_Use[LCDH][LCDW];

extern uint8 Enter_Rings_Flag_1;
extern uint8 Enter_Rings_Flag_2;
extern uint8 Enter_Rings_Flag_3;
extern uint8 Enter_Rings_Flag_4;
extern uint8 Enter_Rings_A_Location;
extern uint8 Enter_Rings_Process;

extern uint8 flag_openRoad;
extern int flag_str_open;

extern int flag_openRoad_timeOut;
extern  int cnt_openRoad_timeOut;

extern float Angle_Sum_z;
extern float Angle_Sum_y;
extern float Angle_z;

extern int flag_1;
extern int flag_2;
extern int flag_3;
extern int flag_4;
extern uint8 flag_zebra;
extern uint8 flag_once;
extern uint8 flag_stop;
extern int findbox;
extern int flag_BiZhang;
extern int flag_Straight;
extern int flag_left_ring;
extern int left_ring_process;
extern int flag_right_ring;
extern int right_ring_process;
extern int flag_cross;
extern int flag_backIn;
extern int flag_straightBack;
extern uint8 Enter_Crosses_Process;
extern bool flag_stop_car;
extern float speed_init;
extern uint8 faajishu;
extern int flag_start_camera;
extern uint8 flag_out;
extern int flag_po;
extern int white_gray_for_barrier;





/*void Image_Compress(void);
void Get_BinaryImage(void);
void Pixle_Filter(void);

void Get_Border_And_SideType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q) ;
void Get_BaseLine(void);
void Get_AllLine(void);
void Get_ExtensionLine(void);
void Three_Forks_Test(void);*/

void Image_Compress(void);
uint8 Get_Threshold(uint8* image,uint16 col, uint16 row);
void Get_BinaryImage(void);
void Get_Border_And_SideType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q);
void Get_BaseLine(void);
void Get_AllLine(void);

void Element_Judgment_Left_Rings(void);
void Element_Judgment_Right_Rings(void);
void Garage_Identification(void);
void Element_Judgment_Crosses(void);

void find_openRoad(void);
void find_zebra(void);
void stop_car(void);
void road_type(void);
void road_type_2(void);
void find_left_ring(void);
void find_right_ring(void);
void find_cross(void);
void bu(void);
void fu_shi(void);
void ramp(void);
int Image_Process(void);
int barr_dect(void);
int is_angles_right(float* angles, float range, float current_angle);
int is_angle_right(float angle, float range, float current_angle);
float normalize_angle(float angle);
void find_left_ring_2(void);
void find_right_ring_2(void);
void find_right_ring_3(void);
float Slope_cal(void);
float Slope_cal_2(void);
void Center_line(void);
void Left_right_line(void);
#endif
