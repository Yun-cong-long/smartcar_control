#include "imageprocess.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ips200.h"
#include "zf_driver_delay.h"
#include "math.h"
#include "stdlib.h"
#include "zf_common_headfile.h"
#include "imu963ra.h"
extern float angle;


unsigned char Image_temp[120][188];

/*******�۲����*********/
int flag_1 = 0;             //��һ�α�־
int flag_2 = 0;             //�ڶ��α�־
int flag_3 = 0;             //�����α�־
int flag_4 = 0;
extern bool is_roundabout;
uint8 Image_Use[LCDH][LCDW];      //�����洢ѹ��֮��Ҷ�ͼ��Ķ�ά����,60*80
uint8 Pixle[LCDH][LCDW];          //ͼ����ʱ��������Ķ�ֵ��ͼ������
uint8 Pixle_hb[LCDH][LCDW];
uint8 Threshold;                                //ͨ����򷨼�������Ķ�ֵ����ֵ
uint8 Bsite = 0;                                    //������������־

int ImageScanInterval=5;                        //ɨ�ߵķ�Χ
static uint8* PicTemp;                          //һ�����浥��ͼ���ָ�����
static int IntervalLow = 0, IntervalHigh = 0;   //ɨ������������ޱ���
static int Ysite = 0, Xsite = 0;                //Ysite����ͼ����У�Xsite����ͼ����С�

int Y = 0;
int X1,X2;

int leftup_x = 0, leftup_y = 0,rightup_x = 0, rightup_y = 0,rightdown_x = 0, rightdown_y = 0,leftdown_x = 0, leftdown_y = 0,rightdown_xb = 0, rightdown_yb = 0,leftdown_xb = 0, leftdown_yb = 0;
uint8 cross_firstflag=0;//ʮ�ֳ��б�־
uint16 count_wnum=0;//�׵��������
uint8 left_count=0;//���ޱ��м���
uint8 right_count=0;//���ޱ��м���
uint8_t sele_open = 1;     //��·����ѡ��3������
uint8_t max_ring = 1;       //�����ϵ��ܻ���
uint8_t BZ_num = 1;         //�����ϵı�������
uint8_t num_gray = 150;     //��·�˳��ĻҶ���ֵ
float A=0;
float B=0;
/*uint8 cross_flag=0;//ʮ�ֱ�־
uint8 leftup_flag=0;
uint8 rightup_flag=0;
uint8 leftdown_flag=0;
uint8 rightdown_flag=0;//�ҵ��յ��־*/
extern uint8 faajishu;
static int BottomBorderRight = 79,              //59�е��ұ߽�
BottomBorderLeft = 0,                           //59�е���߽�
BottomCenter = 0;                               //59�е��е�
uint8 ExtenLFlag = 0;                           //������Ƿ���Ҫ���ߵı�־����
uint8 ExtenRFlag = 0;                           //�ұ����Ƿ���Ҫ���ߵı�־����
ImageDealDatatypedef ImageDeal[60];             //��¼������Ϣ�Ľṹ������
ImageStatustypedef ImageStatus;                 //ͼ����ĵ�ȫ�ֱ���
float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;
bool flag_stop_car = 0;
uint8 Enter_Crosses_Process=0;
static int ytemp = 0;                           //����е���ʱ����
static int TFSite = 0, FTSite = 0;              //���߼���б�ʵ�ʱ����Ҫ�õĴ���еı�����
static float DetR = 0, DetL = 0;                //��Ų���б�ʵı���
int Car_Barn_num = 0;
int Zebra_Cnt = 0;
int Zebra_Flag = 0;
uint8 Starting_lines = 0;//������ʶ��
uint8 Garage_Location = 0;//1 :���������   2 ���������ұ�
extern uint8 flag_stop_Car;

int Midline_Weight_Woefficient[60]={    0,0,0,0,0,        //����Ȩ��ϵ��5
                                        0,0,0,0,0,//10
                                        0,0,0,0,0,//15
                                        0,0,0,0,0,//20

                                        240,300,360,420,420,//25
                                        480,540,580,580,580,//30
                                        480,360,300,240,180,//35
                                        180,180,120,120,120,//40

                                        100,100,100,100,100,//45
                                        60,60,60,60,60,//50
                                        0,0,0,0,0,//55
                                        0,0,0,0,0,//60


                                   };//5.2
uint32 sum_Proportion=5800;
int Midline_Weight_Woefficient_ZHIJ[60]={  0,0,0,0,0,        //����Ȩ��ϵ��5
                                      10,10,10,10,10,//10
                                      20,20,20,20,20,//15
                                      100,100,100,100,100,//20
                                      120,120,120,120,120,//25
                                      120,120,120,120,120,//35
                                      100,100,100,100,100,//40
                                      60,60,60,60,60,//45
                                      20,20,20,20,20,//50
                                      10,10,10,10,10,//55
                                      0,0,0,0,0//60

                                   };



uint8 f_x,f_y;
uint8 Get_Rings_First_Flag;  //��ȡ������һ�α�־λ
uint8 Rings_Second_Flag=0;   //���������׶λ����2�߶������ ��ʮ���жϳ�ͻ�������õı�־λ
uint8 Enter_Rings_Process=0;
uint8 Enter_Rings_Flag=0;
float Out_Rings_Slope=0;//������б��
uint8 Enter_Rings_Process_2=0;//�ڶ��׶α�־
int Enter_Rings_2_Ysite=0;
int Enter_Rings_2_Xsite=0;
uint8 Enter_Rings_Flag_1=0;
uint8 Enter_Rings_Flag_2=0;
uint8 Enter_Rings_Flag_3=0;
uint8 Enter_Rings_Flag_4=0;
uint8 Enter_Rings_A_Location=0;
uint8 Rings_Nums_Left=0;

int flag_clear_inRing = 0;      //�뻷ʱ����������һ��

/******************����λ��**********************/
float center_avg = 39;
float idea_Med = 39;            //��������λ��

/******************��·�Ĳ���***********************/
uint8 flag_openRoad = 0;     //��·ʶ���־λ

int flag_openRoad_timeOut = 0;     //�����ѵ��ı�־
float K1 = 0.0;         //ֱ��1��б��
float K2 = 0.0;         //ֱ��2��б��
float b1 = 0.0;         //ֱ��1���׾�
float b2 = 0.0;         //ֱ��2���׾�

uint8 x_jiaoDian = 0;    //��ֱ�߽���,����Ļ�������Ǻ��������
uint8 y_jiaoDian = 0;
int loca_i_left = 0;        //��ߵĴ������µĵ�һ���ޱ���λ��
int loca_i_right = 0;        //�ұߵĴ������µĵ�һ���ޱ���λ��

int loca_j = 0;

//ֱ����·
//offline����10����������7�С�T������offline�ĵ�·����ĳ�����ڣ���offline��ֵ��ĳһ�����ڣ�����ΪΪֱ����·
uint8_t cnt_str_open = 0;
int flag_str_open = 0;

int32 cnt_gray = 0;         //һ���Ҷ�ֵ���������
//ʶ�𵽶�·�������ټ���Ƿ������˳���·������
int cnt_openRoad_timeOut = 0;
/***************************����Ԫ��ʶ��Ĳ���******************************/
int flag_Straight = 0;    //ֱ���ı�־
int Str_len = 0;          //ֱ������
int y_left_road_tempt = 0;        //��߼�¼��һ�ζ��ߴ�
int y_left_length = 0;

int y_right_road_tempt = 0;       //�ұ߼�¼��һ�ζ��ߴ�
int y_right_length = 0;

float a1 = 0.0;         //ֱ��1��б��(��)
float a2 = 0.0;         //ֱ��2��б��(��)
float BB1 = 0.0;         //ֱ��1���׾�(��)
float BB2 = 0.0;         //ֱ��2���׾�(��)

/********************�����ߺͳ���Ĳ���***********************/
uint8 flag_zebra = 0;
uint8 flag_once =0;     //������תֻ����һ��
uint8 flag_stop = 0;        //���ͣ����־

uint8 flag_out = 0;         //������ͣ����־
/*******************�󻷵��Ĳ���***********************/
//�տ���������ʱ������ĸ��յ㣬ʶ�������С����ǰ�˶����ܿ����Ĺյ��С��ͨ���������������
int left_ring_point_1 = 0;      //�󻷵��������ϵĵ�һ���յ��λ��
int left_ring_point_1_tempt = 0;
int flag_left_ring_point_1 = 0; //�󻷵��������ϵĵ�һ���յ��Ƿ��ҵ��ı�־

int left_ring_point_2 = 0;      //�󻷵��������ϵĵڶ����յ��λ��
int left_ring_point_2_tempt = 0;
int flag_left_ring_point_2 = 0;//�󻷵��������ϵĵڶ����յ��Ƿ��ҵ��ı�־

int left_ring_point_3 = 0;      //�󻷵��������ϵĵ������յ��λ��
int left_ring_point_3_tempt = 0;
int flag_left_ring_point_3 = 0;//�󻷵��������ϵĵ������յ��Ƿ��ҵ��ı�־

int left_ring_point_4 = 0;      //�󻷵��������ϵĵ��ĸ��յ��λ��
int left_ring_point_4_tempt = 0;
int flag_left_ring_point_4 = 0;//�󻷵��������ϵĵ��ĸ��յ��Ƿ��ҵ��ı�־

int flag_left_ring = 0;         //�󻷵�ʶ���־λ

int left_ring_process = 0;      //�󻷵�����
int point_num_left = 0;              //�յ�����
int zhuan = 0;
//��Ϊ���ͬʱ�������ߣ������Ҷ�����������ֱ�ߵ�б�ʺ��׾�
float k_bu_1 = 0.0;
float b_bu_1 = 0.0;

float k_bu_2 = 0.0;
float b_bu_2 = 0.0;

int max_2_3 = 0;                //�յ�2��3֮����߽����ֵ��λ��

int flag_clear_process_3 = 0;   //����3������ֻ����һ��
int flag_clear_process_4 = 0;   //����4������ֻ����һ��

/*******************�󻷵��Ĳ���***********************/
//�տ���������ʱ������ĸ��յ㣬ʶ�������С����ǰ�˶����ܿ����Ĺյ��С��ͨ���������������
int right_ring_point_1 = 0;      //�һ����������ϵĵ�һ���յ��λ��
int right_ring_point_1_tempt = 0;
int flag_right_ring_point_1 = 0; //�һ����������ϵĵ�һ���յ��Ƿ��ҵ��ı�־

int right_ring_point_2 = 0;      //�һ����������ϵĵڶ����յ��λ��
int right_ring_point_2_tempt = 0;
int flag_right_ring_point_2 = 0;//�һ����������ϵĵڶ����յ��Ƿ��ҵ��ı�־

int right_ring_point_3 = 0;      //�һ����������ϵĵ������յ��λ��
int right_ring_point_3_tempt = 0;
int flag_right_ring_point_3 = 0;//�һ����������ϵĵ������յ��Ƿ��ҵ��ı�־

int right_ring_point_4 = 0;      //�һ����������ϵĵ��ĸ��յ��λ��
int right_ring_point_4_tempt = 0;
int flag_right_ring_point_4 = 0;//�һ����������ϵĵ��ĸ��յ��Ƿ��ҵ��ı�־

int flag_right_ring = 0;         //�һ���ʶ���־λ

int right_ring_process = 0;      //�һ�������
int point_num_right = 0;              //�յ�����

//��Ϊ���ͬʱ�������ߣ������Ҷ�����������ֱ�ߵ�б�ʺ��׾�
float k_bu_1_right = 0.0;
float b_bu_1_right = 0.0;

float k_bu_2_right = 0.0;
float b_bu_2_right = 0.0;

int max_2_3_right = 0;                //�յ�2��3֮����߽����ֵ��λ��

int flag_clear_process_3_right = 0;   //����3������ֻ����һ��
int flag_clear_process_4_right = 0;   //����4������ֻ����һ��

/*************************ʮ�ֵĲ���********************************/
int flag_cross = 0;                 //ʮ�ֵ�ʶ���־
int flag_crossring=0;				//ʮ�ֻ���ʶ���־
int left_Ysite_1 = 0;               //ʮ������ߵ�һ���յ�
int left_Ysite_1_tempt = 0;
int flag_left_Ysite_1 = 0;          //ʮ������ߵ�һ���յ��Ƿ��ҵ��ı�־
int left_Ysite_2 = 0;               //ʮ������ߵڶ����յ�
int left_Ysite_2_tempt = 0;
int flag_left_Ysite_2 = 0;          //ʮ������ߵڶ����յ��Ƿ��ҵ��ı�־

int right_Ysite_1 = 0;              //ʮ���ұ��ߵ�һ���յ�
int right_Ysite_1_tempt = 0;
int flag_right_Ysite_1 = 0;         //ʮ���ұ��ߵ�һ���յ��Ƿ��ҵ��ı�־
int right_Ysite_2 = 0;              //ʮ���ұ��ߵڶ����յ�
int right_Ysite_2_tempt = 0;
int flag_right_Ysite_2 = 0;         //ʮ���ұ��ߵڶ����յ��Ƿ��ҵ��ı�־

//�������߲��ߵ�ʱ���ʹ����������Ԫ��ʶ���ֱ�߲�������


/**********************�����ǵĽǶ��ۼ�ֵ������������Ҫ��Ϊ������cpu����ʹ��*/
float Angle_Sum_z = 0.0;              //�Ƕ�ֵ�ۼ�


float speed_init = 0;                //60
int flag_BiZhang = 0;

int flag_start_camera = 0;           //�̶���ǳ����ʼʹ������ͷѭ���ı�־����ʼΪ0

/****************************ģ�����Ʋ���***********************************/
//NB NM NS ZO PS PM PB��-3 -2 -1 0 1 2 3
//ʹ�������������Ⱥ���
#define bias_range                      //ƫ�Χ
#define bias_dot_range                  //ƫ��仯�ʵķ�Χ
#define Kp_range                        //Kp�ı仯��Χ
#define speed_range                     //�ٶȸ���ֵ�ı仯��Χ

#define NB -3
#define NM -2
#define NS -1
#define ZO  0
#define PS  1
#define PM  2
#define PB  3

float fuzzy_bias[7]={0.0};              //ƫ���������
float fuzzy_bias_dot[7]={0.0};          //ƫ��仯�ʵ�������

float bias = 0.0;                       //ƫ��
float bias_last = 0.0;                  //�ϴε�ƫ��
float bias_dot = 0.0;                   //ƫ��仯��

float dot_Kp = 0.0;                     //ģ�������

//ģ������
int fuzzy_rule_Kp[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
                         {PB,PB,PM,PS,PS,ZO,NS},
                         {PM,PM,PM,PS,ZO,NS,NS},
                         {PM,PM,PS,ZO,NS,NM,NM},
                         {PS,PS,ZO,NS,NS,NM,NM},
                         {PS,ZO,NS,NM,NM,NM,NB},
                         {ZO,ZO,NM,NM,NM,NB,NB}
                        };                                      //����Kp��ģ������

int fuzzy_rule_speed_init[7][7];                                //�ٶ��趨ֵ��ģ������

/***********ת�䶪�߲�������****************************************************/
int point_1_left = 0;                //�Ӻ���ǰT��W��λ��
int point_2_left = 0;                //�Ӻ���ǰW��T��λ��,��point1�������W�ޱ��е��������������һ����ֵ����Ϊ��������ߣ��ñ���Ҫ������ֵ
int flag_1_left = 0;

int point_1_right = 0;                //�Ӻ���ǰT��W��λ��
int point_2_right = 0;                //�Ӻ���ǰW��T��λ��,��point1�������W�ޱ��е��������������һ����ֵ����Ϊ��������ߣ��ñ���Ҫ������ֵ
int flag_1_right = 0;


/*********************************�������***************************************/

int flag_backIn = 0;                //��ʼ�����ı�־
int flag_straightBack = 0;          //�ٺ���һС�ξ���

/**********************************�µ�����***************************************/
float k_left_now = 0.0;             //����ߵ�ǰ�������б��
float k_left_last = 0.0;            //������ϴμ������б��

float k_right_now = 0.0;             //�ұ��ߵ�ǰ�������б��
float k_right_last = 0.0;            //�ұ����ϴμ������б��

int posi_left = 0;                  //�����б��ͻ����������
int posi_right = 0;                 //�ұ���б��ͻ����������

int flag_po = 0;                        //�µ���־
int flag_po_clear_ang_y_once = 0;       //�µ�������ֻ���һ��
int flag_ang = 0;                       //���������£���������Ϊ���Ĺ��̵ı�־

/**********************�һ���***********************/
uint8 Rings_Nums_Right=0;
//��������
uint8_t cnt_ring = 0;
//���ܻ����������ı�־λ
uint8_t flag_ring_man = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Compress
//  @brief          ԭʼ�Ҷ�ͼ��ѹ������
//  @brief          ���þ��ǽ�ԭʼ�ߴ�ĻҶ�ͼ��ѹ����������Ҫ�Ĵ�С���������ǰ�ԭʼ80��170�еĻҶ�ͼ��ѹ����60��80�еĻҶ�ͼ��
//  @parameter      void
//  @return         void
//  @time           2022��12��18��
//  @Author
//  Sample usage:   Image_Compress();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_Compress(void)
{
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;         //����ԭʼ��ͼ��ߴ��������Ҫ��ͼ��ߴ�ȷ����ѹ��������
  for (i = 0; i < LCDH; i++)                            //����ͼ���ÿһ�У��ӵ����е���59�С�
  {
    row =i * div_h + 0.5;
    for (j = 0; j < LCDW; j++)                          //����ͼ���ÿһ�У��ӵ����е���79�С�
    {
      line =j * div_w + 0.5;
      Image_Use[i][j] = mt9v03x_image[row][line];       //mt9v03x_image����������ԭʼ�Ҷ�ͼ��Image_Use����洢������֮��Ҫ��ȥ�����ͼ�񣬵���Ȼ�ǻҶ�ͼ��Ŷ��ֻ��ѹ����һ�¶��ѡ�
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Threshold
//  @brief          �Ż�֮��ĵĴ�򷨡���򷨾���һ���ܹ����һ��ͼ����ѵ��Ǹ��ָ���ֵ��һ���㷨����
//  @parameter      image  ԭʼ�ĻҶ�ͼ������
//  @parameter      clo    ͼ��Ŀ�ͼ����У�
//  @parameter      row    ͼ��ĸߣ�ͼ����У�
//  @return         uint8
//  @time           2022��12��19��
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); �Ѵ��60��80�еĶ�άͼ������Image_Use��������������ͼ�����ֵ�����������ֵ����Threshold��
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8 Get_Threshold(uint8* image,uint16 col, uint16 row)
{
  #define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  uint8 threshold_j=0;
  uint8* data = image;                               //����һ��ָ�򴫽������image�����ָ�����data
  for (i = 0; i < GrayScale; i++)                    //�Ȱ�pixelCount��pixelPro��������Ԫ��ȫ����ֵΪ0
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  /**************************************ͳ��ÿ���Ҷ�ֵ(0-255)������ͼ���г��ֵĴ���**************************************/
  for (i = 0; i < height; i += 1)                   //����ͼ���ÿһ�У��ӵ����е���59�С�
  {
    for (j = 0; j < width; j += 1)                  //����ͼ���ÿһ�У��ӵ����е���79�С�
    {
      pixelCount[(int)data[i * width + j]]++;       //����ǰ�����ص������ֵ���Ҷ�ֵ����Ϊ����������±ꡣ
      gray_sum += (int)data[i * width + j];         //���������Ҷ�ͼ��ĻҶ�ֵ�ܺ͡�
    }
  }
  /**************************************ͳ��ÿ���Ҷ�ֵ(0-255)������ͼ���г��ֵĴ���**************************************/



  /**************************************����ÿ������ֵ���Ҷ�ֵ���������Ҷ�ͼ������ռ�ı���*************************************************/
  for (i = 0; i < GrayScale; i++)
  {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (threshold_j = 0; threshold_j < GrayScale-1; threshold_j++)
  {
    w0 += pixelPro[threshold_j];                          //�����������ÿ���Ҷ�ֵ�����ص���ռ�ı���֮�ͣ����������ֵı�����
    u0tmp += threshold_j * pixelPro[threshold_j];

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                            //����ƽ���Ҷ�
    u1 = u1tmp / w1;                            //ǰ��ƽ���Ҷ�
    u = u0tmp + u1tmp;                          //ȫ��ƽ���Ҷ�
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = threshold_j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }

      return threshold;

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BinaryImage
//  @brief          �Ҷ�ͼ���ֵ������
//  @brief          ����˼·���ǣ��ȵ���Get_Threshold���������õ���ֵ��Ȼ�����ԭʼ�Ҷ�ͼ���ÿһ�����ص㣬��ÿһ�����ص�ĻҶ�ֵ������ֵ�ƽϡ�
//  @brief          ������ֵ����Ͱ����Ǹ����ص��ֵ��ֵΪ1����Ϊ�׵㣩������͸�ֵΪ0����Ϊ�ڵ㣩����Ȼ����԰������ֵ��������ֻҪ���Լ����1��0˭�����˭����׾��С�
//  @brief          ������ǰ���ᵽ��60*80�������Ǿ�Ӧ��������ʲô��˼�˰ɣ��������ص��һ����80�����ص㣬һ��60�У�Ҳ����ѹ�����ÿһ��ͼ����4800�����ص㡣
//  @parameter      void
//  @return         void
//  @Author
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BinaryImage(void)
{
  Threshold = Get_Threshold(Image_Use[0], LCDW, LCDH);      //������һ���������ã�ͨ���ú������Լ����һ��Ч���ܲ���Ķ�ֵ����ֵ��
  uint8 i, j = 0;
  for (i = 0; i < LCDH; i++)                                //������ά�����ÿһ��
  {
    for (j = 0; j < LCDW; j++)                              //������ά�����ÿһ��
    {
      if (Image_Use[i][j] > Threshold)                      //��������ĻҶ�ֵ������ֵThreshold
      {
          Pixle[i][j] = 1;                                  //��ô������ص�ͼ�Ϊ�׵�
          Pixle_hb[i][j] = 255;
      }
      else //��������ĻҶ�ֵС����ֵThreshold
      {
          Pixle[i][j] = 0;                                  //��ô������ص�ͼ�Ϊ�ڵ�
          Pixle_hb[i][j] = 0;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Border_And_SideType
//  @brief          �õ����ߺͱ��ߵ����ͣ�������������߷�Ϊ���������ͣ�T���͡�W���ͺ�H���͡��ֱ�����������ߡ��ޱ߱��ߺʹ�������ߡ�
//  @parameter      p        ָ�򴫽��������һ��ָ�������
//  @parameter      type     ֻ����L������R���ֱ����ɨ����ߺ�ɨ�ұ��ߡ�
//  @parameter      L        ɨ����������� ��Ҳ���Ǵ���һ�п�ʼɨ��
//  @parameter      H        ɨ����������� ��Ҳ����һֱɨ����һ�С�
//  @parameter      Q        ��һ���ṹ��ָ��������Լ�����ȥ��������ṹ������ĳ�Ա��
//  @time           2022��12��20��
//  @Author
//  Sample usage:   Get_SideType_And_Border(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);
//  Sample usage:   ��PicTemp(PicTemp�Ǹ�ָ�룬ָ��һ������)��IntervalLow�п�ʼɨ��ɨ��IntervalHigh�У�Ȼ��ѵõ��ı������ڵ��кͱ������ͼ�¼��JumpPoint�ṹ���С�
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)
{
  int i = 0;
  if (type == 'L')                              //���Type��L(Left),��ɨ������ߡ�
  {
    for (i = H; i >= L; i--)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //����кڰ�����    1�ǰ� 0�Ǻ�
      {
        Q->point = i;                           //�ǾͰ�����м�¼������Ϊ�����
        Q->type = 'T';                          //���Ұ���һ�е������������䣬�������ͼ�ΪT��������������
        break;                                  //�ҵ��˾�����ѭ��������
      }
      else if (i == (L + 1))                    //Ҫ��ɨ�����û�ҵ�
       {
        if (*(p + (L + H) / 2) != 0)            //����ɨ��������м��ǰ����ص�
        {
          Q->point = (L + H) / 2;               //��ô����Ϊ��һ�е�������Ǵ�����ɨ��������е㡣
          Q->type = 'W';                        //���Ұ���һ�е����Ƿ��������䣬�������ͼ�ΪW�����ޱ��С�
          break;                                //����ѭ��������
        }
        else                                    //Ҫ��ɨ�����û�ҵ�������ɨ��������м��Ǻ����ص�
        {
          Q->point = H;                         //��ô����Ϊ��һ�е�������Ǵ�����ɨ��������������ޡ�
          Q->type = 'H';                        //����Ҳ����һ�е����Ƿ��������䣬�����������ͼ�ΪH�����������С�
          break;                                //����ѭ��������
        }
      }
    }
  }
  else if (type == 'R')                         //���Type��R(Right),��ɨ���ұ��ߡ�
  {
    for (i = L; i <= H; i++)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //����кڰ�����    1�ǰ� 0�Ǻ�
      {
        Q->point = i;                           //�ǾͰ�����м�¼������Ϊ�ұ���
        Q->type = 'T';                          //���Ұ���һ�е������������䣬�������ͼ�ΪT��������������
        break;                                  //�ҵ��˾�����ѭ��������
      }
      else if (i == (H - 1))                    //Ҫ��ɨ�����û�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����ɨ��������м��ǰ����ص�
        {
          Q->point = (L + H) / 2;               //��ô����Ϊ��һ�е��ұ����Ǵ�����ɨ��������е㡣
          Q->type = 'W';                        //���Ұ���һ�е����Ƿ��������䣬�������ͼ�ΪW�����ޱ��С�
          break;
        }
        else                                    //Ҫ��ɨ�����û�ҵ�������ɨ��������м��Ǻ����ص�
        {
          Q->point = L;                         //��ô����Ϊ��һ�е��ұ����Ǵ�����ɨ��������������ޡ�
          Q->type = 'H';                        //����Ҳ����һ�е����Ƿ��������䣬�����������ͼ�ΪH�����������С�
          break;                                //����ѭ��������
        }
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BaseLine
//  @brief          �ñ����ķ����õ�ͼ����������У�59-55�У��ı��ߺ�������Ϣ�������б��ߺ�������Ϣ��׼ȷ�ȷǳ�����Ҫ��ֱ��Ӱ�쵽����ͼ��Ĵ�������
//  @parameter      void
//  @time           2022��12��21��
//  @Author
//  Sample usage:   Get_BaseLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    /**************************************��������ͼ������У�59�У����ұ��ߴӶ�ȷ�����ߵĹ��� ********************************************************************/
    /****************************************************Begin*****************************************************************************/

    PicTemp = Pixle[59];                                                //��PicTemp���ָ�����ָ��ͼ�������Pixle[59]
    for (Xsite = ImageSensorMid; Xsite < 79; Xsite++)                   //����39�������У��������п�ʼһ��һ�е����ұ������ұ���
    {
      if (*(PicTemp + Xsite-1) == 1 &&*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)       //������������������ڵ㣬˵û�ҵ��˱��ߡ�
      {
        BottomBorderRight = Xsite;                                      //����һ�м�¼������Ϊ��һ�е��ұ���
        break;                                                          //����ѭ��
      }
      else if (Xsite == 78)                                             //����ҵ��˵�58�ж���û���ֺڵ㣬˵����һ�еı��������⡣
      {
        BottomBorderRight = 79;                                         //����������Ĵ�����ǣ�ֱ�Ӽ���ͼ�����ұߵ���һ�У���79�У�������һ�е��ұ��ߡ�
        break;                                                          //����ѭ��
      }
    }

    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)                    //����39�������У��������п�ʼһ��һ�е���������������
    {
      if (*(PicTemp + Xsite+1) == 1 && *(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)       //������������������ڵ㣬˵û�ҵ��˱��ߡ�
      {
        BottomBorderLeft = Xsite;                                       //����һ�м�¼������Ϊ��һ�е������
        break;                                                          //����ѭ��
      }
      else if (Xsite == 1)                                              //����ҵ��˵�1�ж���û���ֺڵ㣬˵����һ�еı��������⡣
      {
        BottomBorderLeft = 0;                                           //����������Ĵ�����ǣ�ֱ�Ӽ���ͼ������ߵ���һ�У���0�У�������һ�е�����ߡ�
        break;                                                          //����ѭ��
      }
    }

    BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;           //�������ұ߽�������59�е�����
    ImageDeal[59].LeftBorder = BottomBorderLeft;                        //�ѵ�59�е���߽�洢�����飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].RightBorder = BottomBorderRight;                      //�ѵ�59�е��ұ߽�洢�����飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].Center = BottomCenter;                                //�ѵ�59�е����ߴ洢�����飬    ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;          //�ѵ�59�е�������ȴ洢���飬ע�⿴ImageDeal������ֵ��±꣬�ǲ������ö�Ӧ59��
    ImageDeal[59].IsLeftFind = 'T';                                     //��¼��59�е����������ΪT���������ҵ�����ߡ�
    ImageDeal[59].IsRightFind = 'T';                                    //��¼��59�е��ұ�������ΪT���������ҵ��ұ��ߡ�

    ImageDeal[59].Left_BlackandWhite=BottomBorderLeft;
    ImageDeal[59].Right_BlackandWhite=BottomBorderRight;
    for (Ysite = 58; Ysite > 54; Ysite--)
    {
        PicTemp = Pixle[Ysite];
        for(Xsite = ImageDeal[Ysite + 1].Center; Xsite < 79;Xsite++)
        {
          if(*(PicTemp + Xsite-1) == 1 && *(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
          {
            ImageDeal[Ysite].RightBorder = Xsite;

            ImageDeal[Ysite].Right_BlackandWhite = Xsite;
            break;
          }
          else if (Xsite == 78)
          {
            ImageDeal[Ysite].RightBorder = 79;

            ImageDeal[Ysite].Right_BlackandWhite = 79;
            break;
          }
        }

        for (Xsite = ImageDeal[Ysite + 1].Center; Xsite > 0;Xsite--)
        {
          if (*(PicTemp + Xsite+1) == 1 && *(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
          {
            ImageDeal[Ysite].LeftBorder = Xsite;

            ImageDeal[Ysite].Left_BlackandWhite=Xsite;//

            break;
          }
          else if (Xsite == 1)
          {
            ImageDeal[Ysite].LeftBorder = 0;

            ImageDeal[Ysite].Left_BlackandWhite=0;
            break;
          }
        }

        ImageDeal[Ysite].IsLeftFind  = 'T';
        ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
        ImageDeal[Ysite].Wide   = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_AllLine
//  @brief          ��Get_BaseLine�Ļ����ϣ���Բ����������������һЩ����Ĵ����㷨�õ�ʣ���еı��ߺ�������Ϣ��
//  @parameter      void
//  @time           2022��12��21��
//  @Author
//  Sample usage:   Get_AllLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_AllLine(void)
{
  uint8 L_Found_T  = 'F';    //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_L_line = 'F';    //�ҵ���һ֡ͼ��Ļ�׼��б�ʣ�Ϊʲô����Ҫ��ΪF����������Ĵ����֪���ˡ�
  uint8 R_Found_T  = 'F';    //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_R_line = 'F';    //�ҵ���һ֡ͼ��Ļ�׼��б�ʣ�Ϊʲô����Ҫ��ΪF����������Ĵ����֪���ˡ�
  float D_L = 0;             //������ӳ��ߵ�б��
  float D_R = 0;             //�ұ����ӳ��ߵ�б��
  int ytemp_W_L;             //��ס�״��󶪱���
  int ytemp_W_R;             //��ס�״��Ҷ�����
  int ExtenRFlag = 0;            //��־λ��0
  int ExtenLFlag = 0;            //��־λ��0
  ImageStatus.OFFLine=1;     //����ṹ���Ա��֮���������︳ֵ������Ϊ��ImageStatus�ṹ������ĳ�Ա̫���ˣ�������ʱ��ֻ�õ���OFFLine�������������õ��������ĸ�ֵ��

  //����������W��־λ��0
  ImageStatus.Miss_Left_lines=0;
  ImageStatus.Miss_Right_lines=0;
  ImageStatus.Miss_High_Left_lines=0;
  ImageStatus.Miss_High_Right_lines=0;
  ImageStatus.Miss_Middle_Left_lines=0;
  ImageStatus.Miss_Middle_Right_lines=0;
  ImageStatus.Miss_Low_Left_lines=0;
  ImageStatus.Miss_Low_Right_lines=0;
  ImageStatus.WhiteLine=0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)                            //ǰ5����Get_BaseLine()���Ѿ�������ˣ����ڴ�55�д����Լ��趨�Ĳ�������OFFLine��
  {                                                                                  //��Ϊ̫ǰ���ͼ��ɿ��Բ��㣬����OFFLine�����ú��б�Ҫ��û��Ҫһֱ����ɨ����0�С�
    PicTemp = Pixle[Ysite];                                                          //Ҫ���������
    JumpPointtypedef JumpPoint[2];                                                   // JumpPoint[0]��������ߣ�JumpPoint[1]�����ұ��ߡ�

  /******************************ɨ�豾�е��ұ���******************************/
    IntervalLow =ImageDeal[Ysite + 1].RightBorder  -ImageScanInterval;               //����һ�е��ұ��߼Ӽ�Interval��Ӧ���п�ʼɨ�豾�У�Intervalһ��ȡ5����Ȼ��Ϊ�˱���������԰����ֵ�ĵĴ�һ�㡣
    IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;              //���������ֻ��Ҫ�����б�������5�Ļ����ϣ����10�е�������䣩ȥɨ�ߣ�һ������ҵ����еı����ˣ��������ֵ��ʵ����̫��
    LimitL(IntervalLow);                                                             //������ǶԴ���GetJumpPointFromDet()������ɨ���������һ���޷�������
    LimitH(IntervalHigh);                                                            //������һ�еı����ǵ�2�У�����2-5=-3��-3�ǲ��Ǿ�û��ʵ�������ˣ���ô����-3���أ�
    Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);  //ɨ���õ�һ���Ӻ������Լ�����ȥ�������߼���
  /******************************ɨ�豾�е��ұ���******************************/

  /******************************ɨ�豾�е������******************************/
    IntervalLow =ImageDeal[Ysite + 1].LeftBorder  -ImageScanInterval;                //����һ�е�����߼Ӽ�Interval��Ӧ���п�ʼɨ�豾�У�Intervalһ��ȡ5����Ȼ��Ϊ�˱���������԰����ֵ�ĵĴ�һ�㡣
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;                //���������ֻ��Ҫ�����б�������5�Ļ����ϣ����10�е�������䣩ȥɨ�ߣ�һ������ҵ����еı����ˣ��������ֵ��ʵ����̫��
    LimitL(IntervalLow);                                                             //������ǶԴ���GetJumpPointFromDet()������ɨ���������һ���޷�������
    LimitH(IntervalHigh);                                                            //������һ�еı����ǵ�2�У�����2-5=-3��-3�ǲ��Ǿ�û��ʵ�������ˣ���ô����-3���أ�
    Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);  //ɨ���õ�һ���Ӻ������Լ�����ȥ�������߼���
  /******************************ɨ�豾�е������******************************/
    if (JumpPoint[0].type =='W')                                                     //������е���������ڲ��������䣬����10���㶼�ǰ׵㡣
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                  //��ô���е�����߾Ͳ�����һ�еı��ߡ�
      for(Xsite=ImageDeal[Ysite].LeftBorder;Xsite>=0;Xsite--)
       {
          if(Xsite==1)
          {
              ImageDeal[Ysite].Left_BlackandWhite=0;
          }
          else if(Pixle[Ysite][Xsite]==1&&Pixle[Ysite][Xsite-1]==0)
          {
               ImageDeal[Ysite].Left_BlackandWhite=Xsite;
               break;
          }
       }
    }
    else                                                                             //������е����������T������H���
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                              //��ôɨ�赽�ı����Ƕ��٣��Ҿͼ�¼�����Ƕ��١�
      ImageDeal[Ysite].Left_BlackandWhite=JumpPoint[0].point;
    }

    if (JumpPoint[1].type == 'W')                                                    //������е��ұ������ڲ��������䣬����10���㶼�ǰ׵㡣
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;                //��ô���е��ұ��߾Ͳ�����һ�еı��ߡ�
      for(Xsite=ImageDeal[Ysite].RightBorder;Xsite<=79;Xsite++)
            {
               if(Xsite==78)
               {
                   ImageDeal[Ysite].Right_BlackandWhite=79;
               }
               else if(Pixle[Ysite][Xsite]==1&&Pixle[Ysite][Xsite+1]==0)
               {
                    ImageDeal[Ysite].Right_BlackandWhite=Xsite;
                    break;
               }
            }
    }
    else                                                                             //������е��ұ�������T������H���
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                             //��ôɨ�赽�ı����Ƕ��٣��Ҿͼ�¼�����Ƕ��١�
      ImageDeal[Ysite].Right_BlackandWhite=JumpPoint[1].point;
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                  //��¼�����ҵ�����������ͣ���T����W������H��������ͺ��������õģ���Ϊ�һ�Ҫ��һ������
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;                                //��¼�����ҵ����ұ������ͣ���T����W������H��������ͺ��������õģ���Ϊ�һ�Ҫ��һ������
    if( Ysite>=40 && Ysite<60 )
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                ImageStatus.Miss_High_Right_lines++;
            }
             if (ImageDeal[Ysite].IsLeftFind == 'W' )
            {
                ImageStatus.Miss_High_Left_lines++;
            }

        }
        else if( Ysite>=20 && Ysite<40 )
        {
                if(ImageDeal[Ysite].IsRightFind == 'W' )
                {
                    ImageStatus.Miss_Middle_Right_lines++;
                }
                 if (ImageDeal[Ysite].IsLeftFind == 'W' )
                {
                    ImageStatus.Miss_Middle_Left_lines++;
                }

        }
        else if( Ysite>=0 && Ysite<20 )
        {
                if(ImageDeal[Ysite].IsRightFind == 'W' )
                {
                    ImageStatus.Miss_Low_Right_lines++;
                }
                 if (ImageDeal[Ysite].IsLeftFind == 'W')
                {
                    ImageStatus.Miss_Low_Left_lines++;
                }

        }

    /************************************����ȷ��������(��H��)�ı߽�*************************************/

    if (( ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H'))
    {
      /**************************��������ߵĴ�����***************************/
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);Xsite <= (ImageDeal[Ysite].RightBorder - 1);Xsite++)                                                           //���ұ���֮������ɨ��
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))
          {
            ImageDeal[Ysite].LeftBorder =Xsite;

            ImageDeal[Ysite].Left_BlackandWhite=Xsite;

            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
          {
            ImageDeal[Ysite].LeftBorder = Xsite;
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      }
      /**************************��������ߵĴ�����***************************/


      /**************************�����ұ��ߵĴ�����***************************/
      if (ImageDeal[Ysite].IsRightFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--)
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
          {
            ImageDeal[Ysite].RightBorder =Xsite;

            ImageDeal[Ysite].Right_BlackandWhite=Xsite;

            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
         }
       }
     }
    /**************************�����ұ��ߵĴ�����***************************/

  /*****************************����ȷ��������ı߽�******************************/



 /************************************����ȷ���ޱ��У���W�ࣩ�ı߽�****************************************************************/
    int ysite = 0;
    int Enter_Rings_Process=0;
    int Enter_Three_Forks=0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    /**************************��������ߵ��ޱ���***************************/
    if(flag_left_ring == 0)                                                        //Enter_Rings_Process==0||Enter_Three_Forks==0
    {
        if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite > 10&&Ysite < 54)
        {
          if (Get_R_line == 'F')
          {
            Get_R_line = 'T';
            ytemp_W_R = Ysite + 2;
            for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
            {
              if (ImageDeal[ysite].IsRightFind =='T')
              {
                  R_found_point++;
              }
            }
            if (R_found_point >7)
            {
              D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
              if (D_R > 0)
              {
                R_Found_T ='T';
              }
              else
              {
                R_Found_T = 'F';
                if (D_R < 0)
                {
                    ExtenRFlag = 'F';
                }
              }
            }
          }
          if (R_Found_T == 'T')
          {
            ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //����ҵ��� ��ô�Ի�׼�����ӳ���
          }
          LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
          LimitH(ImageDeal[Ysite].RightBorder);  //�޷�
        }
        /**************************��������ߵ��ޱ���***************************/


        /**************************�����ұ��ߵ��ޱ���***************************/
        if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 54 )
        {
          if (Get_L_line == 'F')
          {
            Get_L_line = 'T';
            ytemp_W_L = Ysite + 2;
            for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
            {
              if (ImageDeal[ysite].IsLeftFind == 'T')
                {
                  L_found_point++;
                }
            }
            if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
            {
              D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
              if (D_L > 0)
              {
                L_Found_T = 'T';
              }
              else
              {
                L_Found_T = 'F';
                if (D_L < 0)
                {
                    ExtenLFlag = 'F';
                }
              }
            }
          }

          if (L_Found_T == 'T')
          {
              ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
          }

          LimitL(ImageDeal[Ysite].LeftBorder);  //�޷�
          LimitH(ImageDeal[Ysite].LeftBorder);  //�޷�
        }
    }

    /**************************�����ұ��ߵ��ޱ���***************************/
    /************************************����ȷ���ޱ��У���W�ࣩ�ı߽�****************************************************************/
    /************************************��������֮��������һЩ������������*************************************************/
    if(Ysite<54 && Ysite>15)
    {
        if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
        {
          ImageStatus.WhiteLine++;  //Ҫ�����Ҷ��ޱߣ�������+1

        }
         if (ImageDeal[Ysite].IsLeftFind == 'W' )
        {

          ImageStatus.Miss_Left_lines++;
        }
         if(ImageDeal[Ysite].IsRightFind == 'W')
        {
             ImageStatus.Miss_Right_lines++;
        }
    }
      LimitL(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

      if (ImageDeal[Ysite].Wide <= 7)
      {
          ImageStatus.OFFLine = Ysite + 1;
          break;
      }
      else if (ImageDeal[Ysite].RightBorder <= 10||ImageDeal[Ysite].LeftBorder >= 70)
      {
          ImageStatus.OFFLine = Ysite + 1;
          break;
      }
      /************************************��������֮��������һЩ������������*************************************************/
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Element_Judgment_Crosses
//  @brief          �ж��Ƿ�Ϊʮ��·�ڣ�����ʮ��·�ڽ�����Ӧ���ߴ���
//  @parameter      void
//  @time           2022��12��27��
//  @Author
//  Sample usage:   Element_Judgment_Crosses;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Element_Judgment_Crosses()
{
    Enter_Crosses_Process = 1;
    if (ImageStatus.WhiteLine < 16)
    {
        Enter_Crosses_Process = 0;
    }
    else {
        TFSite = 55;
    }
    if (ExtenLFlag != 'F')                                                //���ExtenLFlag��־��������F���ǾͿ�ʼ���в��߲�����
        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)        //�ӵ�54��ʼ����ɨ��һֱɨ���������漸�С�
        {
            PicTemp = Pixle[Ysite];
            if (ImageDeal[Ysite].IsLeftFind == 'W')                            //������е������������W���ͣ�Ҳ�����ޱ������͡�
            {
                if (ImageDeal[Ysite + 1].LeftBorder >= 70)                      //�����߽絽�˵�70���ұ�ȥ�ˣ��Ǵ���ʾ��Ǽ��������˵���Ѿ�����ˡ�
                {
                    ImageStatus.OFFLine = Ysite + 1;                              //���������õĴ��������ǲ�����ֱ������ѭ����
                    break;
                }
                while (Ysite >= (ImageStatus.OFFLine + 4))                      //�����߽��������Ǿͽ���whileѭ�����ţ�ֱ������ѭ������������
                {
                    Ysite--;                                                      //��������
                    if (ImageDeal[Ysite].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 1].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 2].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 2].LeftBorder > 0
                        && ImageDeal[Ysite - 2].LeftBorder < 70
                        )                                                         //���ɨ�����ޱ��е������������ж�����������
                    {
                        FTSite = Ysite - 2;                                         //�ǾͰ�ɨ������һ�е��������д���FTsite����
                        break;                                                      //����whileѭ��
                    }
                }

                DetL = ((float)(ImageDeal[FTSite].LeftBorder - ImageDeal[TFSite].LeftBorder)) / ((float)(FTSite - TFSite));  //��������ߵĲ���б��
                if (FTSite > ImageStatus.OFFLine)                              //���FTSite�洢����һ����ͼ�񶥱�OFFline������
                    for (ytemp = TFSite; ytemp >= FTSite; ytemp--)               //��ô�Ҿʹӵ�һ��ɨ������߽������ڶ��е�λ�ÿ�ʼ����һֱ���ߣ�����FTSite�С�
                    {
                        ImageDeal[ytemp].LeftBorder = (int)(DetL * ((float)(ytemp - TFSite))) + ImageDeal[TFSite].LeftBorder;     //������Ǿ���Ĳ��߲�����
                    }
            }
            else                                                              //ע�⿴������else���ĸ�if��һ�ԣ�������߼���ϵ��
                TFSite = Ysite + 2;                                             //����ΪʲôҪYsite+2����û����ע�����潲������Լ����ɡ�
        }
    /************************************����ߵĲ��ߴ���*************************************************/


    /************************************�ұ��ߵĲ��ߴ���������ߴ���˼·һģһ����ע����*************************************************/
    if (ImageStatus.WhiteLine >= 16)
        TFSite = 55;
    if (ExtenRFlag != 'F')
        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
        {
            PicTemp = Pixle[Ysite];  //�浱ǰ��
            if (ImageDeal[Ysite].IsRightFind == 'W')
            {
                if (ImageDeal[Ysite + 1].RightBorder <= 10)
                {
                    ImageStatus.OFFLine = Ysite + 1;
                    break;
                }
                while (Ysite >= (ImageStatus.OFFLine + 4))
                {
                    Ysite--;
                    if (ImageDeal[Ysite].IsRightFind == 'T'
                        && ImageDeal[Ysite - 1].IsRightFind == 'T'
                        && ImageDeal[Ysite - 2].IsRightFind == 'T'
                        && ImageDeal[Ysite - 2].RightBorder < 79
                        && ImageDeal[Ysite - 2].RightBorder > 10
                        )
                    {
                        FTSite = Ysite - 2;
                        break;
                    }
                }

                if(Ysite < (ImageStatus.OFFLine + 4))               //˵������ѭ�����˶�û�н���whileѭ����if���,��FTSiteû�и���
                {
                    int i = 0;
                    int j=0;
                    for(i = Ysite;i <= (Ysite+10);i++)
                    {
                        for(j=79;j >= 43;j--)
                        {
                            //�����ұ���ֵ
                            if(Pixle[i][j] == 0 && Pixle[i][j-1] == 0 && Pixle[i][j-2] == 1 && Pixle[i][j-3] == 1)  //�ҵ����������ɺڵ��׵�ת��
                            {
                                ImageDeal[i].RightBorder = j-1;
                                ImageDeal[i].IsRightFind = 'T';
                                break;
                            }
                        }
                    }
                    if (ExtenRFlag != 'F')
                        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
                        {
                            PicTemp = Pixle[Ysite];  //�浱ǰ��
                            if (ImageDeal[Ysite].IsRightFind == 'W')
                            {
                                if (ImageDeal[Ysite + 1].RightBorder <= 10)
                                {
                                    ImageStatus.OFFLine = Ysite + 1;
                                    break;
                                }
                                while (Ysite >= (ImageStatus.OFFLine + 4))
                                {
                                    Ysite--;
                                    if (ImageDeal[Ysite].IsRightFind == 'T'
                                        && ImageDeal[Ysite - 1].IsRightFind == 'T'
                                        && ImageDeal[Ysite - 2].IsRightFind == 'T'
                                        && ImageDeal[Ysite - 2].RightBorder < 79
                                        && ImageDeal[Ysite - 2].RightBorder > 10
                                        )
                                    {
                                        FTSite = Ysite - 2;
                                        break;
                                    }
                                }
                            }
                        }
                }

                DetR = ((float)(ImageDeal[FTSite].RightBorder - ImageDeal[TFSite].RightBorder)) / ((float)(FTSite - TFSite));
                if (FTSite > ImageStatus.OFFLine)
                    for (ytemp = TFSite; ytemp >= FTSite; ytemp--)
                    {
                        ImageDeal[ytemp].RightBorder = (int)(DetR * ((float)(ytemp - TFSite))) + ImageDeal[TFSite].RightBorder;
                    }
            }
            else
                TFSite = Ysite + 2;
        }
    /************************************�ұ��ߵĲ��ߴ���������ߴ���˼·һģһ����ע����*************************************************/


 /*****************************************���еĲ��ߴ�������������ߺ����������Ϣ�ٸ���һ��******************************************************/
    for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        ImageDeal[Ysite].Wide = -ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder;
    }
    /*****************************************���еĲ��ߴ�������������ߺ����������Ϣ�ٸ���һ��******************************************************/
}
//--------------------------------------------------------------
//  @name           Garage_Identification()
//  @brief          ����ͼ����ʶ�������ߺͳ���
//  @parameter      void
//  @time           2023��3��6��
//  @Author         MRCHEN
//  Sample usage:   Garage_Identification();
//------------------------------------------------------
void Garage_Identification()
{
    int Zebra_X1 = 0, Zebra_Y1 = 0, Zebra_X2 = 0, Zebra_Y2 = 0, wid = 0;
    int Ysite_Flag[15] = { 0 }, temp = 0, Zebra_num = 0, Zebra_len = 0;
    int dir = 0;
    /*25-10�У�10-70���Ѱ�����*/
    /*if(Car_Barn_num == 2)
        flag_stop_Car = 1;*/
    if (Zebra_Cnt == 0)
    {
        Zebra_num = 0;
        for (Ysite = 25; Ysite >= 15; Ysite--)
        {
            for (Xsite = 15; Xsite <= 65; Xsite++)
            {
                if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                {
                    Zebra_num++;//�ڰ������+1
                }
            }
        }
        if (Zebra_num >= 14)
        {
            Zebra_Cnt = 1;
            Car_Barn_num++;
            //break;
        }
    }
    //�����ߴ���״̬
    if (Zebra_Cnt == 1)
    {
        if (Zebra_Flag == 0)//�ж�55-59���Ƿ���ڰ����� ������ڼ�¼�˿�״̬
        {
            Zebra_len = 0;
            Zebra_num = 0;
            for (Ysite = 59; Ysite >= 55; Ysite--)
            {
                for (Xsite = 10; Xsite <= 70; Xsite++)
                {
                    if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                    {
                        Zebra_num++;//�ڰ������+1
                    }
                    if (Zebra_num >= 4)
                    {
                        Zebra_len++;//�����߳���+1
                        Zebra_num = 0;
                        break;//������һ���ж�
                    }
                }
                if (Zebra_len >= 2)
                {
                    Zebra_Flag = 1;
                    break;
                }
            }

        }
        if (Zebra_Flag == 1)//�ж�55-59�а����߲����� ��־��ǰ�Ѿ���ȥ����
        {
            Zebra_len = 0;
            Zebra_num = 0;
            for (Ysite = 59; Ysite >= 55; Ysite--)
            {
                for (Xsite = 10; Xsite <= 70; Xsite++)
                {
                    if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                    {
                        Zebra_num++;//�ڰ������+1
                    }
                    if (Zebra_num >= 4)
                    {
                        Zebra_len++;//�����߳���+1
                        Zebra_num = 0;
                        break;//������һ���ж�
                    }
                }
            }
            if (Zebra_len == 0)  //��ʱ�޺ڰ�����
            {
                Zebra_Flag = 2;
            }
        }
        if (Zebra_Flag == 2)//��־λ����
        {
            Zebra_Flag = 0;
            Zebra_Cnt = 0;
            //flag_stop_Car = 1;
        }
    }
    /*ȷ��������y��λ��*/
    /*��ʴ������*/
    if (Zebra_Cnt == 1)
    {
        for (Xsite = 35; Xsite <= 45; Xsite++)
        {
            for (Ysite = 59; Ysite >= 15; Ysite--)
            {
                if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite - 1][Xsite] == 0)
                {
                    Ysite_Flag[Xsite - 35] = Ysite;
                    break;
                }
            }
        }
        temp = Ysite_Flag[0];
        for (int i = 0; i <= 10; i++)
        {
            if (temp < Ysite_Flag[i + 1])
            {
                temp = Ysite_Flag[i + 1];//�������¶˰�����
            }
        }
        Y = temp;
        if (temp + 2 <= 59)
        {
            dir = temp + 2;
        }
        else
        {
            dir = 59;
        }

        for (Ysite = dir; Ysite >= dir - 20; Ysite--)
        {
            Zebra_X1 = 0;
            Zebra_X2 = 0;
            for (Xsite = 0; Xsite < 79; Xsite++)
            {
                if (Pixle[Ysite][Xsite - 1] == 1 && Pixle[Ysite][Xsite] == 0)
                {
                    Zebra_X1 = Xsite;
                    while (Xsite++)
                    {
                        if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
                        {
                            Zebra_X2 = Xsite;
                            wid = Zebra_X2 - Zebra_X1 + 1;
                            if (wid <= 4)
                            {
                                //X1 = Zebra_X1;
                                //X2 = Zebra_X2;
                                for (int i = Zebra_X1; i <= Zebra_X2; i++)
                                {
                                    Pixle[Ysite][i] = 1;
                                    Pixle_hb[Ysite][i] = 255;
                                }
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}



//����Ͱ����ߵ�ʶ����
//ʶ���̶����
void find_zebra()
{
    uint8 black_white_num = 0;                  //�ڰ����������
    int i =0;
    int j =0;

    for(i= 43;i <=53 ;i++)                          //�б���
    {
        for(j=10 ;j <70 ;j++)                      //�б���
        {
            if((Pixle[i][j] == 0 && Pixle[i][j+1] == 1) || (Pixle[i][j] == 1 && Pixle[i][j+1] == 0))
            {
                black_white_num++;
            }
        }
    }

    if(black_white_num >= 64 && flag_out == 0 && flag_left_ring == 0 && flag_right_ring == 0)             //С���ڶ�·�к���Ҳ��ʶ��Ϊ������
    {
        flag_zebra = 1;
    }
		else
		{
			flag_zebra = 0;
		}
//		ips200_show_int(100, 300, flag_zebra , 2);
}

//С��������ͣ��
void stop_car()
{
    int num=0;
    for(Ysite=59;Ysite>=55;Ysite--)
     {
         for(Xsite=20;Xsite<60;Xsite++)
         {
             if(Pixle_hb[Ysite][Xsite] == 0)
             {
                 num++;
             }

         }
     }
//     if(((num>210) && flag_openRoad == 0 && flag_BiZhang == 0 && flag_start_camera == 1 && flag_zebra == 0 && flag_po == 0 && Enter_Rings_Process == 0 ))      //����С���ݵĳ���
     if (num > 160)  //145
		 {
         flag_out = 1;
     }
		 else
		 {
			 flag_out = 0;
		 }
//	 ips200_show_int(0, 264, num , 5);
//	 ips200_show_int(0, 280, flag_out , 2);
}


//����Ԫ��ʶ����Ҫ�������ֱ��
//ֱ�����жϣ����������ҵ���һ�γ����ȡ�W���١�T������һ�У���ͼ��������һ�ζ��ߵĵط�����-3��-6ȡ�����㣬�����ֱ�߷��̣�Ȼ��������ϳ����б������ݣ���ԭ���ı������ݽ��бȽϣ�������ĳ��ֵ�����Ϊֱ�߲�����ϸñ��ߣ���¼��λ�ã�����һ����ϲ��ֵĳ���
//              ���ұ��߾����˷����������ȡ���������Ǹ����ֱ�߶γ��ȣ�������ĳ��ֵ��������Ϊ��ֱ�߶Σ�����Ȼ�����ֱ�߱���Ҫ��һ��б�ʣ���������Ǵ�����ɵ����ض���
void road_type()
{
    /**********************ֱ����ʶ��*************************/
    /****************************�ұ���ȡ****************************/
    for(y_right_road_tempt = 55;y_right_road_tempt > ImageStatus.OFFLine;y_right_road_tempt--)
    {
        if(ImageDeal[y_right_road_tempt].IsRightFind == 'W' && ImageDeal[y_right_road_tempt-1].IsRightFind == 'T')
        {
            break;
        }
    }

    a2 = (float)(ImageDeal[y_right_road_tempt-3].RightBorder - ImageDeal[y_right_road_tempt-6].RightBorder)/3.0;
    BB2 = ImageDeal[y_right_road_tempt-3].RightBorder - a2 * (y_right_road_tempt-62);

    int i= 0;
    for(i = y_right_road_tempt-2;i > ImageStatus.OFFLine;i--)
    {
        if(((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) > 3 || ((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) < -3)
        {
            break;
        }
    }
    y_right_length = y_right_road_tempt - i-1;

    /****************************�����ȡ****************************/
    for(y_left_road_tempt= 54;y_left_road_tempt > ImageStatus.OFFLine;y_left_road_tempt--)
    {
        if(ImageDeal[y_left_road_tempt].IsLeftFind == 'W' && ImageDeal[y_left_road_tempt -1].IsLeftFind == 'T')
        {
            break;
        }
    }

    a1 = (float)(ImageDeal[y_left_road_tempt-3].LeftBorder - ImageDeal[y_left_road_tempt-6].LeftBorder)/3.0;
    BB1 = ImageDeal[y_left_road_tempt-3].LeftBorder - a1 * (y_left_road_tempt-62);

    int j= 0;
    for(j = y_left_road_tempt-2;j > ImageStatus.OFFLine;j--)
    {
        if(((a1*(j-59) + BB1) - ImageDeal[j].LeftBorder) > 3 || ((a1*(j-59) + BB1) - ImageDeal[j].LeftBorder) < -3)
        {
            break;
        }
    }
    y_left_length = y_left_road_tempt - j-1;

    if(y_right_length > y_left_length)                                                  //ѡȡ����ֱ�������ֵ
    {
        Str_len = y_right_length;
    }
    else
        Str_len = y_left_length;

    if(Str_len >= 30  && a1 < -0.5 && a2 > 0.5)                 //ֱ�����ж�
    {
        flag_Straight = 1;
    }
    else
        flag_Straight = 0;
//    ips200_show_int(50, 284, flag_Straight, 3);
}

//����Ԫ��ʶ����Ҫ�������ֱ��
//ֱ�����жϣ����������ҵ���һ�γ����ȡ�W���١�T������һ�У���ͼ��������һ�ζ��ߵĵط�����-3��-6ȡ�����㣬�����ֱ�߷��̣�Ȼ��������ϳ����б������ݣ���ԭ���ı������ݽ��бȽϣ�������ĳ��ֵ�����Ϊֱ�߲�����ϸñ��ߣ���¼��λ�ã�����һ����ϲ��ֵĳ���
//              ���ұ��߾����˷����������ȡ���������Ǹ����ֱ�߶γ��ȣ�������ĳ��ֵ��������Ϊ��ֱ�߶Σ�����Ȼ�����ֱ�߱���Ҫ��һ��б�ʣ���������Ǵ�����ɵ����ض���
void road_type_2()
{
    /**********************ֱ����ʶ��*************************/
    /****************************�ұ���ȡ****************************/
    // for(y_right_road_tempt = 55;y_right_road_tempt > ImageStatus.OFFLine;y_right_road_tempt--)
    for(y_right_road_tempt = ImageStatus.OFFLine+5 ; y_right_road_tempt < 55 ; y_right_road_tempt++)
    {
        if(ImageDeal[y_right_road_tempt].IsRightFind == 'W' && ImageDeal[y_right_road_tempt-1].IsRightFind == 'T')
        {
            break; //�޸ģ����������ң��Է���û�ж��ߵ�����»�һֱ�ҵ�ͼ�񶥲������һֱû�ҵ������߿�ѭ���·������ͬ��
        }
    }

    a2 = (float)(ImageDeal[y_right_road_tempt-3].RightBorder - ImageDeal[y_right_road_tempt-6].RightBorder)/3.0;
    BB2 = ImageDeal[y_right_road_tempt-3].RightBorder - a2 * (y_right_road_tempt-62);

    int i= 0;
    for(i = y_right_road_tempt-2;i > ImageStatus.OFFLine;i--)
    {
        if(((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) > 3 || ((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) < -3)
        {
            break;
        }
    }
    y_right_length = y_right_road_tempt - i-1;

    /****************************�����ȡ****************************/
    // for(y_left_road_tempt= 54;y_left_road_tempt > ImageStatus.OFFLine;y_left_road_tempt--)
    for(y_left_road_tempt = ImageStatus.OFFLine+5;y_left_road_tempt < 55;y_left_road_tempt++)
    {
        if(ImageDeal[y_left_road_tempt].IsLeftFind == 'W' && ImageDeal[y_left_road_tempt -1].IsLeftFind == 'T')
        {
            break;
        }
    }

    a1 = (float)(ImageDeal[y_left_road_tempt-3].LeftBorder - ImageDeal[y_left_road_tempt-6].LeftBorder)/3.0;
    BB1 = ImageDeal[y_left_road_tempt-3].LeftBorder - a1 * (y_left_road_tempt-62);

    int j= 0;
    for(j = y_left_road_tempt-2;j > ImageStatus.OFFLine;j--)
    {
        if(((a1*(j-59) + BB1) - ImageDeal[j].LeftBorder) > 3 || ((a1*(j-59) + BB1) - ImageDeal[j].LeftBorder) < -3)
        {
            break;
        }
    }
    y_left_length = y_left_road_tempt - j-1;

    if(y_right_length > y_left_length)                         //ѡȡ����ֱ�������ֵ
    {
        Str_len = y_right_length;
    }
    else
        Str_len = y_left_length;

    if(Str_len >= 30  /*&& a1 < -0.5 && a2 > 0.5*/)        //�޸ģ�ֱ�����жϣ��������޸���ֵ�����������������̣�����Խ�30��С���һ�������б���ж�ע���ˣ���Ϊ������ʱ������������ܿ���ֱ����б�ʻ�ܴ�
    {
        flag_Straight = 1;
    }
    else
        flag_Straight = 0;
		ips200_show_int(50, 284, flag_Straight, 3);

}

//�󻷵���ʶ��
void find_left_ring()
{
    int i,j;
    int r_st_up,r_st_dw;
    int st_cha,st_num; // �ж�ֱ��
    int w_num; // �󶪱߼���
    int lower_point; //����µĹյ�
    int longer_num; // ���������
    if(Enter_Crosses_Process == 0) // ������ʮ��·�ڣ����ȼ�Ҫ��ʮ��·�ڵ�
    {
        if(flag_left_ring == 0)
        {
            r_st_dw = ImageDeal[54].RightBorder;
            r_st_up = ImageDeal[50].RightBorder;
            int k = (r_st_up - r_st_dw)/(50-54)
            for(i=55;i>=ImageStatus.OFFLine;i--) // Ѱ���������ж��Ƿ��ǻ���
            {
                // �ȿ��ұ��Ƿ���ֱ��
                st_cha = ImageDeal[i].RightBorer - (k*(i-54)+r_st_dw)
                if(st_cha < 3 && st_cha > -3)
                {
                    st_num++;
                }
                else{
                    break;
                }
            }
            for(i=55;i>=ImageStatus.OFFLine;i--) // ��߶�������
            {
                if(ImageDeal[i].IsLeftFind == 'W')
                {
                    w_num++;
                }
            }

            if(st_num > 30 && w_num > 4)
            {
                flag_left_ring = 1;
            }
        }
        if(flag_left_ring == 1) // �ҵ�����������һ���׶�
        {
            for(i=55;i>=ImageStatus.OFFLine+1;i--) // ��¼������Ĺյ�
            {
                if(ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
                {
                    lower_point = i;
                    break;
                }
            }
            if(lower_point>40 && lower_point<55)
            {
                flag_left_ring == 2;
            }
        }
        if(flag_left_ring == 2) // С����omu�����赥λ���ţ�����״�뻷
        {
            // ��ؿ���С����ת,Ȼ��Ѱ���
            
            for(i=55;i>=ImageStatus.OFFLine;i--)
            {
                if((ImageDeal[i].IsLeftFind=='W' && ImageDeal[i].IsRightFind=='W')||(ImageDeal[i].RightBorer - ImageDeal[i].Left > 60))
                {
                    longer_num ++;
                }
            }
            if(longer_num > 5)
            {
                flag_left_ring == 3
            }
        }
        if(flag_left_ring == 3) // ������
        {
            // ��ؿ���С������������ת����
            // ���дС�������������������Ǽ�¼�������תһ���̶ȣ�ʹflag_left_ring == 0
        }
    }
}

//�һ���
void find_right_ring()
{
}

//ʮ�ֵ�ʶ��ʹ���
//��Ҫ���������߶���һ���ܴ�Ŀհ������������T-W-T�����
float Angle_Sum_y = 0.0;              //y��Ƕ�ֵ�ۼӣ��ж��µ�
void find_cross()
{
    //ʮ�ֵ�ʶ��һ���ᾭ�����Ҷ���һ�οհ����Ľ׶Σ������Ҿ��������յ�
    //ʶ���������߸��������յ�λ��
    int i =0;
    if(flag_cross == 0)
    {
    for(i = 53;i>=ImageStatus.OFFLine;i--)
    {
        //���
        if(flag_left_Ysite_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            flag_left_Ysite_1 = 1;
            left_Ysite_1 = i;
        }
        else if(flag_left_Ysite_1 == 1 && flag_left_Ysite_2 == 0 && ImageDeal[i].IsLeftFind =='T' && ImageDeal[i+1].IsLeftFind == 'W')
        {
            flag_left_Ysite_2 = 1;
            left_Ysite_2 = i;
        }
        //�ұ�
        if(flag_right_Ysite_1 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            flag_right_Ysite_1 = 1;
            right_Ysite_1 = i;
        }
        else if(flag_right_Ysite_1 == 1 && flag_right_Ysite_2 == 0 && ImageDeal[i].IsRightFind =='T' && ImageDeal[i+1].IsRightFind == 'W')
        {
            flag_right_Ysite_2 = 1;
            right_Ysite_2 = i;
        }
    }

    if(flag_left_Ysite_1 == 1 && flag_left_Ysite_2 == 1 && flag_right_Ysite_1 == 1 && flag_right_Ysite_2 == 1 && ImageStatus.WhiteLine >=8)
    {
        flag_cross =1;

        flag_left_Ysite_1 = 0;
        flag_left_Ysite_2 = 0;
        flag_right_Ysite_1 = 0;
        flag_right_Ysite_2 = 0;
    }
    }

    //���Ѿ�ʶ��ʮ�ֵ�����½��в��ߴ���
    else if(flag_cross == 1)
    {
        //��Ҫ���ж������м����յ㣬������������������һ��
        for(i = 53;i>=ImageStatus.OFFLine;i--)
        {
            //���
            if(flag_left_Ysite_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
            {
                flag_left_Ysite_1 = 1;
                left_Ysite_1_tempt = i;
            }
            else if(flag_left_Ysite_1 == 1 && flag_left_Ysite_2 == 0 && ImageDeal[i].IsLeftFind =='T' && ImageDeal[i+1].IsLeftFind == 'W')
            {
                flag_left_Ysite_2 = 1;
                left_Ysite_2_tempt = i;
            }
            //�ұ�
            if(flag_right_Ysite_1 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
            {
                flag_right_Ysite_1 = 1;
                right_Ysite_1_tempt = i;
            }
            else if(flag_right_Ysite_1 == 1 && flag_right_Ysite_2 == 0 && ImageDeal[i].IsRightFind =='T' && ImageDeal[i+1].IsRightFind == 'W')
            {
                flag_right_Ysite_2 = 1;
                right_Ysite_2_tempt = i;
            }
        }

        if(left_Ysite_1_tempt >= left_Ysite_1 && left_Ysite_2_tempt>=left_Ysite_2 && flag_left_Ysite_1 == 1 && flag_left_Ysite_2 == 1)              //˵������������յ�
        {
            //����
            a1 = (float)(ImageDeal[left_Ysite_1_tempt].LeftBorder - ImageDeal[left_Ysite_2_tempt].LeftBorder)/(left_Ysite_1_tempt - left_Ysite_2_tempt);
            BB1 = ImageDeal[left_Ysite_1_tempt].LeftBorder - a1*(left_Ysite_1_tempt - 59);
            for(int j = left_Ysite_1_tempt ;j >= left_Ysite_2_tempt;j--)
            {
                ImageDeal[j].LeftBorder = a1*(j-59) + BB1;
            }
        }
        else        //���ֻ��һ���յ�
        {
            //����������յ�
            for(i = 53;i>=ImageStatus.OFFLine;i--)
            {
                if(flag_left_Ysite_2 == 0 && ImageDeal[i].IsLeftFind =='T' && ImageDeal[i+1].IsLeftFind == 'W')
                {
                    flag_left_Ysite_2 = 1;
                    left_Ysite_2_tempt = i;
                    break;
                }
                //����
                a1 = (float)(ImageDeal[59].LeftBorder - ImageDeal[left_Ysite_2_tempt].LeftBorder)/(59 - left_Ysite_2_tempt);
                BB1 = ImageDeal[59].LeftBorder;
                for(int j = 59 ;j >= left_Ysite_2_tempt;j--)
                {
                    ImageDeal[j].LeftBorder = a1*(j-59) + BB1;
                }
            }
        }

        //�ұ߲���
        if(right_Ysite_1_tempt >= right_Ysite_1 && right_Ysite_2_tempt>=right_Ysite_2 && flag_right_Ysite_1 == 1 && flag_right_Ysite_2 == 1)              //˵���ұ��������յ�
        {
            //����
            a2 = (float)(ImageDeal[right_Ysite_1_tempt].RightBorder - ImageDeal[right_Ysite_2_tempt].RightBorder)/(right_Ysite_1_tempt - right_Ysite_2_tempt);
            BB2 = ImageDeal[right_Ysite_1_tempt].RightBorder - a2*(right_Ysite_1_tempt - 59);
            for(int j = right_Ysite_1_tempt ;j >= right_Ysite_2_tempt;j--)
            {
                ImageDeal[j].RightBorder = a2*(j-59) + BB2;
            }
        }
        else        //�ұ�ֻ��һ���յ�
        {
            //����������յ�
            for(i = 53;i>=ImageStatus.OFFLine;i--)
            {
                if(flag_right_Ysite_2 == 0 && ImageDeal[i].RightBorder =='T' && ImageDeal[i+1].RightBorder == 'W')
                {
                    flag_right_Ysite_2 = 1;
                    right_Ysite_2_tempt = i;
                    break;
                }
                //����
                a2 = (float)(ImageDeal[59].RightBorder - ImageDeal[right_Ysite_2_tempt].RightBorder)/(59 - right_Ysite_2_tempt);
                BB2 = ImageDeal[59].RightBorder;
                for(int j = 59 ;j >= right_Ysite_2_tempt;j--)
                {
                    ImageDeal[j].RightBorder = a2*(j-59) + BB2;
                }
            }
        }

        //���������Ҫ��������ֵ
        for(int d = 59;d>=0;d--)
        {
            ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
        }
    }

    if(ImageStatus.WhiteLine <8)
    {
        flag_cross = 0;
        
    }
    flag_crossring = 1-flag_cross;
    //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_Ysite_1 = 0;
    flag_left_Ysite_2 = 0;
    flag_right_Ysite_1 = 0;
    flag_right_Ysite_2 = 0;
}



//ת�䶪�߲�������
void bu()
{
    int i = 0;
    for(i = 59;i>=ImageStatus.OFFLine;i--)
    {
        if(flag_1_left == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i+1].IsLeftFind != 'W')
        {
            flag_1_left = 1;
            point_1_left = i;
        }
        if(ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind != 'W' && flag_1_left == 1)
        {
            flag_1_left = 2;
            point_2_left = i;
        }
        if(flag_1_right == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i+1].IsRightFind != 'W')
        {
            flag_1_right = 1;
            point_1_right = i;
        }
        if(flag_1_right == 1 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind != 'W')
        {
            flag_1_right = 2;
            point_2_right = i;
        }
    }

    if(point_1_left - point_2_left > 30)            //˵������д������ޱ���W�������ߣ���߲�ֵ�Ǽ�С
    {
        for(int j =point_1_left;j >= point_2_left;j--)
        {
            ImageDeal[j].LeftBorder = -8;
        }
    }

    if(point_1_right - point_2_right > 30)            //˵���ұ��д������ޱ���W�������ߣ��ұ߲�ֵ������
    {
        for(int j =point_1_right;j >= point_2_right;j--)
        {
            ImageDeal[j].RightBorder = 88;
        }
    }

    flag_1_left = 0;
    flag_1_right = 0;
}

//�����߸�ʴ����
void fu_shi()
{
    if(flag_zebra == 1)
    {
        int i =0;
        for(i= 30;i <=59 ;i++)                  //��ʶ�𵽰����ߣ��Ұ���Щ�е�����ȫŪΪ��������
        {
            ImageDeal[i].Center = 39;
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Slope_cal
//  @brief          ����б�ʣ����ڽ���ת��֮�����
//  @parameter      void
//  @time           2024��2��24��
//  @Author					zzj
//  Sample usage:   angle=Slope_cal()
//----------------------------------------------------------------------------------------------------------------------------------------------
float Slope_cal(void)
{
		float slope=0;
		float A1=0;
		float A2=0;
		float A3=0;
		float B=0;
		float tem=0;
			for(int i =59;i>=40;i--)
		{
			A1=ImageDeal[i].Center+A1;
		}
			for(int i =39;i>=20;i--)
		{
			A2=ImageDeal[i].Center+A2;
		}
			for(int i =19;i>=0;i--)
		{
			A3=ImageDeal[i].Center+A3;
		}
		A1=A1/20;
		A2=A2/20;
		A3=A3/20;
		B=A1;
		tem=atan((B-40)/30);
		slope=tem*(180/3.14);//���ƽǶȽ�������53.13��
//		if (left_ring_process == 3)
//			slope = -10;
//		if (right_ring_process == 3)
//		  slope = 10;
		return slope;
}


float Slope_cal_2(void)
{
		float slope=0;
		float A1=0;
		float A2=0;
		float A3=0;
		float B=0;
		float tem=0;
		for(int i =59;i>=40;i--)
		{
			A1=ImageDeal[i].Center+A1;
		}
			for(int i =39;i>=20;i--)
		{
			A2=ImageDeal[i].Center+A2;
		}
			for(int i =19;i>=0;i--)
		{
			A3=ImageDeal[i].Center+A3;
		}
		A1=A1/20;
		A2=A2/20;
		A3=A3/20;
		B=A1*0.5+A2*0.3+A3*0.2;
		tem=atan((B-40)/30);
		slope=tem*(180/3.14);//���ƽǶȽ�������53.13��
		if (flag_left_ring == 1)
		{
		if (left_ring_process == 3)
			slope = -13;
		if (left_ring_process == 4 && slope > 0)
			slope = 0;
	  }
		if (flag_right_ring == 1)
		{
		if (right_ring_process == 3)
		  slope = 13;
		if (right_ring_process == 4 && slope < 0)
			slope = 0;
	  }
		return slope;
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Center_line
//  @name           Left_right_line
//  @brief          ����Ļ����ʾ���ߺ����ұ��ߣ�����ע�͵�
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void Center_line()
{
        static unsigned char i=0;
        for(i=59;i>=1;i--)
        {
             if(ImageDeal[i].Center<1||ImageDeal[i].Center>79)
                continue;
             ips200_draw_point(160+ImageDeal[i].Center,260+i,0xF800);
        }
}
void Left_right_line()
{
     static unsigned char i=0;
        for(i=59;i>=1;i--)
        {
              if(ImageDeal[i].LeftBorder<1||ImageDeal[i].LeftBorder>79)
                continue;
            if(ImageDeal[i].RightBorder<1||ImageDeal[i].RightBorder>79)
                continue;
           ips200_draw_point(160+ImageDeal[i].LeftBorder,260+i,0xF800);
           ips200_draw_point(160+ImageDeal[i].RightBorder,260+i,0xF800);
        }
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           ringfig
//  @brief          �ж��Ƿ����
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void ringfig()
{
		if(flag_crossring==1 ||Enter_Rings_Flag>0)
			is_roundabout=1;
		else
			is_roundabout=0;

}

//�󻷵���ʶ��
//�ұ���Ϊֱ������߳��֡�ȱ��-һ���-ȱ��-��������-�������ߡ������
void find_left_ring_2()
{
    int i= 0;
	  int left11 = 0,left12 = 0,left13 = 0,left14 = 0;
	  int biao11 = 0,biao12 = 0,biao13 = 0,biao14 = 0;
	  int left21 = 0,left22 = 0,left23 = 0,left24 = 0;
	  int biao21 = 0,biao22 = 0,biao23 = 0,biao24 = 0;
	  int num = 0,num1 = 0,num2 =0;
	  int biaoshang = 0,biaonow = 0;
    if(flag_left_ring == 0)
    {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //�󻷵��������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
            left14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao24 = 1;
				left24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao23 = 1;
				left23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao22 = 1;
				left22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao21 = 1;
        left21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_left_ring_point_1 = biao11;
			flag_left_ring_point_2 = biao12;
			flag_left_ring_point_3 = biao13;
			flag_left_ring_point_4 = biao14;
			left_ring_point_1_tempt = left11;
			left_ring_point_2_tempt = left12;
			left_ring_point_3_tempt = left13;
			left_ring_point_4_tempt = left14;
			num = num1;
		}
		else 
		{
			flag_left_ring_point_1 = biao21;
			flag_left_ring_point_2 = biao22;
			flag_left_ring_point_3 = biao23;
			flag_left_ring_point_4 = biao24;
			left_ring_point_1_tempt = left21;
			left_ring_point_2_tempt = left22;
			left_ring_point_3_tempt = left23;
			left_ring_point_4_tempt = left24;
			num = num2;
		}

    //�󻷵����жϻ���Ҫ���ұ���Ϊֱ�����������һ������Ԫ��ʶ��Ĳ������ж��ұ����Ƿ�Ϊ��ֱ��
    for(i=53;i>=ImageStatus.OFFLine;i--)
    {
        if(ImageDeal[i].IsRightFind == 'W' && ImageDeal[i -1].IsRightFind == 'T')
        {
            y_right_road_tempt = i;
            break;
        }
    }
    a2 = (ImageDeal[y_right_road_tempt-4].RightBorder - ImageDeal[y_right_road_tempt-7].RightBorder)/3.0;
    BB2 = ImageDeal[y_right_road_tempt-4].RightBorder - a2 * (y_right_road_tempt-63);
    
    for(i = y_right_road_tempt-2;i > ImageStatus.OFFLine;i--)
    {
//			ips200_show_int(0, 216, (a2*(i-59) + BB2) - ImageDeal[i].RightBorder, 10);
        if(((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) > 3 || ((a2*(i-59) + BB2) - ImageDeal[i].RightBorder) < -3)
        {
            break;
        }
    }
     y_right_length = y_right_road_tempt - i;

    //�ж��Ƿ�Ϊ�󻷵�,����ҵ��ĸ��յ㣬�ұ���Ϊֱ��
    if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 1 && y_right_length >= 25 && Enter_Crosses_Process == 0)
    {
        flag_left_ring = 1;
        left_ring_process = 1;              //�����󻷵�����1����������ҵ�4���յ�
    }
   /* else
        flag_left_ring = 0;*/

    flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
		left_ring_point_1_tempt = 0;
		left_ring_point_2_tempt = 0;
		left_ring_point_3_tempt = 0;
		left_ring_point_4_tempt = 0;
		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
    }

    //�󻷵��Ĳ��ߴ���
    if(flag_left_ring == 1)
    {
			zhuan++;
        if(left_ring_process == 1)                      //�󻷵�����1����������ҵ�4���յ�
        {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //�󻷵��������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
            left14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao24 = 1;
				left24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao23 = 1;
				left23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao22 = 1;
				left22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao21 = 1;
        left21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_left_ring_point_1 = biao11;
			flag_left_ring_point_2 = biao12;
			flag_left_ring_point_3 = biao13;
			flag_left_ring_point_4 = biao14;
			left_ring_point_1_tempt = left11;
			left_ring_point_2_tempt = left12;
			left_ring_point_3_tempt = left13;
			left_ring_point_4_tempt = left14;
			num = num1;
		}
		else 
		{
			flag_left_ring_point_1 = biao21;
			flag_left_ring_point_2 = biao22;
			flag_left_ring_point_3 = biao23;
			flag_left_ring_point_4 = biao24;
			left_ring_point_1_tempt = left21;
			left_ring_point_2_tempt = left22;
			left_ring_point_3_tempt = left23;
			left_ring_point_4_tempt = left24;
			num = num2;
		}
//							 ips200_clear();
							 ips200_show_int(50, 200, left_ring_point_1_tempt, 2);
							 ips200_show_int(82, 200, left_ring_point_2_tempt, 2);
							 ips200_show_int(114, 200, left_ring_point_3_tempt, 2);
							 ips200_show_int(146, 200, left_ring_point_4_tempt, 2);
//							 ips200_show_int(50, 264, left_ring_point_1, 2);
//							 ips200_show_int(100, 264, left_ring_point_1_tempt, 2);
            //��ʼ���ߣ�process=1��ʾ�����4���յ㣬�貹������
            //����2��3�յ�����߽�����λ��
		if (num == 4)
		{
            for(max_2_3 = left_ring_point_2_tempt;max_2_3>=left_ring_point_3_tempt;max_2_3--)
            {
                if(ImageDeal[max_2_3].LeftBorder < ImageDeal[max_2_3+1].LeftBorder)
                {
                    break;
                }
            }
           k_bu_1 = (float)(ImageDeal[left_ring_point_1_tempt+2].LeftBorder - ImageDeal[max_2_3+1].LeftBorder) / (left_ring_point_1_tempt+2 - (max_2_3+1));
//           b_bu_1 = ImageDeal[left_ring_point_1_tempt+2].LeftBorder - k_bu_1*(left_ring_point_1_tempt - 57);
					 b_bu_1 = ImageDeal[left_ring_point_1_tempt+2].LeftBorder - k_bu_1*(left_ring_point_1_tempt - 59);
           for(int j = left_ring_point_1_tempt+2; j>= max_2_3;j--)
           {
               ImageDeal[j].LeftBorder = k_bu_1 * (j-59) + b_bu_1;
           }

           k_bu_2 = (float)(ImageDeal[left_ring_point_4_tempt-1].LeftBorder - ImageDeal[left_ring_point_4_tempt + 5].RightBorder) / (-6.0);
           b_bu_2 = ImageDeal[left_ring_point_4_tempt + 5].RightBorder - k_bu_2 * (left_ring_point_4_tempt -54);
           for(int j = left_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
           {
               ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
               if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
               {
                   ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
               }
           }

           //�޸�����ֵ
           for(int d = 59;d>=0;d--)
           {
               ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
           }
					     flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
               flag_left_ring_point_2 = 0;
               flag_left_ring_point_3 = 0;
               flag_left_ring_point_4 = 0;

        }
		if (num == 3 && zhuan > 100)
		{
			zhuan = 0;
			left_ring_process = 2;
		}
		flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
		left_ring_point_1_tempt = 0;
		left_ring_point_2_tempt = 0;
		left_ring_point_3_tempt = 0;
		left_ring_point_4_tempt = 0;
		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
		}
      if(left_ring_process == 2)                             //�󻷵�����2����������ҵ�3���յ�
        {
//					left_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //�󻷵��������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
            left14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao24 = 1;
				left24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao23 = 1;
				left23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
			{
				biao22 = 1;
				left22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
			{
				biao21 = 1;
        left21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_left_ring_point_1 = biao11;
			flag_left_ring_point_2 = biao12;
			flag_left_ring_point_3 = biao13;
			flag_left_ring_point_4 = biao14;
			left_ring_point_1_tempt = left11;
			left_ring_point_2_tempt = left12;
			left_ring_point_3_tempt = left13;
			left_ring_point_4_tempt = left14;
			num = num1;
		}
		else 
		{
			flag_left_ring_point_1 = biao21;
			flag_left_ring_point_2 = biao22;
			flag_left_ring_point_3 = biao23;
			flag_left_ring_point_4 = biao24;
			left_ring_point_1_tempt = left21;
			left_ring_point_2_tempt = left22;
			left_ring_point_3_tempt = left23;
			left_ring_point_4_tempt = left24;
			num = num2;
		}
							 ips200_show_int(50, 248, left_ring_point_2_tempt, 2);
							 ips200_show_int(100, 248, left_ring_point_3_tempt, 2);
							 ips200_show_int(150, 248, left_ring_point_4_tempt, 2);

            //��ʼ���ߣ���������
            //����2��3�յ�����߽�����λ��
		if (num == 3)
		{
            for(max_2_3 = left_ring_point_2_tempt;max_2_3>=left_ring_point_3_tempt;max_2_3--)
            {
                if(ImageDeal[max_2_3].LeftBorder < ImageDeal[max_2_3+1].LeftBorder)
                {
                    break;
                }
            }

            k_bu_1 = (float)(ImageDeal[59].LeftBorder - ImageDeal[max_2_3+1].LeftBorder) / (59 - (max_2_3+1));
            b_bu_1 = ImageDeal[59].LeftBorder;
            for(int j = 59; j>= max_2_3;j--)
            {
                ImageDeal[j].LeftBorder = k_bu_1 * (j-59) + b_bu_1;
            }

            k_bu_2 = (float)(ImageDeal[left_ring_point_4_tempt-1].LeftBorder - ImageDeal[left_ring_point_4_tempt + 5].RightBorder) / (-6.0);
            b_bu_2 = ImageDeal[left_ring_point_4_tempt + 5].RightBorder - k_bu_2 * (left_ring_point_4_tempt -54);
            for(int j = left_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
            {
                ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
                if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
                {
                    ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
                }
            }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                if(d > left_ring_point_3_tempt && d< left_ring_point_4_tempt)
                    ImageDeal[d].LeftBorder = 0;
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }

        }
		if (num == 2)
		{
			      k_bu_2 = (float)(ImageDeal[left_ring_point_4_tempt-1].LeftBorder - ImageDeal[left_ring_point_4_tempt + 5].RightBorder) / (-6.0);
            b_bu_2 = ImageDeal[left_ring_point_4_tempt + 5].RightBorder - k_bu_2 * (left_ring_point_4_tempt -54);
            for(int j = left_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
            {
                ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
                if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
                {
                    ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
                }
            }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                if(d > left_ring_point_3_tempt && d< left_ring_point_4_tempt)
                    ImageDeal[d].LeftBorder = 0;
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
		}
		if (left_ring_point_4_tempt >23)
		{
			left_ring_process = 3;
		}
		flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
		left_ring_point_1_tempt = 0;
		left_ring_point_2_tempt = 0;
		left_ring_point_3_tempt = 0;
		left_ring_point_4_tempt = 0;
		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
			}
     if(left_ring_process == 3)                             //�󻷵�����3����������ҵ�1���յ�
        {

            if(flag_clear_process_3 == 0)
            {
                Angle_Sum_z = 0.0;
                flag_clear_process_3 = 1;
            }
						Angle_Sum_z++;
            if(Angle_Sum_z > 200)                      //�󻷵���תһ���Ƕȴӹ���3�������4
            {
                left_ring_process = 0;
            }
//						for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
//              {
//                  if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
//                   {
//                        flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
//                        left_ring_point_4_tempt = i-1;
//                        break;
//                   }
//              }
//						if (flag_left_ring_point_4 == 1)
//						{
//					  	k_bu_2 = (float)(ImageDeal[left_ring_point_4_tempt-1].LeftBorder - ImageDeal[left_ring_point_4_tempt + 5].RightBorder) / (-6.0);
//              b_bu_2 = ImageDeal[left_ring_point_4_tempt + 5].RightBorder - k_bu_2 * (left_ring_point_4_tempt -54);
//              for(int j = left_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
//              {
//                ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
//                if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
//                {
//                    ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
//                }
//              }

//            //�޸�����ֵ
//              for(int d = 59;d>=0;d--)
//              {
//                if(d > left_ring_point_3_tempt && d< left_ring_point_4_tempt)
//                    ImageDeal[d].LeftBorder = 0;
//                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
//              }
//				  	}
//						if(left_ring_point_4_tempt >= 44 && left_ring_point_4_tempt <= 48)
//            {
//                left_ring_process = 4;
//            }
//		flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
//    flag_left_ring_point_2 = 0;
//    flag_left_ring_point_3 = 0;
//    flag_left_ring_point_4 = 0;
//		left_ring_point_1_tempt = 0;
//		left_ring_point_2_tempt = 0;
//		left_ring_point_3_tempt = 0;
//		left_ring_point_4_tempt = 0;
//		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
//	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
//	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
//	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
//	  num = 0;num1 = 0;num2 =0;
        }
      if(left_ring_process == 4)   //����4������������Ѳ�ߺͳ���,ͨ�������ǻ��ֽǶȵ���һ��ֵ��3����4
        {
					if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //����4��������׼��ʹ�õ��Ѳ��
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //ͬ������4�������ۼ�һ���ǶȺ󣨼������󣩣��������5
            if(Angle_Sum_z > 1000)
            {
                left_ring_process = 0;
            }
						for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
               {
                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                       left_ring_point_4_tempt = i-1;
                       break;
                   }
               }
						if (flag_left_ring_point_4 == 1)
						{
							for (i=left_ring_point_4_tempt;i>=ImageStatus.OFFLine;i--)
							 {
								 ImageDeal[i].RightBorder=40;
							 }
							             //�޸�����ֵ
              for(int d = 59;d>=0;d--)
              {
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
              }
					  }
//						if (left_ring_point_4_tempt >= 44 && left_ring_point_4_tempt <= 48)
//						{
//							left_ring_process = 5;
//						}
//						biaonow = flag_left_ring_point_4;
//						if (biaoshang == 1 && biaonow == 0)
//						{
//							left_ring_process = 5;
//						}
//						biaoshang = biaonow;
		flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
		left_ring_point_1_tempt = 0;
		left_ring_point_2_tempt = 0;
		left_ring_point_3_tempt = 0;
		left_ring_point_4_tempt = 0;
		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
        }
        if(left_ring_process == 5)                             //����5�͹���3������е��񣬵����߲�һ��
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
              {
                  if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                        flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                        left_ring_point_4_tempt = i-1;
                        break;
                   }
              }

            //�������յ��λ�ô�Զ�����˽���һ����ֵ�ڣ�������Ϊ�󻷵����ֽ���
            if(left_ring_point_4_tempt >= 34 && left_ring_point_4_tempt <= 38)
            {
                flag_left_ring = 0;                                     //����󻷵���־
                left_ring_process = 0;
            }

            k_bu_2 = (float)(ImageDeal[59].LeftBorder - ImageDeal[left_ring_point_4_tempt].LeftBorder)/(59 - left_ring_point_4_tempt);
            b_bu_2 = ImageDeal[59].LeftBorder;

            for(int i = 59;i >= left_ring_point_4_tempt - 2 ;i--)
            {
                ImageDeal[i].LeftBorder = k_bu_2 * (i-59) + b_bu_2;
            }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
		flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
		left_ring_point_1_tempt = 0;
		left_ring_point_2_tempt = 0;
		left_ring_point_3_tempt = 0;
		left_ring_point_4_tempt = 0;
		left11 = 0;left12 = 0;left13 = 0;left14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  left21 = 0;left22 = 0;left23 = 0;left24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
        }
        flag_left_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
        flag_left_ring_point_2 = 0;
        flag_left_ring_point_3 = 0;
        flag_left_ring_point_4 = 0;

        left_ring_point_1_tempt = 0;
        left_ring_point_2_tempt = 0;
        left_ring_point_3_tempt = 0;
        left_ring_point_4_tempt = 0;

    }
}
//�һ�����ʶ��
//�����Ϊֱ������߳��֡�ȱ��-һ���-ȱ��-��������-�������ߡ������
void find_right_ring_2()
{
    int i= 0;
	  int right11 = 0,right12 = 0,right13 = 0,right14 = 0;
	  int biao11 = 0,biao12 = 0,biao13 = 0,biao14 = 0;
	  int right21 = 0,right22 = 0,right23 = 0,right24 = 0;
	  int biao21 = 0,biao22 = 0,biao23 = 0,biao24 = 0;
	  int num = 0,num1 = 0,num2 =0;
	  int biaoshang = 0,biaonow = 0;
    if(flag_right_ring == 0)
    {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //�һ����������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�һ����������ϵĵ�2���յ㱻�ҵ�����¼λ��
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�һ����������ϵĵ�3���յ㱻�ҵ�����¼λ��
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�һ����������ϵĵ�4���յ㱻�ҵ�����¼λ��
            right14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao24 = 1;
				right24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao23 = 1;
				right23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao22 = 1;
				right22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao21 = 1;
        right21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_right_ring_point_1 = biao11;
			flag_right_ring_point_2 = biao12;
			flag_right_ring_point_3 = biao13;
			flag_right_ring_point_4 = biao14;
			right_ring_point_1_tempt = right11;
			right_ring_point_2_tempt = right12;
			right_ring_point_3_tempt = right13;
			right_ring_point_4_tempt = right14;
			num = num1;
		}
		else 
		{
			flag_right_ring_point_1 = biao21;
			flag_right_ring_point_2 = biao22;
			flag_right_ring_point_3 = biao23;
			flag_right_ring_point_4 = biao24;
			right_ring_point_1_tempt = right21;
			right_ring_point_2_tempt = right22;
			right_ring_point_3_tempt = right23;
			right_ring_point_4_tempt = right24;
			num = num2;
		}

    //�һ������жϻ���Ҫ�������Ϊֱ�����������һ������Ԫ��ʶ��Ĳ������ж�������Ƿ�Ϊ��ֱ��
    for(i=53;i>=ImageStatus.OFFLine;i--)
    {
        if(ImageDeal[i].IsRightFind == 'W' && ImageDeal[i -1].IsRightFind == 'T')
        {
            y_left_road_tempt = i;
            break;
        }
    }
    a2 = (ImageDeal[y_left_road_tempt-4].LeftBorder - ImageDeal[y_left_road_tempt-7].LeftBorder)/3.0;
    BB2 = ImageDeal[y_left_road_tempt-4].LeftBorder - a2 * (y_left_road_tempt-63);
    
    for(i = y_left_road_tempt-2;i > ImageStatus.OFFLine;i--)
    {
//			ips200_show_int(0, 216, (a2*(i-59) + BB2) - ImageDeal[i].RightBorder, 10);
        if(((a2*(i-59) + BB2) - ImageDeal[i].LeftBorder) > 3 || ((a2*(i-59) + BB2) - ImageDeal[i].LeftBorder) < -3)
        {
            break;
        }
    }
     y_left_length = y_left_road_tempt - i;

    //�ж��Ƿ�Ϊ�һ���,�ұ��ҵ��ĸ��յ㣬�����Ϊֱ��
    if(flag_right_ring_point_1 == 1 && flag_right_ring_point_2 == 1 && flag_right_ring_point_3 == 1 && flag_right_ring_point_4 == 1 && y_left_length >= 25 && Enter_Crosses_Process == 0)
    {
        flag_right_ring = 1;
        right_ring_process = 1;              //�����һ�������1�����ұ����ҵ�4���յ�
    }
   /* else
        flag_left_ring = 0;*/

    flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
    }

    //�һ����Ĳ��ߴ���
    if(flag_right_ring == 1)
    {
			
        if(right_ring_process == 1)                      //�һ�������1�������ұ����ҵ�4���յ�
        {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //�һ����������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�һ����������ϵĵ�2���յ㱻�ҵ�����¼λ��
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�һ����������ϵĵ�3���յ㱻�ҵ�����¼λ��
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�һ����������ϵĵ�4���յ㱻�ҵ�����¼λ��
            right14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao24 = 1;
				right24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao23 = 1;
				right23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao22 = 1;
				right22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao21 = 1;
        right21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_right_ring_point_1 = biao11;
			flag_right_ring_point_2 = biao12;
			flag_right_ring_point_3 = biao13;
			flag_right_ring_point_4 = biao14;
			right_ring_point_1_tempt = right11;
			right_ring_point_2_tempt = right12;
			right_ring_point_3_tempt = right13;
			right_ring_point_4_tempt = right14;
			num = num1;
		}
		else 
		{
			flag_right_ring_point_1 = biao21;
			flag_right_ring_point_2 = biao22;
			flag_right_ring_point_3 = biao23;
			flag_right_ring_point_4 = biao24;
			right_ring_point_1_tempt = right21;
			right_ring_point_2_tempt = right22;
			right_ring_point_3_tempt = right23;
			right_ring_point_4_tempt = right24;
			num = num2;
		}
//							 ips200_clear();
//							 ips200_show_int(50, 200, right_ring_point_1_tempt, 2);
//							 ips200_show_int(82, 200, right_ring_point_2_tempt, 2);
//							 ips200_show_int(114, 200, right_ring_point_3_tempt, 2);
//							 ips200_show_int(146, 200, right_ring_point_4_tempt, 2);
//							 ips200_show_int(50, 264, left_ring_point_1, 2);
//							 ips200_show_int(100, 264, left_ring_point_1_tempt, 2);
            //��ʼ���ߣ�process=1��ʾ�����4���յ㣬�貹������
            //����2��3�յ�����߽�����λ��
		if (num == 4)
		{
            //��ʼ���ߣ�process=1��ʾ�Ҳ���4���յ㣬�貹������
            //����2��3�յ����ұ߽�����λ��
            for(max_2_3_right = right_ring_point_2_tempt;max_2_3_right>=right_ring_point_3_tempt;max_2_3_right--)
            {
                if(ImageDeal[max_2_3_right].RightBorder > ImageDeal[max_2_3_right+1].RightBorder)
                {
                    break;
                }
            }
           k_bu_1_right = (float)((ImageDeal[right_ring_point_1_tempt+2].RightBorder - ImageDeal[max_2_3_right+1].RightBorder) / (right_ring_point_1_tempt+2 - (max_2_3+1)));
//           b_bu_1_right = ImageDeal[right_ring_point_1_tempt+2].RightBorder - k_bu_1_right*(right_ring_point_1_tempt - 57);
						 b_bu_1_right = ImageDeal[right_ring_point_1_tempt+2].RightBorder - k_bu_1_right*(right_ring_point_1_tempt - 59);
//						ips200_show_int(50, 300, ImageDeal[right_ring_point_1_tempt+2].RightBorder, 2);
//						ips200_show_int(70, 300, ImageDeal[max_2_3_right+1].RightBorder, 2);
//						ips200_show_int(50, 284, right_ring_point_1_tempt+2, 2);
//						ips200_show_int(70, 284, max_2_3_right+1, 2);
           for(int j = right_ring_point_1_tempt+2; j>= max_2_3_right;j--)
           {
               ImageDeal[j].RightBorder = k_bu_1_right * (j-59) + b_bu_1_right;
           }

           k_bu_2_right = (float)(ImageDeal[right_ring_point_4_tempt-1].RightBorder - ImageDeal[right_ring_point_4_tempt + 5].LeftBorder) / (-6.0);
           b_bu_2_right = ImageDeal[right_ring_point_4_tempt + 5].LeftBorder - k_bu_2_right * (right_ring_point_4_tempt -54);
           for(int j = right_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
           {
               ImageDeal[j].LeftBorder = k_bu_2_right * (j-59) + b_bu_2_right;
               if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
               {
                   ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
               }
           }

           //�޸�����ֵ
           for(int d = 59;d>=0;d--)
           {
               ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
           }
					     flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
               flag_right_ring_point_2 = 0;
               flag_right_ring_point_3 = 0;
               flag_right_ring_point_4 = 0;

        }
		if (num == 3)
		{
			right_ring_process = 2;
		}
		flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
		}
      if(right_ring_process == 2)                             //�󻷵�����2����������ҵ�3���յ�
        {
//					right_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //�һ����������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            biao12 = 1;                         //�һ����������ϵĵ�2���յ㱻�ҵ�����¼λ��
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao13 = 1;                         //�һ����������ϵĵ�3���յ㱻�ҵ�����¼λ��
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            biao14 = 1;                         //�һ����������ϵĵ�4���յ㱻�ҵ�����¼λ��
            right14 = i-1;
					num1++;
        }
    }
		for(i=ImageStatus.OFFLine;i<=53;i++) 
		{
			if (biao24 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao24 = 1;
				right24 = i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao23 = 1;
				right23 = i;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')
			{
				biao22 = 1;
				right22 =i-1;
				num2++;
			}
			if (biao24 == 1 && biao23 == 1 && biao22 == 1 && biao21 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
			{
				biao21 = 1;
        right21 = i-1;
        num2++;				
			}
		}
		if (num1 > num2)
		{
			flag_right_ring_point_1 = biao11;
			flag_right_ring_point_2 = biao12;
			flag_right_ring_point_3 = biao13;
			flag_right_ring_point_4 = biao14;
			right_ring_point_1_tempt = right11;
			right_ring_point_2_tempt = right12;
			right_ring_point_3_tempt = right13;
			right_ring_point_4_tempt = right14;
			num = num1;
		}
		else 
		{
			flag_right_ring_point_1 = biao21;
			flag_right_ring_point_2 = biao22;
			flag_right_ring_point_3 = biao23;
			flag_right_ring_point_4 = biao24;
			right_ring_point_1_tempt = right21;
			right_ring_point_2_tempt = right22;
			right_ring_point_3_tempt = right23;
			right_ring_point_4_tempt = right24;
			num = num2;
		}
//							 ips200_show_int(50, 248, right_ring_point_2_tempt, 2);
//							 ips200_show_int(100, 248, right_ring_point_3_tempt, 2);
//							 ips200_show_int(150, 248, right_ring_point_4_tempt, 2);

            //��ʼ���ߣ���������
            //����2��3�յ�����߽�����λ��
		if (num == 3)
		{
            //��ʼ���ߣ���������
            //����2��3�յ����ұ߽�����λ��
            for(max_2_3 = right_ring_point_2_tempt;max_2_3>=right_ring_point_3_tempt;max_2_3--)
            {
                if(ImageDeal[max_2_3].RightBorder > ImageDeal[max_2_3+1].RightBorder)
                {
                    break;
                }
            }

            k_bu_1_right = (float)(ImageDeal[59].RightBorder - ImageDeal[max_2_3+1].RightBorder) / (59 - (max_2_3+1));
            b_bu_1_right = ImageDeal[59].RightBorder;
            for(int j = 59; j>= max_2_3;j--)
            {
                ImageDeal[j].RightBorder = k_bu_1_right * (j-59) + b_bu_1_right;
            }

           k_bu_2_right = (float)(ImageDeal[right_ring_point_4_tempt-1].RightBorder - ImageDeal[right_ring_point_4_tempt + 5].LeftBorder) / (-6.0);
           b_bu_2_right = ImageDeal[right_ring_point_4_tempt + 5].LeftBorder - k_bu_2_right * (right_ring_point_4_tempt -54);
           for(int j = right_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
           {
               ImageDeal[j].LeftBorder = k_bu_2_right * (j-59) + b_bu_2_right;
               if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
               {
                   ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
               }
           }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                if(d > right_ring_point_3_tempt && d< right_ring_point_4_tempt)
                    ImageDeal[d].LeftBorder = 0;
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }

        }
		if (num == 2)
		{
			     k_bu_2_right = (float)(ImageDeal[right_ring_point_4_tempt-1].RightBorder - ImageDeal[right_ring_point_4_tempt + 5].LeftBorder) / (-6.0);
           b_bu_2_right = ImageDeal[right_ring_point_4_tempt + 5].LeftBorder - k_bu_2_right * (right_ring_point_4_tempt -54);
           for(int j = right_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
           {
               ImageDeal[j].LeftBorder = k_bu_2_right * (j-59) + b_bu_2_right;
               if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
               {
                   ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
               }
           }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                if(d > right_ring_point_3_tempt && d< right_ring_point_4_tempt)
                    ImageDeal[d].LeftBorder = 0;
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
		}
		if (right_ring_point_4_tempt >18)
		{
			right_ring_process = 3;
		}
		flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
			}
     if(right_ring_process == 3)                             //�󻷵�����3����������ҵ�1���յ�
        {

            if(flag_clear_process_3_right == 0)
            {
                Angle_Sum_z = 0.0;
                flag_clear_process_3_right = 1;
            }
						Angle_Sum_z++;
            if(Angle_Sum_z > 200)                      //�󻷵���תһ���Ƕȴӹ���3�������4
            {
                right_ring_process = 0;
            }
					
//							for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
//              {
//                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
//                   {
//                        flag_right_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
//                        right_ring_point_4_tempt = i-1;
//                        break;
//                   }
//              }
//							if (flag_right_ring_point_4 == 1)
//							{
//								k_bu_2_right = (float)(ImageDeal[right_ring_point_4_tempt-1].RightBorder - ImageDeal[right_ring_point_4_tempt + 5].LeftBorder) / (-6.0);
//                b_bu_2_right = ImageDeal[right_ring_point_4_tempt + 5].LeftBorder - k_bu_2_right * (right_ring_point_4_tempt -54);
//                for(int j = right_ring_point_4_tempt + 5;j > ImageStatus.OFFLine;j--)
//                 {
//                    ImageDeal[j].LeftBorder = k_bu_2_right * (j-59) + b_bu_2_right;
//                    if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
//                    {
//                        ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
//                    }
//                 }

//            //�޸�����ֵ
//            for(int d = 59;d>=0;d--)
//            {
//                if(d > right_ring_point_3_tempt && d< right_ring_point_4_tempt)
//                    ImageDeal[d].LeftBorder = 0;
//                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
//            }
//							}
//							if (right_ring_point_4_tempt >= 44 && right_ring_point_4_tempt <= 48)
//							{
//								right_ring_process = 4;
//							}
		flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
        }
      if(right_ring_process == 4)   //����4������������Ѳ�ߺͳ���,ͨ�������ǻ��ֽǶȵ���һ��ֵ��3����4
        {
					if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //����4��������׼��ʹ�õ��Ѳ��
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //ͬ������4�������ۼ�һ���ǶȺ󣨼������󣩣��������5
            if(Angle_Sum_z > 1000)
            {
//                flag_right_ring = 0;                                     //����һ�����־
                right_ring_process = 0;
            }
					
						for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
              {
                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                        flag_right_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                        right_ring_point_4_tempt = i-1;
                        break;
                   }
              }
							if (flag_right_ring_point_4 == 1)
							{
								for (i=right_ring_point_4_tempt;i>=ImageStatus.OFFLine;i--)
							 {
								 ImageDeal[i].RightBorder=20;
							 }
							 for(int d = 59;d>=0;d--)
               {
                 ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
               }
							 }
//						if (right_ring_point_4_tempt >= 44 && right_ring_point_4_tempt <= 48)
//						{
//							right_ring_process = 5;
//						}
//						biaonow = flag_left_ring_point_4;
//						if (biaoshang == 1 && biaonow == 0)
//						{
//							left_ring_process = 5;
//						}
//						biaoshang = biaonow;
		flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;			
        }
        if(right_ring_process == 5)                             //����5�͹���3������е��񣬵����߲�һ��
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
              {
                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                        flag_right_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                        right_ring_point_4_tempt = i-1;
                        break;
                   }
              }

            //�������յ��λ�ô�Զ�����˽���һ����ֵ�ڣ�������Ϊ�󻷵����ֽ���
            if(right_ring_point_4_tempt >= 34 && right_ring_point_4_tempt <= 38)
            {
                flag_right_ring = 0;                                     //����󻷵���־
                right_ring_process = 0;
            }
						
            if (flag_right_ring_point_4 == 1)
						{
            k_bu_2 = (float)(ImageDeal[59].RightBorder - ImageDeal[left_ring_point_4_tempt].RightBorder)/(59 - right_ring_point_4_tempt);
            b_bu_2 = ImageDeal[59].RightBorder;

            for(int i = 59;i >= right_ring_point_4_tempt - 2 ;i--)
            {
                ImageDeal[i].RightBorder = k_bu_2 * (i-59) + b_bu_2;
            }

            //�޸�����ֵ
            for(int d = 59;d>=0;d--)
            {
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
					  }
		flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
    flag_right_ring_point_2 = 0;
    flag_right_ring_point_3 = 0;
    flag_right_ring_point_4 = 0;
		right_ring_point_1_tempt = 0;
		right_ring_point_2_tempt = 0;
		right_ring_point_3_tempt = 0;
		right_ring_point_4_tempt = 0;
		right11 = 0;right12 = 0;right13 = 0;right14 = 0;
	  biao11 = 0;biao12 = 0;biao13 = 0;biao14 = 0;
	  right21 = 0;right22 = 0;right23 = 0;right24 = 0;
	  biao21 = 0;biao22 = 0;biao23 = 0;biao24 = 0;
	  num = 0;num1 = 0;num2 =0;
        }
        flag_right_ring_point_1 = 0;             //�������Ӱ����һ�γ������ú���ʱ��������
        flag_right_ring_point_2 = 0;
        flag_right_ring_point_3 = 0;
        flag_right_ring_point_4 = 0;

        right_ring_point_1_tempt = 0;
        right_ring_point_2_tempt = 0;
        right_ring_point_3_tempt = 0;
        right_ring_point_4_tempt = 0;

    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------
//ͼ�������������������������������������ִ��
//----------------------------------------------------------------------------------------------------------------------------------------------
int Image_Process(void)
{
		       	 Image_Compress();           //ͼ��ѹ������ԭʼ��120*188��ͼ��ѹ����60*80�ġ�������ȫ�ֱ���Image_Use��
             Get_BinaryImage();          //ͼ���ֵ�������Ѳɼ�����ԭʼ�Ҷ�ͼ���ɶ�ֵ��ͼ�񡣴����Pixle_hb��80*60
             Get_BaseLine();             //�Ż�֮��������㷨���õ�һ��ͼ��Ļ������ߣ�Ҳ������������еı�����Ϣ��������������
             Get_AllLine();              //�Ż�֮��������㷨���õ�һ��ͼ���ȫ�����ߺ����ߡ�
             road_type_2();                   //�ж���������
	           if (faajishu > 0)
						 {
                find_zebra();                //ʶ�������
						 }
             Element_Judgment_Crosses(); //�ж��Ƿ�Ϊʮ��·�ڲ�������д���

             find_openRoad();               //ʶ���Ƿ�Ϊ��·
             bu();                  //ת�䶪�߲���
             stop_car();                //������ͣ�����
             ips200_show_gray_image(160, 200, Image_Use[0], 80, 60, 80, 60, 0);
			       ips200_show_gray_image(160, 260, Pixle_hb[0], 80, 60, 80, 60, 0);
//						 Get_angle();
////						 Get_angle();
//					  ips200_show_int(0, 232, Angle_Sum_z, 10);
//						 ips200_show_int(0, 248, Angle_z, 10);
//						 ips200_show_int(0, 216, flag_left_ring, 2);
//						 ips200_show_int(0, 232, left_ring_process, 2);
//						 ips200_show_int(0, 216, flag_right_ring, 2);
//						 ips200_show_int(0, 232, right_ring_process, 2);
//						 ips200_show_int(0, 248, ImageStatus.Miss_Right_lines , 2);
//			       ips200_show_int(0, 264, ImageStatus.Miss_Left_lines , 2);

						 ips200_show_int(0, 300, angle, 2);

						 

    		     angle=Slope_cal_2();
             Center_line();
			       Left_right_line();
//						 ringfig();
						 return angle;
}

