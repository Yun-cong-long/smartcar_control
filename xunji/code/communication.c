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


//�󻷵���ʶ��
//�ұ���Ϊֱ������߳��֡�ȱ��-һ���-ȱ��-��������-�������ߡ������
void find_left_ring()
{
    int i= 0;
    if(flag_left_ring == 0)
    {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
    {
        if(flag_left_ring_point_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            flag_left_ring_point_1 = 1;                         //�󻷵��������ϵĵ�һ���յ㱻�ҵ�����¼λ��
            left_ring_point_1 = i;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
        {
            flag_left_ring_point_2 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
            left_ring_point_2 = i-1;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            flag_left_ring_point_3 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
            left_ring_point_3 = i;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
        {
            flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
            left_ring_point_4 = i-1;
            break;
        }
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
    }

    //�󻷵��Ĳ��ߴ���
    if(flag_left_ring == 1)
    {
			
        if(left_ring_process == 1)                      //�󻷵�����1����������ҵ�4���յ�
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
               {
                   if(flag_left_ring_point_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
                   {
                       flag_left_ring_point_1 = 1;                         //�󻷵��������ϵĵ�һ���յ㱻�ҵ�����¼λ��
                       left_ring_point_1_tempt = i;
                       if(left_ring_point_1_tempt < left_ring_point_1)      //�����Ҵ������ϵĵ�һ���յ��Ѿ���������
                       {
                           left_ring_process = 2;
                           break;
                       }
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
                   {
                       flag_left_ring_point_2 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
                       left_ring_point_2_tempt = i-1;
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                       flag_left_ring_point_3 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
                       left_ring_point_3_tempt = i;
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                       left_ring_point_4_tempt = i-1;
                       break;
                   }
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
        else if(left_ring_process == 2)                             //�󻷵�����2����������ҵ�3���յ�
        {
					left_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
//					  for(i=ImageStatus.OFFLine;i<=53;i++)
					ips200_show_int(50, 300, ImageStatus.OFFLine, 2);
					  for(i=53;i>=ImageStatus.OFFLine;i--)
               {
                   if(flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
                   {
                       flag_left_ring_point_2 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
                       left_ring_point_2_tempt = i-1;
                       if(left_ring_point_2_tempt > 17 && left_ring_point_2_tempt < 25)
                       {        
												   
//												   Angle_Sum_z=Angle_z;
												   left_ring_process = 3;
                           break;
                       }
                   }
                   else if(flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                       flag_left_ring_point_3 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
                       left_ring_point_3_tempt = i;
//										   if(left_ring_point_3_tempt > 40 && left_ring_point_3_tempt < 48)
//                       {        
//												   
////												   Angle_Sum_z=Angle_z;
//												   left_ring_process = 3;
//                           break;
//                       }

                   }
                   else if(flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
                   {
                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
                       left_ring_point_4_tempt = i-1;
//										   ips200_show_int(0, 248, left_ring_point_4_tempt, 10);
//										   if(left_ring_point_4_tempt > 18 && left_ring_point_4_tempt < 26)
//                       {        
//												   
////												   Angle_Sum_z=Angle_z;
//												   left_ring_process = 3;
//                           break;
//                       }
                       break;
                   }
//                    if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')
//										{
//                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
//                       left_ring_point_4_tempt = i-1;
////										   ips200_show_int(0, 248, left_ring_point_4_tempt, 10);
////										   if(left_ring_point_4_tempt > 18 && left_ring_point_4_tempt < 26)
////                       {        
////												   
//////												   Angle_Sum_z=Angle_z;
////												   left_ring_process = 3;
////                           break;
////                       }
//                      
//                   }
//										if(flag_left_ring_point_4 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
//                   {
//                       flag_left_ring_point_3 = 1;                         //�󻷵��������ϵĵ�3���յ㱻�ҵ�����¼λ��
//                       left_ring_point_3_tempt = i;
////										   if(left_ring_point_3_tempt > 40 && left_ring_point_3_tempt < 48)
////                       {        
////												   
//////												   Angle_Sum_z=Angle_z;
////												   left_ring_process = 3;
////                           break;
////                       }

//                   }
//									  if(flag_left_ring_point_4 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ�е�һ���ҵ��Ž���ȥ��ʣ��������
//                   {
//                       flag_left_ring_point_2 = 1;                         //�󻷵��������ϵĵ�2���յ㱻�ҵ�����¼λ��
//                       left_ring_point_2_tempt = i-1; 
//										    
////                       if(left_ring_point_2_tempt > 10 && left_ring_point_2_tempt < 18)
////                       {        
////												   
//////												   Angle_Sum_z=Angle_z;
////												   left_ring_process = 3;
////                           break;
////                       }
//										  break;
//                   }
               }
							 ips200_show_int(50, 248, left_ring_point_2_tempt, 2);
							 ips200_show_int(100, 248, left_ring_point_3_tempt, 2);
							 ips200_show_int(150, 248, left_ring_point_4_tempt, 2);
            /*if(flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && flag_left_ring_point_4 == 0 && left_ring_point_3_tempt < left_ring_point_3)
            {
                left_ring_process = 3;
            }*/
            //��ʼ���ߣ���������
            //����2��3�յ�����߽�����λ��
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
				
        else if(left_ring_process == 3)                             //�󻷵�����3����������ҵ�1���յ�
        {
					
//					if(ImageStatus.Miss_Right_lines < 15)//ȡ���ڶ��β���
//					{
//						left_ring_process = 5;
//					}
//            if(flag_clear_process_3 == 0)
//            {
//                Angle_Sum_z = 0.0;
//                flag_clear_process_3 = 1;
//            }
//						Get_angle();
//					  ips200_show_int(0, 232, Angle_Sum_z, 10);
//						 ips200_show_int(0, 248, Angle_z, 10);
//            if(Angle_Sum_z > 54)                      //�󻷵���תһ���Ƕȴӹ���3�������4
//            {
//                left_ring_process = 4;
//            }
//            for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
//               {
//                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
//                   {
//                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
//                       left_ring_point_4_tempt = i-1;
//                       break;
//                   }
//               }
//            //��ʱ���߻��Ǵ��ұ�W-T�����俪ʼ�ȽϺ�
//            int k = 0;
//            for(k=53;k>=ImageStatus.OFFLine;k--)
//            {
//                if(ImageDeal[k].IsRightFind == 'W' && ImageDeal[k -1].IsRightFind == 'T')
//                {
//                    break;
//                }
//            }

//            if(k - left_ring_point_4_tempt > 14)
//            {
//                k_bu_2 = (float)(ImageDeal[left_ring_point_4_tempt-1].LeftBorder - ImageDeal[k-1].RightBorder) / (left_ring_point_4_tempt-k);
//                b_bu_2 = ImageDeal[k-1].RightBorder - k_bu_2 * (k-60);
//                for(int j = k-1;j > ImageStatus.OFFLine;j--)
//                {
//                    ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
//                    if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
//                    {
//                        ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
//                    }
//                }
//            }
//            else
//            {
//                k_bu_2 = (float)(ImageDeal[59].RightBorder - ImageDeal[left_ring_point_4_tempt].LeftBorder) / (59 - left_ring_point_4_tempt);
//                b_bu_2 = ImageDeal[59].RightBorder;
//                for(int j = 59;j > ImageStatus.OFFLine;j--)
//                {
//                    ImageDeal[j].RightBorder = k_bu_2 * (j-59) + b_bu_2;
//                    if(ImageDeal[j].RightBorder<ImageDeal[j].LeftBorder)
//                    {
//                        ImageDeal[j].RightBorder = ImageDeal[j].LeftBorder;
//                    }
//                }
//            }

//            //����3�϶�����߻���������
//            //�޸�����ֵ
//            for(int d = 59;d>=0;d--)
//            {
//                ImageDeal[d].LeftBorder=0;
//                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
//            }
            if(flag_clear_process_3 == 0)
            {
                Angle_Sum_z = 0.0;
                flag_clear_process_3 = 1;
            }
						Angle_Sum_z++;
            if(Angle_Sum_z > 200)                      //�󻷵���תһ���Ƕȴӹ���3�������4
            {
                left_ring_process = 4;
            }
        }
        else if(left_ring_process == 4)   //����4������������Ѳ�ߺͳ���,ͨ�������ǻ��ֽǶȵ���һ��ֵ��3����4
        {
            if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //����4��������׼��ʹ�õ��Ѳ��
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //ͬ������4�������ۼ�һ���ǶȺ󣨼������󣩣��������5
            if(Angle_Sum_z > 400)
            {
                left_ring_process = 5;
            }
//						for(i=53;i>=ImageStatus.OFFLine;i--)                       //��������Ϊ֮ǰɨ�߳����ԭ�򣬱�ʶ����Ϊ��T������Ӱ����֮����жϣ����������ǿ�ʼѭ��
//               {
//                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //ֻ��ǰ��Ĺյ��ҵ��˲�ȥ��ʣ�µ�
//                   {
//                       flag_left_ring_point_4 = 1;                         //�󻷵��������ϵĵ�4���յ㱻�ҵ�����¼λ��
//                       left_ring_point_4_tempt = i-1;
//                       break;
//                   }
//               }
//						if (flag_left_ring_point_4 == 1)
//						{
//							for (i=left_ring_point_4_tempt;i>=ImageStatus.OFFLine;i--)
//							 {
//								 ImageDeal[i].RightBorder=40;
//							 }
//							             //�޸�����ֵ
//              for(int d = 59;d>=0;d--)
//              {
//                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
//              }
//					  }
        }
        else if(left_ring_process == 5)                             //����5�͹���3������е��񣬵����߲�һ��
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
            if(left_ring_point_4_tempt >= 34 && left_ring_point_4_tempt <= 41)
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