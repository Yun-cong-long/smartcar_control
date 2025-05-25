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

/*******观测参数*********/
int flag_1 = 0;             //第一段标志
int flag_2 = 0;             //第二段标志
int flag_3 = 0;             //第三段标志
int flag_4 = 0;
extern bool is_roundabout;
uint8 Image_Use[LCDH][LCDW];      //用来存储压缩之后灰度图像的二维数组,60*80
uint8 Pixle[LCDH][LCDW];          //图像处理时真正处理的二值化图像数组
uint8 Pixle_hb[LCDH][LCDW];
uint8 Threshold;                                //通过大津法计算出来的二值化阈值
uint8 Bsite = 0;                                    //赛道宽度扩大标志

int ImageScanInterval=5;                        //扫边的范围
static uint8* PicTemp;                          //一个保存单行图像的指针变量
static int IntervalLow = 0, IntervalHigh = 0;   //扫描区间的上下限变量
static int Ysite = 0, Xsite = 0;                //Ysite就是图像的行，Xsite就是图像的列。

int Y = 0;
int X1,X2;

int leftup_x = 0, leftup_y = 0,rightup_x = 0, rightup_y = 0,rightdown_x = 0, rightdown_y = 0,leftdown_x = 0, leftdown_y = 0,rightdown_xb = 0, rightdown_yb = 0,leftdown_xb = 0, leftdown_yb = 0;
uint8 cross_firstflag=0;//十字初判标志
uint16 count_wnum=0;//白点计数变量
uint8 left_count=0;//左无边行计数
uint8 right_count=0;//右无边行计数
uint8_t sele_open = 1;     //断路方案选择，3个方案
uint8_t max_ring = 1;       //赛道上的总环数
uint8_t BZ_num = 1;         //赛道上的避障数量
uint8_t num_gray = 150;     //断路退出的灰度阈值
float A=0;
float B=0;
/*uint8 cross_flag=0;//十字标志
uint8 leftup_flag=0;
uint8 rightup_flag=0;
uint8 leftdown_flag=0;
uint8 rightdown_flag=0;//找到拐点标志*/
extern uint8 faajishu;
static int BottomBorderRight = 79,              //59行的右边界
BottomBorderLeft = 0,                           //59行的左边界
BottomCenter = 0;                               //59行的中点
uint8 ExtenLFlag = 0;                           //左边线是否需要补线的标志变量
uint8 ExtenRFlag = 0;                           //右边线是否需要补线的标志变量
ImageDealDatatypedef ImageDeal[60];             //记录单行信息的结构体数组
ImageStatustypedef ImageStatus;                 //图像处理的的全局变量
float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;
bool flag_stop_car = 0;
uint8 Enter_Crosses_Process=0;
static int ytemp = 0;                           //存放行的临时变量
static int TFSite = 0, FTSite = 0;              //补线计算斜率的时候需要用的存放行的变量。
static float DetR = 0, DetL = 0;                //存放补线斜率的变量
int Car_Barn_num = 0;
int Zebra_Cnt = 0;
int Zebra_Flag = 0;
uint8 Starting_lines = 0;//起跑线识别
uint8 Garage_Location = 0;//1 :车库在左边   2 ：车库在右边
extern uint8 flag_stop_Car;

int Midline_Weight_Woefficient[60]={    0,0,0,0,0,        //中线权重系数5
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
int Midline_Weight_Woefficient_ZHIJ[60]={  0,0,0,0,0,        //中线权重系数5
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
uint8 Get_Rings_First_Flag;  //获取进环第一次标志位
uint8 Rings_Second_Flag=0;   //进环第三阶段会出现2边丢线情况 与十字判断冲突所以设置的标志位
uint8 Enter_Rings_Process=0;
uint8 Enter_Rings_Flag=0;
float Out_Rings_Slope=0;//出环的斜率
uint8 Enter_Rings_Process_2=0;//第二阶段标志
int Enter_Rings_2_Ysite=0;
int Enter_Rings_2_Xsite=0;
uint8 Enter_Rings_Flag_1=0;
uint8 Enter_Rings_Flag_2=0;
uint8 Enter_Rings_Flag_3=0;
uint8 Enter_Rings_Flag_4=0;
uint8 Enter_Rings_A_Location=0;
uint8 Rings_Nums_Left=0;

int flag_clear_inRing = 0;      //入环时陀螺仪清零一次

/******************理想位置**********************/
float center_avg = 39;
float idea_Med = 39;            //理想中线位置

/******************断路的参数***********************/
uint8 flag_openRoad = 0;     //断路识别标志位

int flag_openRoad_timeOut = 0;     //半秒已到的标志
float K1 = 0.0;         //直线1的斜率
float K2 = 0.0;         //直线2的斜率
float b1 = 0.0;         //直线1的拮据
float b2 = 0.0;         //直线2的拮据

uint8 x_jiaoDian = 0;    //两直线交点,从屏幕来看就是横向的坐标
uint8 y_jiaoDian = 0;
int loca_i_left = 0;        //左边的从上往下的第一个无边行位置
int loca_i_right = 0;        //右边的从上往下的第一个无边行位置

int loca_j = 0;

//直道断路
//offline往后10行若至少有7行‘T’，且offline的道路宽在某区间内，且offline的值在某一区间内，则认为为直道断路
uint8_t cnt_str_open = 0;
int flag_str_open = 0;

int32 cnt_gray = 0;         //一定灰度值点个数计数
//识别到断路后半秒后再检测是否满足退出断路的条件
int cnt_openRoad_timeOut = 0;
/***************************赛道元素识别的参数******************************/
int flag_Straight = 0;    //直道的标志
int Str_len = 0;          //直道长度
int y_left_road_tempt = 0;        //左边记录第一次丢边处
int y_left_length = 0;

int y_right_road_tempt = 0;       //右边记录第一次丢边处
int y_right_length = 0;

float a1 = 0.0;         //直线1的斜率(左)
float a2 = 0.0;         //直线2的斜率(右)
float BB1 = 0.0;         //直线1的拮据(左)
float BB2 = 0.0;         //直线2的拮据(右)

/********************斑马线和车库的参数***********************/
uint8 flag_zebra = 0;
uint8 flag_once =0;     //斑马线转只清零一次
uint8 flag_stop = 0;        //入库停车标志

uint8 flag_out = 0;         //出赛道停车标志
/*******************左环岛的参数***********************/
//刚看到环岛的时候会有四个拐点，识别后随着小车向前运动，能看到的拐点减小，通过这个过程来补线
int left_ring_point_1 = 0;      //左环岛从下往上的第一个拐点的位置
int left_ring_point_1_tempt = 0;
int flag_left_ring_point_1 = 0; //左环岛从下往上的第一个拐点是否被找到的标志

int left_ring_point_2 = 0;      //左环岛从下往上的第二个拐点的位置
int left_ring_point_2_tempt = 0;
int flag_left_ring_point_2 = 0;//左环岛从下往上的第二个拐点是否被找到的标志

int left_ring_point_3 = 0;      //左环岛从下往上的第三个拐点的位置
int left_ring_point_3_tempt = 0;
int flag_left_ring_point_3 = 0;//左环岛从下往上的第三个拐点是否被找到的标志

int left_ring_point_4 = 0;      //左环岛从下往上的第四个拐点的位置
int left_ring_point_4_tempt = 0;
int flag_left_ring_point_4 = 0;//左环岛从下往上的第四个拐点是否被找到的标志

int flag_left_ring = 0;         //左环岛识别标志位

int left_ring_process = 0;      //左环岛过程
int point_num_left = 0;              //拐点数量

//因为最多同时补两条线，所以我定义两条补线直线的斜率和拮据
float k_bu_1 = 0.0;
float b_bu_1 = 0.0;

float k_bu_2 = 0.0;
float b_bu_2 = 0.0;

int max_2_3 = 0;                //拐点2和3之间左边界最大值的位置

int flag_clear_process_3 = 0;   //过程3陀螺仪只清零一次
int flag_clear_process_4 = 0;   //过程4陀螺仪只清零一次

/*******************左环岛的参数***********************/
//刚看到环岛的时候会有四个拐点，识别后随着小车向前运动，能看到的拐点减小，通过这个过程来补线
int right_ring_point_1 = 0;      //右环岛从下往上的第一个拐点的位置
int right_ring_point_1_tempt = 0;
int flag_right_ring_point_1 = 0; //右环岛从下往上的第一个拐点是否被找到的标志

int right_ring_point_2 = 0;      //右环岛从下往上的第二个拐点的位置
int right_ring_point_2_tempt = 0;
int flag_right_ring_point_2 = 0;//右环岛从下往上的第二个拐点是否被找到的标志

int right_ring_point_3 = 0;      //右环岛从下往上的第三个拐点的位置
int right_ring_point_3_tempt = 0;
int flag_right_ring_point_3 = 0;//右环岛从下往上的第三个拐点是否被找到的标志

int right_ring_point_4 = 0;      //右环岛从下往上的第四个拐点的位置
int right_ring_point_4_tempt = 0;
int flag_right_ring_point_4 = 0;//右环岛从下往上的第四个拐点是否被找到的标志

int flag_right_ring = 0;         //右环岛识别标志位

int right_ring_process = 0;      //右环岛过程
int point_num_right = 0;              //拐点数量

//因为最多同时补两条线，所以我定义两条补线直线的斜率和拮据
float k_bu_1_right = 0.0;
float b_bu_1_right = 0.0;

float k_bu_2_right = 0.0;
float b_bu_2_right = 0.0;

int max_2_3_right = 0;                //拐点2和3之间左边界最大值的位置

int flag_clear_process_3_right = 0;   //过程3陀螺仪只清零一次
int flag_clear_process_4_right = 0;   //过程4陀螺仪只清零一次

/*************************十字的参数********************************/
int flag_cross = 0;                 //十字的识别标志
int flag_crossring=0;				//十字环的识别标志
int left_Ysite_1 = 0;               //十字左边线第一个拐点
int left_Ysite_1_tempt = 0;
int flag_left_Ysite_1 = 0;          //十字左边线第一个拐点是否找到的标志
int left_Ysite_2 = 0;               //十字左边线第二个拐点
int left_Ysite_2_tempt = 0;
int flag_left_Ysite_2 = 0;          //十字左边线第二个拐点是否找到的标志

int right_Ysite_1 = 0;              //十字右边线第一个拐点
int right_Ysite_1_tempt = 0;
int flag_right_Ysite_1 = 0;         //十字右边线第一个拐点是否找到的标志
int right_Ysite_2 = 0;              //十字右边线第二个拐点
int right_Ysite_2_tempt = 0;
int flag_right_Ysite_2 = 0;         //十字右边线第二个拐点是否找到的标志

//左右两边补线的时候就使用上面赛道元素识别的直线参数补线


/**********************陀螺仪的角度累加值，定义在这主要是为了两个cpu都能使用*/
float Angle_Sum_z = 0.0;              //角度值累加


float speed_init = 0;                //60
int flag_BiZhang = 0;

int flag_start_camera = 0;           //固定打角出库后开始使用摄像头循迹的标志，初始为0

/****************************模糊控制参数***********************************/
//NB NM NS ZO PS PM PB：-3 -2 -1 0 1 2 3
//使用三角形隶属度函数
#define bias_range                      //偏差范围
#define bias_dot_range                  //偏差变化率的范围
#define Kp_range                        //Kp的变化范围
#define speed_range                     //速度给定值的变化范围

#define NB -3
#define NM -2
#define NS -1
#define ZO  0
#define PS  1
#define PM  2
#define PB  3

float fuzzy_bias[7]={0.0};              //偏差的隶属度
float fuzzy_bias_dot[7]={0.0};          //偏差变化率的隶属度

float bias = 0.0;                       //偏差
float bias_last = 0.0;                  //上次的偏差
float bias_dot = 0.0;                   //偏差变化率

float dot_Kp = 0.0;                     //模糊化结果

//模糊规则
int fuzzy_rule_Kp[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
                         {PB,PB,PM,PS,PS,ZO,NS},
                         {PM,PM,PM,PS,ZO,NS,NS},
                         {PM,PM,PS,ZO,NS,NM,NM},
                         {PS,PS,ZO,NS,NS,NM,NM},
                         {PS,ZO,NS,NM,NM,NM,NB},
                         {ZO,ZO,NM,NM,NM,NB,NB}
                        };                                      //方向环Kp的模糊规则

int fuzzy_rule_speed_init[7][7];                                //速度设定值的模糊规则

/***********转弯丢线补偿函数****************************************************/
int point_1_left = 0;                //从后往前T到W的位置
int point_2_left = 0;                //从后往前W到T的位置,与point1差决定了W无边行的数量，如果超出一定阈值，则为大面积丢线，该边需要补偿数值
int flag_1_left = 0;

int point_1_right = 0;                //从后往前T到W的位置
int point_2_right = 0;                //从后往前W到T的位置,与point1差决定了W无边行的数量，如果超出一定阈值，则为大面积丢线，该边需要补偿数值
int flag_1_right = 0;


/*********************************倒车入库***************************************/

int flag_backIn = 0;                //开始倒车的标志
int flag_straightBack = 0;          //再后退一小段距离

/**********************************坡道参数***************************************/
float k_left_now = 0.0;             //左边线当前计算出的斜率
float k_left_last = 0.0;            //左边线上次计算出的斜率

float k_right_now = 0.0;             //右边线当前计算出的斜率
float k_right_last = 0.0;            //右边线上次计算出的斜率

int posi_left = 0;                  //左边线斜率突变点的纵坐标
int posi_right = 0;                 //右边线斜率突变点的纵坐标

int flag_po = 0;                        //坡道标志
int flag_po_clear_ang_y_once = 0;       //坡道俯仰角只清除一次
int flag_ang = 0;                       //经历过向下，即俯仰角为负的过程的标志


//左环岛都识别
//右边线为直道，左边呈现‘缺口-一坨黑-缺口-正常边线-近处丢边’的情况
void find_left_ring()
{
    int i= 0;
    if(flag_left_ring == 0)
    {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(flag_left_ring_point_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            flag_left_ring_point_1 = 1;                         //左环岛从下往上的第一个拐点被找到并记录位置
            left_ring_point_1 = i;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            flag_left_ring_point_2 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
            left_ring_point_2 = i-1;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            flag_left_ring_point_3 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
            left_ring_point_3 = i;
        }
        else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
            left_ring_point_4 = i-1;
            break;
        }
    }

    //左环岛的判断还得要求右边线为直道，这里借用一下赛道元素识别的参数来判断右边线是否为长直道
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

    //判断是否为左环岛,左边找到四个拐点，右边线为直道
    if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 1 && y_right_length >= 25 && Enter_Crosses_Process == 0)
    {
        flag_left_ring = 1;
        left_ring_process = 1;              //进入左环岛过程1，即左边能找到4个拐点
    }
   /* else
        flag_left_ring = 0;*/

    flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
    flag_left_ring_point_2 = 0;
    flag_left_ring_point_3 = 0;
    flag_left_ring_point_4 = 0;
    }

    //左环岛的补线处理
    if(flag_left_ring == 1)
    {
			
        if(left_ring_process == 1)                      //左环岛过程1，即左边能找到4个拐点
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
               {
                   if(flag_left_ring_point_1 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
                   {
                       flag_left_ring_point_1 = 1;                         //左环岛从下往上的第一个拐点被找到并记录位置
                       left_ring_point_1_tempt = i;
                       if(left_ring_point_1_tempt < left_ring_point_1)      //代表我从下往上的第一个拐点已经看不到了
                       {
                           left_ring_process = 2;
                           break;
                       }
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
                   {
                       flag_left_ring_point_2 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
                       left_ring_point_2_tempt = i-1;
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
                   {
                       flag_left_ring_point_3 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
                       left_ring_point_3_tempt = i;
                   }
                   else if(flag_left_ring_point_1 == 1 && flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
                   {
                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
            //开始补线，process=1表示左侧有4个拐点，需补两条线
            //遍历2和3拐点找左边界最大的位置
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

           //修改中线值
           for(int d = 59;d>=0;d--)
           {
               ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
           }
					     flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
               flag_left_ring_point_2 = 0;
               flag_left_ring_point_3 = 0;
               flag_left_ring_point_4 = 0;

        }
        else if(left_ring_process == 2)                             //左环岛过程2，即左边能找到3个拐点
        {
					left_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
//					  for(i=ImageStatus.OFFLine;i<=53;i++)
					ips200_show_int(50, 300, ImageStatus.OFFLine, 2);
					  for(i=53;i>=ImageStatus.OFFLine;i--)
               {
                   if(flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
                   {
                       flag_left_ring_point_2 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
                       left_ring_point_2_tempt = i-1;
                       if(left_ring_point_2_tempt > 17 && left_ring_point_2_tempt < 25)
                       {        
												   
//												   Angle_Sum_z=Angle_z;
												   left_ring_process = 3;
                           break;
                       }
                   }
                   else if(flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
                   {
                       flag_left_ring_point_3 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
                       left_ring_point_3_tempt = i;
//										   if(left_ring_point_3_tempt > 40 && left_ring_point_3_tempt < 48)
//                       {        
//												   
////												   Angle_Sum_z=Angle_z;
//												   left_ring_process = 3;
//                           break;
//                       }

                   }
                   else if(flag_left_ring_point_2 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
                   {
                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
//                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
//										if(flag_left_ring_point_4 == 1 && flag_left_ring_point_3 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
//                   {
//                       flag_left_ring_point_3 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
//                       left_ring_point_3_tempt = i;
////										   if(left_ring_point_3_tempt > 40 && left_ring_point_3_tempt < 48)
////                       {        
////												   
//////												   Angle_Sum_z=Angle_z;
////												   left_ring_process = 3;
////                           break;
////                       }

//                   }
//									  if(flag_left_ring_point_4 == 1 && flag_left_ring_point_3 == 1 && flag_left_ring_point_2 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
//                   {
//                       flag_left_ring_point_2 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
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
            //开始补线，补两条线
            //遍历2和3拐点找左边界最大的位置
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

            //修改中线值
            for(int d = 59;d>=0;d--)
            {
                if(d > left_ring_point_3_tempt && d< left_ring_point_4_tempt)
                    ImageDeal[d].LeftBorder = 0;
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }

        }
				
        else if(left_ring_process == 3)                             //左环岛过程3，即左边能找到1个拐点
        {
					
//					if(ImageStatus.Miss_Right_lines < 15)//取消第二次补线
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
//            if(Angle_Sum_z > 54)                      //左环岛左转一定角度从过程3进入过程4
//            {
//                left_ring_process = 4;
//            }
//            for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
//               {
//                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
//                   {
//                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
//                       left_ring_point_4_tempt = i-1;
//                       break;
//                   }
//               }
//            //此时补线还是从右边W-T的跳变开始比较好
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

//            //过程3肯定左边线基本都丢了
//            //修改中线值
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
            if(Angle_Sum_z > 200)                      //左环岛左转一定角度从过程3进入过程4
            {
                left_ring_process = 4;
            }
        }
        else if(left_ring_process == 4)   //过程4，即正常环内巡线和出环,通过陀螺仪积分角度到达一定值从3进入4
        {
            if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //过程4即环内我准备使用电磁巡线
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //同理，过程4陀螺仪累加一定角度后（即出环后），进入过程5
            if(Angle_Sum_z > 400)
            {
                left_ring_process = 5;
            }
//						for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
//               {
//                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //只有前面的拐点找到了才去找剩下的
//                   {
//                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
//							             //修改中线值
//              for(int d = 59;d>=0;d--)
//              {
//                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
//              }
//					  }
        }
        else if(left_ring_process == 5)                             //过程5和过程3的情况有点像，但补线不一样
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
              {
                  if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
                   {
                        flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
                        left_ring_point_4_tempt = i-1;
                        break;
                   }
              }

            //如果这个拐点的位置从远处到了近处一定阈值内，可以认为左环岛部分结束
            if(left_ring_point_4_tempt >= 34 && left_ring_point_4_tempt <= 41)
            {
                flag_left_ring = 0;                                     //清除左环岛标志
                left_ring_process = 0;
            }

            k_bu_2 = (float)(ImageDeal[59].LeftBorder - ImageDeal[left_ring_point_4_tempt].LeftBorder)/(59 - left_ring_point_4_tempt);
            b_bu_2 = ImageDeal[59].LeftBorder;

            for(int i = 59;i >= left_ring_point_4_tempt - 2 ;i--)
            {
                ImageDeal[i].LeftBorder = k_bu_2 * (i-59) + b_bu_2;
            }

            //修改中线值
            for(int d = 59;d>=0;d--)
            {
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
        }
        flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
        flag_left_ring_point_2 = 0;
        flag_left_ring_point_3 = 0;
        flag_left_ring_point_4 = 0;

        left_ring_point_1_tempt = 0;
        left_ring_point_2_tempt = 0;
        left_ring_point_3_tempt = 0;
        left_ring_point_4_tempt = 0;

    }
}