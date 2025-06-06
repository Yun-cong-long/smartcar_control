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
int zhuan = 0;
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

/**********************右环岛***********************/
uint8 Rings_Nums_Right=0;
//环岛个数
uint8_t cnt_ring = 0;
//已跑环岛个数满的标志位
uint8_t flag_ring_man = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Compress
//  @brief          原始灰度图像压缩处理
//  @brief          作用就是将原始尺寸的灰度图像压缩成你所需要的大小，这里我是把原始80行170列的灰度图像压缩成60行80列的灰度图像。
//  @parameter      void
//  @return         void
//  @time           2022年12月18日
//  @Author
//  Sample usage:   Image_Compress();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_Compress(void)
{
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;         //根据原始的图像尺寸和你所需要的图像尺寸确定好压缩比例。
  for (i = 0; i < LCDH; i++)                            //遍历图像的每一行，从第零行到第59行。
  {
    row =i * div_h + 0.5;
    for (j = 0; j < LCDW; j++)                          //遍历图像的每一列，从第零列到第79列。
    {
      line =j * div_w + 0.5;
      Image_Use[i][j] = mt9v03x_image[row][line];       //mt9v03x_image数组里面是原始灰度图像，Image_Use数组存储的是我之后要拿去处理的图像，但依然是灰度图像哦！只是压缩了一下而已。
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Threshold
//  @brief          优化之后的的大津法。大津法就是一种能够算出一幅图像最佳的那个分割阈值的一种算法。。
//  @parameter      image  原始的灰度图像数组
//  @parameter      clo    图像的宽（图像的列）
//  @parameter      row    图像的高（图像的行）
//  @return         uint8
//  @time           2022年12月19日
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); 把存放60行80列的二维图像数组Image_Use传进来，求出这幅图像的阈值，并将这个阈值赋给Threshold。
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
  uint8* data = image;                               //定义一个指向传进来这个image数组的指针变量data
  for (i = 0; i < GrayScale; i++)                    //先把pixelCount和pixelPro两个数组元素全部赋值为0
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/
  for (i = 0; i < height; i += 1)                   //遍历图像的每一行，从第零行到第59行。
  {
    for (j = 0; j < width; j += 1)                  //遍历图像的每一列，从第零列到第79列。
    {
      pixelCount[(int)data[i * width + j]]++;       //将当前的像素点的像素值（灰度值）作为计数数组的下标。
      gray_sum += (int)data[i * width + j];         //计算整幅灰度图像的灰度值总和。
    }
  }
  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/



  /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例*************************************************/
  for (i = 0; i < GrayScale; i++)
  {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (threshold_j = 0; threshold_j < GrayScale-1; threshold_j++)
  {
    w0 += pixelPro[threshold_j];                          //求出背景部分每个灰度值的像素点所占的比例之和，即背景部分的比例。
    u0tmp += threshold_j * pixelPro[threshold_j];

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                            //背景平均灰度
    u1 = u1tmp / w1;                            //前景平均灰度
    u = u0tmp + u1tmp;                          //全局平均灰度
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
//  @brief          灰度图像二值化处理
//  @brief          整体思路就是：先调用Get_Threshold（）函数得到阈值，然后遍历原始灰度图像的每一个像素点，用每一个像素点的灰度值来跟阈值计较。
//  @brief          大于阈值的你就把它那个像素点的值赋值为1（记为白点），否则就赋值为0（记为黑点）。当然你可以把这个赋值反过来，只要你自己清楚1和0谁代表黑谁代表白就行。
//  @brief          所以我前面提到的60*80现在你们就应该明白是什么意思了吧！就是像素点嘛，一行有80个像素点，一共60行，也就是压缩后的每一幅图像有4800个像素点。
//  @parameter      void
//  @return         void
//  @Author
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BinaryImage(void)
{
  Threshold = Get_Threshold(Image_Use[0], LCDW, LCDH);      //这里是一个函数调用，通过该函数可以计算出一个效果很不错的二值化阈值。
  uint8 i, j = 0;
  for (i = 0; i < LCDH; i++)                                //遍历二维数组的每一行
  {
    for (j = 0; j < LCDW; j++)                              //遍历二维数组的每一列
    {
      if (Image_Use[i][j] > Threshold)                      //如果这个点的灰度值大于阈值Threshold
      {
          Pixle[i][j] = 1;                                  //那么这个像素点就记为白点
          Pixle_hb[i][j] = 255;
      }
      else //如果这个点的灰度值小于阈值Threshold
      {
          Pixle[i][j] = 0;                                  //那么这个像素点就记为黑点
          Pixle_hb[i][j] = 0;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Border_And_SideType
//  @brief          得到边线和边线的类型，我在这里给边线分为了三种类型，T类型、W类型和H类型。分别代表正常边线、无边边线和大跳变边线。
//  @parameter      p        指向传进来数组的一个指针变量。
//  @parameter      type     只能是L或者是R，分别代表扫左边线和扫右边线。
//  @parameter      L        扫描的区间下限 ，也就是从哪一列开始扫。
//  @parameter      H        扫描的区间上限 ，也就是一直扫到哪一列。
//  @parameter      Q        是一个结构体指针变量，自己跳过去看看这个结构体里面的成员。
//  @time           2022年12月20日
//  @Author
//  Sample usage:   Get_SideType_And_Border(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);
//  Sample usage:   从PicTemp(PicTemp是个指针，指向一个数组)的IntervalLow列开始扫，扫到IntervalHigh列，然后把得到的边线所在的列和边线类型记录到JumpPoint结构体中。
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_Border_And_SideType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)
{
  int i = 0;
  if (type == 'L')                              //如果Type是L(Left),则扫描左边线。
  {
    for (i = H; i >= L; i--)                    //从右往左扫
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //如果有黑白跳变    1是白 0是黑
      {
        Q->point = i;                           //那就把这个列记录下来作为左边线
        Q->type = 'T';                          //并且把这一行当作是正常跳变，边线类型记为T，即边线正常。
        break;                                  //找到了就跳出循环不找了
      }
      else if (i == (L + 1))                    //要是扫到最后都没找到
       {
        if (*(p + (L + H) / 2) != 0)            //并且扫描区间的中间是白像素点
        {
          Q->point = (L + H) / 2;               //那么就认为这一行的左边线是传进来扫描区间的中点。
          Q->type = 'W';                        //并且把这一行当作是非正常跳变，边线类型记为W，即无边行。
          break;                                //跳出循环不找了
        }
        else                                    //要是扫到最后都没找到，并且扫描区间的中间是黑像素点
        {
          Q->point = H;                         //那么就认为这一行的左边线是传进来扫描区间的区间上限。
          Q->type = 'H';                        //并且也把这一行当作是非正常跳变，不过边线类型记为H，即大跳变行。
          break;                                //跳出循环不找了
        }
      }
    }
  }
  else if (type == 'R')                         //如果Type是R(Right),则扫描右边线。
  {
    for (i = L; i <= H; i++)                    //从左往右扫
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //如果有黑白跳变    1是白 0是黑
      {
        Q->point = i;                           //那就把这个列记录下来作为右边线
        Q->type = 'T';                          //并且把这一行当作是正常跳变，边线类型记为T，即边线正常。
        break;                                  //找到了就跳出循环不找了
      }
      else if (i == (H - 1))                    //要是扫到最后都没找到
      {
        if (*(p + (L + H) / 2) != 0)            //并且扫描区间的中间是白像素点
        {
          Q->point = (L + H) / 2;               //那么就认为这一行的右边线是传进来扫描区间的中点。
          Q->type = 'W';                        //并且把这一行当作是非正常跳变，边线类型记为W，即无边行。
          break;
        }
        else                                    //要是扫到最后都没找到，并且扫描区间的中间是黑像素点
        {
          Q->point = L;                         //那么就认为这一行的右边线是传进来扫描区间的区间下限。
          Q->type = 'H';                        //并且也把这一行当作是非正常跳变，不过边线类型记为H，即大跳变行。
          break;                                //跳出循环不找了
        }
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BaseLine
//  @brief          用遍历的方法得到图像最底下五行（59-55行）的边线和中线信息。这五行边线和中线信息的准确度非常的重要，直接影响到整幅图像的处理结果。
//  @parameter      void
//  @time           2022年12月21日
//  @Author
//  Sample usage:   Get_BaseLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_BaseLine(void)
{
    /**************************************遍历搜索图像最底行（59行）左右边线从而确定中线的过程 ********************************************************************/
    /****************************************************Begin*****************************************************************************/

    PicTemp = Pixle[59];                                                //让PicTemp这个指针变量指向图像数组的Pixle[59]
    for (Xsite = ImageSensorMid; Xsite < 79; Xsite++)                   //假设39是中心列，从中心列开始一列一列的往右边搜索右边线
    {
      if (*(PicTemp + Xsite-1) == 1 &&*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)       //如果连续出现了两个黑点，说没找到了边线。
      {
        BottomBorderRight = Xsite;                                      //把这一列记录下来作为这一行的右边线
        break;                                                          //跳出循环
      }
      else if (Xsite == 78)                                             //如果找到了第58列都还没出现黑点，说明这一行的边线有问题。
      {
        BottomBorderRight = 79;                                         //所以我这里的处理就是，直接假设图像最右边的那一列（第79列）就是这一行的右边线。
        break;                                                          //跳出循环
      }
    }

    for (Xsite = ImageSensorMid; Xsite > 0; Xsite--)                    //假设39是中心列，从中心列开始一列一列的往左边搜索左边线
    {
      if (*(PicTemp + Xsite+1) == 1 && *(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)       //如果连续出现了两个黑点，说没找到了边线。
      {
        BottomBorderLeft = Xsite;                                       //把这一列记录下来作为这一行的左边线
        break;                                                          //跳出循环
      }
      else if (Xsite == 1)                                              //如果找到了第1列都还没出现黑点，说明这一行的边线有问题。
      {
        BottomBorderLeft = 0;                                           //所以我这里的处理就是，直接假设图像最左边的那一列（第0列）就是这一行的左边线。
        break;                                                          //跳出循环
      }
    }

    BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;           //根据左右边界计算出第59行的中线
    ImageDeal[59].LeftBorder = BottomBorderLeft;                        //把第59行的左边界存储进数组，注意看ImageDeal这个数字的下标，是不是正好对应59。
    ImageDeal[59].RightBorder = BottomBorderRight;                      //把第59行的右边界存储进数组，注意看ImageDeal这个数字的下标，是不是正好对应59。
    ImageDeal[59].Center = BottomCenter;                                //把第59行的中线存储进数组，    注意看ImageDeal这个数字的下标，是不是正好对应59。
    ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;          //把第59行的赛道宽度存储数组，注意看ImageDeal这个数字的下标，是不是正好对应59。
    ImageDeal[59].IsLeftFind = 'T';                                     //记录第59行的左边线类型为T，即正常找到左边线。
    ImageDeal[59].IsRightFind = 'T';                                    //记录第59行的右边线类型为T，即正常找到右边线。

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
//  @brief          在Get_BaseLine的基础上，针对部分特殊情况，利用一些特殊的处理算法得到剩余行的边线和中线信息。
//  @parameter      void
//  @time           2022年12月21日
//  @Author
//  Sample usage:   Get_AllLine();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Get_AllLine(void)
{
  uint8 L_Found_T  = 'F';    //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_L_line = 'F';    //找到这一帧图像的基准左斜率，为什么这里要置为F，看了下面的代码就知道了。
  uint8 R_Found_T  = 'F';    //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_R_line = 'F';    //找到这一帧图像的基准右斜率，为什么这里要置为F，看了下面的代码就知道了。
  float D_L = 0;             //左边线延长线的斜率
  float D_R = 0;             //右边线延长线的斜率
  int ytemp_W_L;             //记住首次左丢边行
  int ytemp_W_R;             //记住首次右丢边行
  int ExtenRFlag = 0;            //标志位清0
  int ExtenLFlag = 0;            //标志位清0
  ImageStatus.OFFLine=1;     //这个结构体成员我之所以在这里赋值，是因为我ImageStatus结构体里面的成员太多了，但是暂时又只用到了OFFLine，所以我在哪用到它就在哪赋值。

  //将左右两边W标志位清0
  ImageStatus.Miss_Left_lines=0;
  ImageStatus.Miss_Right_lines=0;
  ImageStatus.Miss_High_Left_lines=0;
  ImageStatus.Miss_High_Right_lines=0;
  ImageStatus.Miss_Middle_Left_lines=0;
  ImageStatus.Miss_Middle_Right_lines=0;
  ImageStatus.Miss_Low_Left_lines=0;
  ImageStatus.Miss_Low_Right_lines=0;
  ImageStatus.WhiteLine=0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)                            //前5行在Get_BaseLine()中已经处理过了，现在从55行处理到自己设定的不处理行OFFLine。
  {                                                                                  //因为太前面的图像可靠性不搞，所以OFFLine的设置很有必要，没必要一直往上扫到第0行。
    PicTemp = Pixle[Ysite];                                                          //要处理的那行
    JumpPointtypedef JumpPoint[2];                                                   // JumpPoint[0]代表左边线，JumpPoint[1]代表右边线。

  /******************************扫描本行的右边线******************************/
    IntervalLow =ImageDeal[Ysite + 1].RightBorder  -ImageScanInterval;               //从上一行的右边线加减Interval对应的列开始扫描本行，Interval一般取5，当然你为了保险起见可以把这个值改的大一点。
    IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;              //正常情况下只需要在上行边线左右5的基础上（差不多10列的这个区间）去扫线，一般就能找到本行的边线了，所以这个值其实不用太大。
    LimitL(IntervalLow);                                                             //这里就是对传给GetJumpPointFromDet()函数的扫描区间进行一个限幅操作。
    LimitH(IntervalHigh);                                                            //假如上一行的边线是第2列，那你2-5=-3，-3是不是就没有实际意义了？怎么会有-3列呢？
    Get_Border_And_SideType(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);  //扫线用的一个子函数，自己跳进去看明白逻辑。
  /******************************扫描本行的右边线******************************/

  /******************************扫描本行的左边线******************************/
    IntervalLow =ImageDeal[Ysite + 1].LeftBorder  -ImageScanInterval;                //从上一行的左边线加减Interval对应的列开始扫描本行，Interval一般取5，当然你为了保险起见可以把这个值改的大一点。
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;                //正常情况下只需要在上行边线左右5的基础上（差不多10列的这个区间）去扫线，一般就能找到本行的边线了，所以这个值其实不用太大。
    LimitL(IntervalLow);                                                             //这里就是对传给GetJumpPointFromDet()函数的扫描区间进行一个限幅操作。
    LimitH(IntervalHigh);                                                            //假如上一行的边线是第2列，那你2-5=-3，-3是不是就没有实际意义了？怎么会有-3列呢？
    Get_Border_And_SideType(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);  //扫线用的一个子函数，自己跳进去看明白逻辑。
  /******************************扫描本行的左边线******************************/
    if (JumpPoint[0].type =='W')                                                     //如果本行的左边线属于不正常跳变，即这10个点都是白点。
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                  //那么本行的左边线就采用上一行的边线。
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
    else                                                                             //如果本行的左边线属于T或者是H类别
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                              //那么扫描到的边线是多少，我就记录下来是多少。
      ImageDeal[Ysite].Left_BlackandWhite=JumpPoint[0].point;
    }

    if (JumpPoint[1].type == 'W')                                                    //如果本行的右边线属于不正常跳变，即这10个点都是白点。
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;                //那么本行的右边线就采用上一行的边线。
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
    else                                                                             //如果本行的右边线属于T或者是H类别
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                             //那么扫描到的边线是多少，我就记录下来是多少。
      ImageDeal[Ysite].Right_BlackandWhite=JumpPoint[1].point;
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                  //记录本行找到的左边线类型，是T？是W？还是H？这个类型后面是有用的，因为我还要进一步处理。
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;                                //记录本行找到的右边线类型，是T？是W？还是H？这个类型后面是有用的，因为我还要进一步处理。
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

    /************************************重新确定大跳变(即H类)的边界*************************************/

    if (( ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H'))
    {
      /**************************处理左边线的大跳变***************************/
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);Xsite <= (ImageDeal[Ysite].RightBorder - 1);Xsite++)                                                           //左右边线之间重新扫描
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
      /**************************处理左边线的大跳变***************************/


      /**************************处理右边线的大跳变***************************/
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
    /**************************处理右边线的大跳变***************************/

  /*****************************重新确定大跳变的边界******************************/



 /************************************重新确定无边行（即W类）的边界****************************************************************/
    int ysite = 0;
    int Enter_Rings_Process=0;
    int Enter_Three_Forks=0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;
    /**************************处理左边线的无边行***************************/
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
            ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //如果找到了 那么以基准行做延长线
          }
          LimitL(ImageDeal[Ysite].RightBorder);  //限幅
          LimitH(ImageDeal[Ysite].RightBorder);  //限幅
        }
        /**************************处理左边线的无边行***************************/


        /**************************处理右边线的无边行***************************/
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
            if (L_found_point > 8)              //找到基准斜率边  做延长线重新确定无边
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

          LimitL(ImageDeal[Ysite].LeftBorder);  //限幅
          LimitH(ImageDeal[Ysite].LeftBorder);  //限幅
        }
    }

    /**************************处理右边线的无边行***************************/
    /************************************重新确定无边行（即W类）的边界****************************************************************/
    /************************************都处理完之后，其他的一些数据整定操作*************************************************/
    if(Ysite<54 && Ysite>15)
    {
        if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
        {
          ImageStatus.WhiteLine++;  //要是左右都无边，丢边数+1

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
      LimitL(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitL(ImageDeal[Ysite].RightBorder);  //限幅
      LimitH(ImageDeal[Ysite].RightBorder);  //限幅

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
      /************************************都处理完之后，其他的一些数据整定操作*************************************************/
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Element_Judgment_Crosses
//  @brief          判断是否为十字路口，并对十字路口进行相应补线处理。
//  @parameter      void
//  @time           2022年12月27日
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
    if (ExtenLFlag != 'F')                                                //如果ExtenLFlag标志量不等于F，那就开始进行补线操作。
        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)        //从第54开始往上扫，一直扫到顶边下面几行。
        {
            PicTemp = Pixle[Ysite];
            if (ImageDeal[Ysite].IsLeftFind == 'W')                            //如果本行的左边线类型是W类型，也就是无边行类型。
            {
                if (ImageDeal[Ysite + 1].LeftBorder >= 70)                      //如果左边界到了第70列右边去了，那大概率就是极端情况，说明已经快寄了。
                {
                    ImageStatus.OFFLine = Ysite + 1;                              //这种情况最好的处理方法就是不处理，直接跳出循环。
                    break;
                }
                while (Ysite >= (ImageStatus.OFFLine + 4))                      //如果左边界正常，那就进入while循环卡着，直到满足循环结束条件。
                {
                    Ysite--;                                                      //行数减减
                    if (ImageDeal[Ysite].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 1].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 2].IsLeftFind == 'T'
                        && ImageDeal[Ysite - 2].LeftBorder > 0
                        && ImageDeal[Ysite - 2].LeftBorder < 70
                        )                                                         //如果扫到的无边行的上面连续三行都是正常边线
                    {
                        FTSite = Ysite - 2;                                         //那就把扫到的这一行的上面两行存入FTsite变量
                        break;                                                      //跳出while循环
                    }
                }

                DetL = ((float)(ImageDeal[FTSite].LeftBorder - ImageDeal[TFSite].LeftBorder)) / ((float)(FTSite - TFSite));  //计算左边线的补线斜率
                if (FTSite > ImageStatus.OFFLine)                              //如果FTSite存储的那一行在图像顶边OFFline的下面
                    for (ytemp = TFSite; ytemp >= FTSite; ytemp--)               //那么我就从第一次扫到的左边界的下面第二行的位置开始往上一直补线，补到FTSite行。
                    {
                        ImageDeal[ytemp].LeftBorder = (int)(DetL * ((float)(ytemp - TFSite))) + ImageDeal[TFSite].LeftBorder;     //这里就是具体的补线操作了
                    }
            }
            else                                                              //注意看清楚这个else和哪个if是一对，搞清楚逻辑关系。
                TFSite = Ysite + 2;                                             //这里为什么要Ysite+2，我没法在注释里面讲清楚，自己领会吧。
        }
    /************************************左边线的补线处理*************************************************/


    /************************************右边线的补线处理（跟左边线处理思路一模一样）注释略*************************************************/
    if (ImageStatus.WhiteLine >= 16)
        TFSite = 55;
    if (ExtenRFlag != 'F')
        for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
        {
            PicTemp = Pixle[Ysite];  //存当前行
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

                if(Ysite < (ImageStatus.OFFLine + 4))               //说明上面循环完了都没有进入while循环的if语句,即FTSite没有更新
                {
                    int i = 0;
                    int j=0;
                    for(i = Ysite;i <= (Ysite+10);i++)
                    {
                        for(j=79;j >= 43;j--)
                        {
                            //更新右边线值
                            if(Pixle[i][j] == 0 && Pixle[i][j-1] == 0 && Pixle[i][j-2] == 1 && Pixle[i][j-3] == 1)  //找到从右往左由黑到白的转点
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
                            PicTemp = Pixle[Ysite];  //存当前行
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
    /************************************右边线的补线处理（跟左边线处理思路一模一样）注释略*************************************************/


 /*****************************************所有的补线处理结束，把中线和赛道宽度信息再更新一遍******************************************************/
    for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
    {
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
        ImageDeal[Ysite].Wide = -ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder;
    }
    /*****************************************所有的补线处理结束，把中线和赛道宽度信息再更新一遍******************************************************/
}
//--------------------------------------------------------------
//  @name           Garage_Identification()
//  @brief          整个图像处理识别起跑线和车库
//  @parameter      void
//  @time           2023年3月6日
//  @Author         MRCHEN
//  Sample usage:   Garage_Identification();
//------------------------------------------------------
void Garage_Identification()
{
    int Zebra_X1 = 0, Zebra_Y1 = 0, Zebra_X2 = 0, Zebra_Y2 = 0, wid = 0;
    int Ysite_Flag[15] = { 0 }, temp = 0, Zebra_num = 0, Zebra_len = 0;
    int dir = 0;
    /*25-10行，10-70列搜斑马线*/
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
                    Zebra_num++;//黑白跳变点+1
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
    //斑马线处于状态
    if (Zebra_Cnt == 1)
    {
        if (Zebra_Flag == 0)//判断55-59行是否存在斑马线 如果存在记录此刻状态
        {
            Zebra_len = 0;
            Zebra_num = 0;
            for (Ysite = 59; Ysite >= 55; Ysite--)
            {
                for (Xsite = 10; Xsite <= 70; Xsite++)
                {
                    if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                    {
                        Zebra_num++;//黑白跳变点+1
                    }
                    if (Zebra_num >= 4)
                    {
                        Zebra_len++;//斑马线长度+1
                        Zebra_num = 0;
                        break;//进行下一行判断
                    }
                }
                if (Zebra_len >= 2)
                {
                    Zebra_Flag = 1;
                    break;
                }
            }

        }
        if (Zebra_Flag == 1)//判断55-59行斑马线不存在 标志当前已经过去车库
        {
            Zebra_len = 0;
            Zebra_num = 0;
            for (Ysite = 59; Ysite >= 55; Ysite--)
            {
                for (Xsite = 10; Xsite <= 70; Xsite++)
                {
                    if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                    {
                        Zebra_num++;//黑白跳变点+1
                    }
                    if (Zebra_num >= 4)
                    {
                        Zebra_len++;//斑马线长度+1
                        Zebra_num = 0;
                        break;//进行下一行判断
                    }
                }
            }
            if (Zebra_len == 0)  //此时无黑白跳变
            {
                Zebra_Flag = 2;
            }
        }
        if (Zebra_Flag == 2)//标志位清零
        {
            Zebra_Flag = 0;
            Zebra_Cnt = 0;
            //flag_stop_Car = 1;
        }
    }
    /*确定斑马线y轴位置*/
    /*腐蚀斑马线*/
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
                temp = Ysite_Flag[i + 1];//保存最下端斑马线
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



//车库和斑马线的识别处理
//识别后固定打角
void find_zebra()
{
    uint8 black_white_num = 0;                  //黑白跳变点数量
    int i =0;
    int j =0;

    for(i= 43;i <=53 ;i++)                          //行遍历
    {
        for(j=10 ;j <70 ;j++)                      //列遍历
        {
            if((Pixle[i][j] == 0 && Pixle[i][j+1] == 1) || (Pixle[i][j] == 1 && Pixle[i][j+1] == 0))
            {
                black_white_num++;
            }
        }
    }

    if(black_white_num >= 64 && flag_out == 0 && flag_left_ring == 0 && flag_right_ring == 0)             //小车在断路中好像也能识别为斑马线
    {
        flag_zebra = 1;
    }
		else
		{
			flag_zebra = 0;
		}
//		ips200_show_int(100, 300, flag_zebra , 2);
}

//小车出赛道停车
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
//     if(((num>210) && flag_openRoad == 0 && flag_BiZhang == 0 && flag_start_camera == 1 && flag_zebra == 0 && flag_po == 0 && Enter_Rings_Process == 0 ))      //限制小陀螺的出现
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


//赛道元素识别，主要是弯道和直道
//直道的判断：从下往上找到第一次出现先‘W’再‘T’的那一行（即图像近距离第一次丢边的地方），-3和-6取两个点，计算该直线方程，然后向上拟合出所有边线数据，与原来的边线数据进行比较，若超出某阈值，理解为直线不再拟合该边线，记录该位置，计算一下拟合部分的长度
//              左右边线均按此方法，计算后，取左右最大的那个拟合直线段长度，若大于某阈值，可以认为是直线段；，当然，拟合直线必须要有一定斜率，否则可能是大弯造成的严重丢边
void road_type()
{
    /**********************直道的识别*************************/
    /****************************右边提取****************************/
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

    /****************************左边提取****************************/
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

    if(y_right_length > y_left_length)                                                  //选取左右直道的最大值
    {
        Str_len = y_right_length;
    }
    else
        Str_len = y_left_length;

    if(Str_len >= 30  && a1 < -0.5 && a2 > 0.5)                 //直道的判断
    {
        flag_Straight = 1;
    }
    else
        flag_Straight = 0;
//    ips200_show_int(50, 284, flag_Straight, 3);
}

//赛道元素识别，主要是弯道和直道
//直道的判断：从下往上找到第一次出现先‘W’再‘T’的那一行（即图像近距离第一次丢边的地方），-3和-6取两个点，计算该直线方程，然后向上拟合出所有边线数据，与原来的边线数据进行比较，若超出某阈值，理解为直线不再拟合该边线，记录该位置，计算一下拟合部分的长度
//              左右边线均按此方法，计算后，取左右最大的那个拟合直线段长度，若大于某阈值，可以认为是直线段；，当然，拟合直线必须要有一定斜率，否则可能是大弯造成的严重丢边
void road_type_2()
{
    /**********************直道的识别*************************/
    /****************************右边提取****************************/
    // for(y_right_road_tempt = 55;y_right_road_tempt > ImageStatus.OFFLine;y_right_road_tempt--)
    for(y_right_road_tempt = ImageStatus.OFFLine+5 ; y_right_road_tempt < 55 ; y_right_road_tempt++)
    {
        if(ImageDeal[y_right_road_tempt].IsRightFind == 'W' && ImageDeal[y_right_road_tempt-1].IsRightFind == 'T')
        {
            break; //修改：从上往下找，以防在没有丢边的情况下会一直找到图像顶部。如果一直没找到会有线可循。下方左边线同理
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

    /****************************左边提取****************************/
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

    if(y_right_length > y_left_length)                         //选取左右直道的最大值
    {
        Str_len = y_right_length;
    }
    else
        Str_len = y_left_length;

    if(Str_len >= 30  /*&& a1 < -0.5 && a2 > 0.5*/)        //修改：直道的判断，在这里修改阈值，如果你觉得条件苛刻，你可以将30减小。我还将后面斜率判断注释了，因为我们有时候出入赛道可能看到直道的斜率会很大
    {
        flag_Straight = 1;
    }
    else
        flag_Straight = 0;
		ips200_show_int(50, 284, flag_Straight, 3);

}

//左环岛都识别
void find_left_ring()
{
    int i,j;
    int r_st_up,r_st_dw;
    int st_cha,st_num; // 判断直道
    int w_num; // 左丢边计数
    int lower_point; //最底下的拐点
    int longer_num; // 出环岛检测
    if(Enter_Crosses_Process == 0) // 不能是十字路口，优先级要比十字路口低
    {
        if(flag_left_ring == 0)
        {
            r_st_dw = ImageDeal[54].RightBorder;
            r_st_up = ImageDeal[50].RightBorder;
            int k = (r_st_up - r_st_dw)/(50-54)
            for(i=55;i>=ImageStatus.OFFLine;i--) // 寻找特征，判断是否是环岛
            {
                // 先看右边是否是直道
                st_cha = ImageDeal[i].RightBorer - (k*(i-54)+r_st_dw)
                if(st_cha < 3 && st_cha > -3)
                {
                    st_num++;
                }
                else{
                    break;
                }
            }
            for(i=55;i>=ImageStatus.OFFLine;i--) // 左边丢边行数
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
        if(flag_left_ring == 1) // 找到环岛进入下一个阶段
        {
            for(i=55;i>=ImageStatus.OFFLine+1;i--) // 记录最下面的拐点
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
        if(flag_left_ring == 2) // 小车以omu（电阻单位符号）的形状入环
        {
            // 电控控制小车左转,然后寻弯道
            
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
        if(flag_left_ring == 3) // 出环岛
        {
            // 电控控制小车出环，即左转出环
            // 电控写小车出环条件，用陀螺仪记录航向角左转一定程度，使flag_left_ring == 0
        }
    }
}

//右环岛
void find_right_ring()
{
}

//十字的识别和处理
//主要是左右两边都有一个很大的空白区，都会出现T-W-T的情况
float Angle_Sum_y = 0.0;              //y轴角度值累加，判断坡道
void find_cross()
{
    //十字的识别，一定会经过左右都有一段空白区的阶段，即左右均有两个拐点
    //识别左右两边各自两个拐点位置
    int i =0;
    if(flag_cross == 0)
    {
    for(i = 53;i>=ImageStatus.OFFLine;i--)
    {
        //左边
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
        //右边
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

    //在已经识别到十字的情况下进行补线处理
    else if(flag_cross == 1)
    {
        //需要先判断左右有几个拐点，可能有两个，可能有一个
        for(i = 53;i>=ImageStatus.OFFLine;i--)
        {
            //左边
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
            //右边
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

        if(left_Ysite_1_tempt >= left_Ysite_1 && left_Ysite_2_tempt>=left_Ysite_2 && flag_left_Ysite_1 == 1 && flag_left_Ysite_2 == 1)              //说明左边有两个拐点
        {
            //补线
            a1 = (float)(ImageDeal[left_Ysite_1_tempt].LeftBorder - ImageDeal[left_Ysite_2_tempt].LeftBorder)/(left_Ysite_1_tempt - left_Ysite_2_tempt);
            BB1 = ImageDeal[left_Ysite_1_tempt].LeftBorder - a1*(left_Ysite_1_tempt - 59);
            for(int j = left_Ysite_1_tempt ;j >= left_Ysite_2_tempt;j--)
            {
                ImageDeal[j].LeftBorder = a1*(j-59) + BB1;
            }
        }
        else        //左边只有一个拐点
        {
            //重新找这个拐点
            for(i = 53;i>=ImageStatus.OFFLine;i--)
            {
                if(flag_left_Ysite_2 == 0 && ImageDeal[i].IsLeftFind =='T' && ImageDeal[i+1].IsLeftFind == 'W')
                {
                    flag_left_Ysite_2 = 1;
                    left_Ysite_2_tempt = i;
                    break;
                }
                //补线
                a1 = (float)(ImageDeal[59].LeftBorder - ImageDeal[left_Ysite_2_tempt].LeftBorder)/(59 - left_Ysite_2_tempt);
                BB1 = ImageDeal[59].LeftBorder;
                for(int j = 59 ;j >= left_Ysite_2_tempt;j--)
                {
                    ImageDeal[j].LeftBorder = a1*(j-59) + BB1;
                }
            }
        }

        //右边补线
        if(right_Ysite_1_tempt >= right_Ysite_1 && right_Ysite_2_tempt>=right_Ysite_2 && flag_right_Ysite_1 == 1 && flag_right_Ysite_2 == 1)              //说明右边有两个拐点
        {
            //补线
            a2 = (float)(ImageDeal[right_Ysite_1_tempt].RightBorder - ImageDeal[right_Ysite_2_tempt].RightBorder)/(right_Ysite_1_tempt - right_Ysite_2_tempt);
            BB2 = ImageDeal[right_Ysite_1_tempt].RightBorder - a2*(right_Ysite_1_tempt - 59);
            for(int j = right_Ysite_1_tempt ;j >= right_Ysite_2_tempt;j--)
            {
                ImageDeal[j].RightBorder = a2*(j-59) + BB2;
            }
        }
        else        //右边只有一个拐点
        {
            //重新找这个拐点
            for(i = 53;i>=ImageStatus.OFFLine;i--)
            {
                if(flag_right_Ysite_2 == 0 && ImageDeal[i].RightBorder =='T' && ImageDeal[i+1].RightBorder == 'W')
                {
                    flag_right_Ysite_2 = 1;
                    right_Ysite_2_tempt = i;
                    break;
                }
                //补线
                a2 = (float)(ImageDeal[59].RightBorder - ImageDeal[right_Ysite_2_tempt].RightBorder)/(59 - right_Ysite_2_tempt);
                BB2 = ImageDeal[59].RightBorder;
                for(int j = 59 ;j >= right_Ysite_2_tempt;j--)
                {
                    ImageDeal[j].RightBorder = a2*(j-59) + BB2;
                }
            }
        }

        //补线完成需要更新中线值
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
    //清除以免影响下一次程序进入该函数时条件误判
    flag_left_Ysite_1 = 0;
    flag_left_Ysite_2 = 0;
    flag_right_Ysite_1 = 0;
    flag_right_Ysite_2 = 0;
}



//转弯丢线补偿函数
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

    if(point_1_left - point_2_left > 30)            //说明左边有大量的无边行W，即丢边，左边补值是减小
    {
        for(int j =point_1_left;j >= point_2_left;j--)
        {
            ImageDeal[j].LeftBorder = -8;
        }
    }

    if(point_1_right - point_2_right > 30)            //说明右边有大量的无边行W，即丢边，右边补值是增加
    {
        for(int j =point_1_right;j >= point_2_right;j--)
        {
            ImageDeal[j].RightBorder = 88;
        }
    }

    flag_1_left = 0;
    flag_1_right = 0;
}

//斑马线腐蚀函数
void fu_shi()
{
    if(flag_zebra == 1)
    {
        int i =0;
        for(i= 30;i <=59 ;i++)                  //若识别到斑马线，我把这些行的中线全弄为理想中线
        {
            ImageDeal[i].Center = 39;
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Slope_cal
//  @brief          计算斜率，用于进行转向之类操作
//  @parameter      void
//  @time           2024年2月24日
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
		slope=tem*(180/3.14);//控制角度介于正负53.13度
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
		slope=tem*(180/3.14);//控制角度介于正负53.13度
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
//  @brief          在屏幕上显示中线和左右边线，可以注释掉
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
//  @brief          判断是否进环
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void ringfig()
{
		if(flag_crossring==1 ||Enter_Rings_Flag>0)
			is_roundabout=1;
		else
			is_roundabout=0;

}

//左环岛都识别
//右边线为直道，左边呈现‘缺口-一坨黑-缺口-正常边线-近处丢边’的情况
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
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //左环岛从下往上的第一个拐点被找到并记录位置
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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

    //左环岛的补线处理
    if(flag_left_ring == 1)
    {
			zhuan++;
        if(left_ring_process == 1)                      //左环岛过程1，即左边能找到4个拐点
        {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //左环岛从下往上的第一个拐点被找到并记录位置
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
            //开始补线，process=1表示左侧有4个拐点，需补两条线
            //遍历2和3拐点找左边界最大的位置
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
		if (num == 3 && zhuan > 100)
		{
			zhuan = 0;
			left_ring_process = 2;
		}
		flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
      if(left_ring_process == 2)                             //左环岛过程2，即左边能找到3个拐点
        {
//					left_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')
        {
            biao11 = 1;                         //左环岛从下往上的第一个拐点被找到并记录位置
            left11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //左环岛从下往上的第2个拐点被找到并记录位置
            left12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsLeftFind == 'T' && ImageDeal[i-1].IsLeftFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //左环岛从下往上的第3个拐点被找到并记录位置
            left13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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

            //开始补线，补两条线
            //遍历2和3拐点找左边界最大的位置
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

            //修改中线值
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

            //修改中线值
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
		flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
     if(left_ring_process == 3)                             //左环岛过程3，即左边能找到1个拐点
        {

            if(flag_clear_process_3 == 0)
            {
                Angle_Sum_z = 0.0;
                flag_clear_process_3 = 1;
            }
						Angle_Sum_z++;
            if(Angle_Sum_z > 200)                      //左环岛左转一定角度从过程3进入过程4
            {
                left_ring_process = 0;
            }
//						for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
//              {
//                  if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsLeftFind == 'W' && ImageDeal[i-1].IsLeftFind == 'T')                    //只有前面的拐点找到了才去找剩下的
//                   {
//                        flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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

//            //修改中线值
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
//		flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
      if(left_ring_process == 4)   //过程4，即正常环内巡线和出环,通过陀螺仪积分角度到达一定值从3进入4
        {
					if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //过程4即环内我准备使用电磁巡线
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //同理，过程4陀螺仪累加一定角度后（即出环后），进入过程5
            if(Angle_Sum_z > 1000)
            {
                left_ring_process = 0;
            }
						for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
               {
                   if(flag_left_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //只有前面的拐点找到了才去找剩下的
                   {
                       flag_left_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
							             //修改中线值
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
		flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
        if(left_ring_process == 5)                             //过程5和过程3的情况有点像，但补线不一样
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
            if(left_ring_point_4_tempt >= 34 && left_ring_point_4_tempt <= 38)
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
		flag_left_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
//右环岛都识别
//左边线为直道，左边呈现‘缺口-一坨黑-缺口-正常边线-近处丢边’的情况
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
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //右环岛从下往上的第一个拐点被找到并记录位置
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //右环岛从下往上的第2个拐点被找到并记录位置
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //右环岛从下往上的第3个拐点被找到并记录位置
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //右环岛从下往上的第4个拐点被找到并记录位置
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

    //右环岛的判断还得要求左边线为直道，这里借用一下赛道元素识别的参数来判断左边线是否为长直道
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

    //判断是否为右环岛,右边找到四个拐点，左边线为直道
    if(flag_right_ring_point_1 == 1 && flag_right_ring_point_2 == 1 && flag_right_ring_point_3 == 1 && flag_right_ring_point_4 == 1 && y_left_length >= 25 && Enter_Crosses_Process == 0)
    {
        flag_right_ring = 1;
        right_ring_process = 1;              //进入右环岛过程1，即右边能找到4个拐点
    }
   /* else
        flag_left_ring = 0;*/

    flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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

    //右环岛的补线处理
    if(flag_right_ring == 1)
    {
			
        if(right_ring_process == 1)                      //右环岛过程1，即左右边能找到4个拐点
        {
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //右环岛从下往上的第一个拐点被找到并记录位置
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //右环岛从下往上的第2个拐点被找到并记录位置
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //右环岛从下往上的第3个拐点被找到并记录位置
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //右环岛从下往上的第4个拐点被找到并记录位置
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
            //开始补线，process=1表示左侧有4个拐点，需补两条线
            //遍历2和3拐点找左边界最大的位置
		if (num == 4)
		{
            //开始补线，process=1表示右侧有4个拐点，需补两条线
            //遍历2和3拐点找右边界最大的位置
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

           //修改中线值
           for(int d = 59;d>=0;d--)
           {
               ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
           }
					     flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
               flag_right_ring_point_2 = 0;
               flag_right_ring_point_3 = 0;
               flag_right_ring_point_4 = 0;

        }
		if (num == 3)
		{
			right_ring_process = 2;
		}
		flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
      if(right_ring_process == 2)                             //左环岛过程2，即左边能找到3个拐点
        {
//					right_ring_point_2_tempt=53;
//					left_ring_point_3_tempt=53;
//					left_ring_point_4_tempt=53;
    for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
    {
        if(biao11 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')
        {
            biao11 = 1;                         //右环岛从下往上的第一个拐点被找到并记录位置
            right11 = i;
					  num1++;
        }
        if(biao11 == 1 && biao12 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有第一个找到才接着去找剩下两个点
        {
            biao12 = 1;                         //右环岛从下往上的第2个拐点被找到并记录位置
            right12 = i-1;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 0 && ImageDeal[i].IsRightFind == 'T' && ImageDeal[i-1].IsRightFind == 'W')                    //只有前面的拐点找到了才去找剩下的
        {
            biao13 = 1;                         //右环岛从下往上的第3个拐点被找到并记录位置
            right13 = i;
					num1++;
        }
        if(biao11 == 1 && biao12 == 1 && biao13 == 1 && biao14 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
        {
            biao14 = 1;                         //右环岛从下往上的第4个拐点被找到并记录位置
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

            //开始补线，补两条线
            //遍历2和3拐点找左边界最大的位置
		if (num == 3)
		{
            //开始补线，补两条线
            //遍历2和3拐点找右边界最大的位置
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

            //修改中线值
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

            //修改中线值
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
		flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
     if(right_ring_process == 3)                             //左环岛过程3，即左边能找到1个拐点
        {

            if(flag_clear_process_3_right == 0)
            {
                Angle_Sum_z = 0.0;
                flag_clear_process_3_right = 1;
            }
						Angle_Sum_z++;
            if(Angle_Sum_z > 200)                      //左环岛左转一定角度从过程3进入过程4
            {
                right_ring_process = 0;
            }
					
//							for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
//              {
//                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
//                   {
//                        flag_right_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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

//            //修改中线值
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
		flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
      if(right_ring_process == 4)   //过程4，即正常环内巡线和出环,通过陀螺仪积分角度到达一定值从3进入4
        {
					if(flag_clear_process_4 == 0)
            {
                Angle_Sum_z = 0;                                   //过程4即环内我准备使用电磁巡线
                flag_clear_process_4 = 1;
            }
            Angle_Sum_z++;
            //同理，过程4陀螺仪累加一定角度后（即出环后），进入过程5
            if(Angle_Sum_z > 1000)
            {
//                flag_right_ring = 0;                                     //清除右环岛标志
                right_ring_process = 0;
            }
					
						for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
              {
                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
                   {
                        flag_right_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
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
		flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
        if(right_ring_process == 5)                             //过程5和过程3的情况有点像，但补线不一样
        {
            for(i=53;i>=ImageStatus.OFFLine;i--)                       //后五行因为之前扫线程序的原因，标识符均为‘T’，会影响我之后的判断，故跳过他们开始循环
              {
                  if(flag_right_ring_point_4 == 0 && ImageDeal[i].IsRightFind == 'W' && ImageDeal[i-1].IsRightFind == 'T')                    //只有前面的拐点找到了才去找剩下的
                   {
                        flag_right_ring_point_4 = 1;                         //左环岛从下往上的第4个拐点被找到并记录位置
                        right_ring_point_4_tempt = i-1;
                        break;
                   }
              }

            //如果这个拐点的位置从远处到了近处一定阈值内，可以认为左环岛部分结束
            if(right_ring_point_4_tempt >= 34 && right_ring_point_4_tempt <= 38)
            {
                flag_right_ring = 0;                                     //清除左环岛标志
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

            //修改中线值
            for(int d = 59;d>=0;d--)
            {
                ImageDeal[d].Center = (ImageDeal[d].LeftBorder + ImageDeal[d].RightBorder)/2;
            }
					  }
		flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
        flag_right_ring_point_1 = 0;             //清除以免影响下一次程序进入该函数时条件误判
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
//图像处理的主函数，将函数放在这个函数里面才能执行
//----------------------------------------------------------------------------------------------------------------------------------------------
int Image_Process(void)
{
		       	 Image_Compress();           //图像压缩，把原始的120*188的图像压缩成60*80的。保存在全局变量Image_Use中
             Get_BinaryImage();          //图像二值化处理，把采集到的原始灰度图像变成二值化图像。存放在Pixle_hb中80*60
             Get_BaseLine();             //优化之后的搜线算法：得到一副图像的基础边线，也就是最底下五行的边线信息，用来后续处理。
             Get_AllLine();              //优化之后的搜线算法：得到一副图像的全部边线和中线。
             road_type_2();                   //判断赛道类型
	           if (faajishu > 0)
						 {
                find_zebra();                //识别斑马线
						 }
             Element_Judgment_Crosses(); //判断是否为十字路口并对其进行处理

             find_openRoad();               //识别是否为断路
             bu();                  //转弯丢线补偿
             stop_car();                //出赛道停车检测
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

