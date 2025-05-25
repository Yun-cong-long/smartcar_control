#include "zf_common_headfile.h"
#include "imu963ra.h"

#define dt 0.001

gyro_param_t GyroOffset;
bool GyroOffset_init = 0;

float Angle_y=0;
float filer_Gyro_y;
float Gyro_y=0;

float Angle_x=0;
float filer_Gyro_x;
float Gyro_x=0;

float Gyro_z=0;
float filer_Gyro_z;
float ICM20602_FIFO[11];
float ICM20602_FIFOy[11];
float ICM20602_FIFOx[11];
int Gyro_flag;
float Angle_z=90.0f;   //初始角度为90度

void gyroOffset_init(void)      /////////陀螺仪零飘初始化
{
	  uint16_t i;
    GyroOffset.Zdata = 0;
    for ( i= 0; i < 1000; ++i) {
        imu963ra_get_gyro();
		    GyroOffset.Zdata += imu660ra_gyro_z;
		  	GyroOffset.Ydata += imu660ra_gyro_y;
        GyroOffset.Xdata += imu660ra_gyro_x;
        system_delay_ms(5);
    }

    GyroOffset.Zdata /= 1000;
		GyroOffset.Xdata /= 1000;
	  GyroOffset.Ydata /= 1000;
		
		GyroOffset_init=1;
		

}

/**************************************************************************
函数功能：递推平均滤波算法 处理角速度
入口参数：无
返回  值：无
**************************************************************************/
void ICM20602_newValues(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_z = (float)(imu660ra_gyro_z-GyroOffset.Zdata)/14.285f;    //减去零漂，换算为角加速度(15.1)
	if(abs(Gyro_z)<0.1)//角加速度小于0.1时  默认为小车静止  
	{
		Gyro_z = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFO[Gyro_flag-1] = ICM20602_FIFO[Gyro_flag];//FIFO 操作
	}
	ICM20602_FIFO[9] = Gyro_z;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFO[Gyro_flag];//求当前数组的合，再取平均值
	}
	filer_Gyro_z = sum/10;    //角加速度的平均值
}		
void ICM20602_newValuesy(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_y = (float)(imu660ra_gyro_y-GyroOffset.Ydata)/14.285f;    //减去零漂，换算为角加速度(15.1)
	if(abs(Gyro_y)<0.1)//角加速度小于0.1时  默认为小车静止  
	{
		Gyro_y = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFOy[Gyro_flag-1] = ICM20602_FIFOy[Gyro_flag];//FIFO 操作
	}
	ICM20602_FIFOy[9] = Gyro_y;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFOy[Gyro_flag];//求当前数组的合，再取平均值
	}
	filer_Gyro_y = sum/10;    //角加速度的平均值
}
void ICM20602_newValuesx(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_x = (float)(imu660ra_gyro_x-GyroOffset.Xdata)/14.285f;    //减去零漂，换算为角加速度(15.1)
	if(abs(Gyro_x)<0.1)//角加速度小于0.1时  默认为小车静止  
	{
		Gyro_x = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFOx[Gyro_flag-1] = ICM20602_FIFOx[Gyro_flag];//FIFO 操作
	}
	ICM20602_FIFOx[9] = Gyro_x;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFOx[Gyro_flag];//求当前数组的合，再取平均值
	}
	filer_Gyro_x = sum/10;    //角加速度的平均值
}
/**************************************************************************
函数功能：对角速度积分 得到角度
入口参数：无
返回  值：无
**************************************************************************/
void Get_angle(void)
{
   ICM20602_newValues();
	 ICM20602_newValuesy();
	 ICM20602_newValuesx();
	 Angle_z -= filer_Gyro_z*dt;
	 Angle_y -= filer_Gyro_y*dt;
	 Angle_x -= filer_Gyro_x*dt;
	 ips200_show_int(0, 284, Angle_z, 4);
}