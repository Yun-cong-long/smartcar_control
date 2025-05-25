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
float Angle_z=90.0f;   //��ʼ�Ƕ�Ϊ90��

void gyroOffset_init(void)      /////////��������Ʈ��ʼ��
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
�������ܣ�����ƽ���˲��㷨 ������ٶ�
��ڲ�������
����  ֵ����
**************************************************************************/
void ICM20602_newValues(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_z = (float)(imu660ra_gyro_z-GyroOffset.Zdata)/14.285f;    //��ȥ��Ư������Ϊ�Ǽ��ٶ�(15.1)
	if(abs(Gyro_z)<0.1)//�Ǽ��ٶ�С��0.1ʱ  Ĭ��ΪС����ֹ  
	{
		Gyro_z = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFO[Gyro_flag-1] = ICM20602_FIFO[Gyro_flag];//FIFO ����
	}
	ICM20602_FIFO[9] = Gyro_z;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFO[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
	}
	filer_Gyro_z = sum/10;    //�Ǽ��ٶȵ�ƽ��ֵ
}		
void ICM20602_newValuesy(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_y = (float)(imu660ra_gyro_y-GyroOffset.Ydata)/14.285f;    //��ȥ��Ư������Ϊ�Ǽ��ٶ�(15.1)
	if(abs(Gyro_y)<0.1)//�Ǽ��ٶ�С��0.1ʱ  Ĭ��ΪС����ֹ  
	{
		Gyro_y = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFOy[Gyro_flag-1] = ICM20602_FIFOy[Gyro_flag];//FIFO ����
	}
	ICM20602_FIFOy[9] = Gyro_y;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFOy[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
	}
	filer_Gyro_y = sum/10;    //�Ǽ��ٶȵ�ƽ��ֵ
}
void ICM20602_newValuesx(void)
{
  float sum=0;
	 
	imu660ra_get_gyro();	
	Gyro_x = (float)(imu660ra_gyro_x-GyroOffset.Xdata)/14.285f;    //��ȥ��Ư������Ϊ�Ǽ��ٶ�(15.1)
	if(abs(Gyro_x)<0.1)//�Ǽ��ٶ�С��0.1ʱ  Ĭ��ΪС����ֹ  
	{
		Gyro_x = 0;
	}
	for(Gyro_flag = 1;Gyro_flag < 10;Gyro_flag++)
	{	
		ICM20602_FIFOx[Gyro_flag-1] = ICM20602_FIFOx[Gyro_flag];//FIFO ����
	}
	ICM20602_FIFOx[9] = Gyro_x;
	for(Gyro_flag = 0;Gyro_flag < 10;Gyro_flag++)
	{	            
		sum += ICM20602_FIFOx[Gyro_flag];//��ǰ����ĺϣ���ȡƽ��ֵ
	}
	filer_Gyro_x = sum/10;    //�Ǽ��ٶȵ�ƽ��ֵ
}
/**************************************************************************
�������ܣ��Խ��ٶȻ��� �õ��Ƕ�
��ڲ�������
����  ֵ����
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