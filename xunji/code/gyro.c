#include "zf_common_headfile.h"
#include "math.h"
#include "gyro.h"


float yaw;
float yaw_angle=90.0f;
float dt = 0.001f;    //????

float z_gyro=0;
float z_gyro_less=0;
float z_gyro_last=0;

float Z_gyro_final=0;





double LowPassFilter_Silding(double dataNewest,double dataMiddle,double dataLast)
	{
        double result;
	      result = 0.5f *dataNewest+ 0.3f *dataMiddle+0.2f *dataLast;
	      return result;
	}		
		
uint16 LowPassFilter_Average(uint16 data[],uint16 length)
{
			int32 add=0;
			int16 result;int i;
				for(i=0;i<length;i++)
				{
					add += data[i];
				}
        result=add/length;
        return result;
}
