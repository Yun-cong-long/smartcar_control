#ifndef CODE_gyro_H_
#define CODE_gyro_H_

#include "zf_common_typedef.h"

extern float yaw_angle;
extern float z_gyro;
extern float Z_gyro_final;
extern bool GyroOffset_init;



extern float yaw;
extern float yaw_add;
extern float dt ;    //采样时间



double LowPassFilter_Silding(double dataNewest,double dataMiddle,double dataLast);

uint16 LowPassFilter_Average(uint16 data[],uint16 length);

#endif /* CODE_ICM20602_H_ */