#ifndef __IMU963RA_H__
#define __IMU963RA_H__

#include "zf_common_headfile.h"

typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
	  float X_magdata;
	  float Y_magdata;
	  float Z_magdata;
} gyro_param_t;

extern bool GyroOffset_init;
extern float Gyro_z;
extern float Angle_z;
extern float Angle_x;

void gyroOffset_init(void);
void ICM20602_newValues(void);
void Get_angle(void);


#endif
