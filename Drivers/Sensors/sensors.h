#ifndef _SENSORS_H
#define _SENSORS_H


#include <stm32f10x.h>
#include "ucos_ii.h" 




typedef struct sensors_data
{
  u16 rc_command[4];//结构体当中不能赋初值
	
	float acc[3];
  float gyro[3];
	float temp[1];//当前数据处理还不需要MPU6050和MS5611的温度数据
	float mag[3]; 
	float press;
	
}sd;




typedef struct sensors_calib
{
	float acc_calib[3];
	float gyro_calib[3];
  float	rc_calib[4]; 
}sc; 





void sensors_init(void);
void delay_approx(u16);
void TaskSensors(void* pdata);



#endif



