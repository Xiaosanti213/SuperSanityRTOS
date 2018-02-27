/**
 *
 * @file attitude_estimate.c
 *
 * ��̬����ͷ�ļ�
 *
 **/
#ifndef _ATTITUDE_ESTIMATE_H
#define _ATTITUDE_ESTIMATE_H

 #include "sensors.h" 
 
 
 #define DEG_TO_RAD 0.0175
 #define RAD_TO_DEG 57.3
 
 typedef struct attitude_data
 {
	 float euler_angle[3];	//�����洢roll pitch yaw������̬��
	 float angle_rate[3];   //�洢������
   
   u16 rc_command[4]; //���ڼ�����Ʋο�	 
 }ad;
 
 
 
 
 void printf_sensors_data_estimate(sd sdata);
 void TaskAttEst(void* pdata);
 
#endif
 
 
 
 
 
 
 
 
 
