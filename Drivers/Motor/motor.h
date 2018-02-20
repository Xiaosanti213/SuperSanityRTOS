#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 

//ͨ�����
#define ROLL			0
#define PITCH			1
#define THROTTLE  2
#define YAW				3


//��������Ƶ��Ϊ10*5Hz ��Ӧ��������ʱ����ֵ
#define ARM_DELAY               10    //��ǰ����״̬�ȽϺ��õ�
#define DISARM_DELAY            10




void write_mini_motors(u16* motor);// ���ı����
void write_motors(u16* motor);// ��ͨ��ˢ���
void mix_table(int16_t*, sd*);
 
void go_arm_check(u16*);

 
 
#endif





