#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 






//��������Ƶ��Ϊ10*5Hz ��Ӧ��������ʱ����ֵ
#define ARM_DELAY               10    //��ǰ����״̬�ȽϺ��õ�
#define DISARM_DELAY            10


void go_arm_check(u16*);
void TaskMotors(void* pdata);
 
 
#endif





