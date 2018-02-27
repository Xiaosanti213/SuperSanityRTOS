#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 






//函数调用频率为10*5Hz 相应计算下面时间阈值
#define ARM_DELAY               10    //当前测试状态比较好用的
#define DISARM_DELAY            10


void go_arm_check(u16*);
void TaskMotors(void* pdata);
 
 
#endif





