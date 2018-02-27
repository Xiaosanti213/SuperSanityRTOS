/**
 *
 * @file main.c
 *
 * 主循环
 *
 **/
 
 
 
 
#include <stm32f10x.h>
#include "motor.h"
#include "sensors.h"
#include <string.h>
#include <stdio.h> 
#include "attitude_estimate.h"
#include "attitude_control.h"
#include "board_config.h"

#include "ucos_ii.h"
#include "app_cfg.h"






/**
 *
 * 任务：
 * 1 传感器数据校准，读取
 * 2 姿态解算
 * 3 姿态控制
 * 4 舵量输出
 * 任务ID 优先级 栈空间大小
 * 分配栈空间
 *
 **/
#define TASK_SENSORS_PRIO 7
#define TASK_SENSORS_STACK_SIZE 128

#define TASK_ATT_EST_PRIO 6
#define TASK_ATT_EST_STACK_SIZE 128

#define TASK_ATT_CTRL_PRIO 5
#define TASK_ATT_CTRL_STACK_SIZE 128

#define TASK_MOTORS_PRIO 4
#define TASK_MOTORS_STACK_SIZE 128

OS_STK TaskSensorsStk[TASK_SENSORS_STACK_SIZE];
OS_STK TaskAttEstStk[TASK_ATT_EST_STACK_SIZE];
OS_STK TaskAttCtrlStk[TASK_ATT_CTRL_STACK_SIZE];
OS_STK TaskMotorsStk[TASK_MOTORS_STACK_SIZE];



#define TaskStkLength 64
void Task0(void* pdata)
{
	pdata = pdata;
	controller_init();//飞控板初始化
	while(1)
	{
		STATUS_LED1_ON;
		OSTimeDly(OS_TICKS_PER_SEC/4);
		STATUS_LED1_OFF;
		OSTimeDly(OS_TICKS_PER_SEC/4);
	}
}





void Task1(void* pdata)
{
	pdata = pdata;
	while(1)
	{
		STATUS_LED_ON;
		OSTimeDly(OS_TICKS_PER_SEC/3);
		STATUS_LED_OFF;
		OSTimeDly(OS_TICKS_PER_SEC/3);
	}
}







int main()
{
  // 测试操作系统配置	任务调度
	/*OS_STK TaskStk0[TaskStkLength];
	OS_STK TaskStk1[TaskStkLength];
  OSInit();
	OSTaskCreate(Task0, (void*)0, &TaskStk0[TaskStkLength-1],4);
	OSTaskCreate(Task1, (void*)0, &TaskStk1[TaskStkLength-1],5); 
  OSStart(); 
	*/
	
	
  OSInit();
	OSTaskCreate(TaskSensors, (void*)0, &TaskSensorsStk[TASK_SENSORS_STACK_SIZE-1],TASK_SENSORS_PRIO);
	OSTaskCreate(TaskAttEst, (void*)0, &TaskAttEstStk[TASK_ATT_EST_STACK_SIZE-1],TASK_ATT_EST_PRIO);
	OSTaskCreate(TaskAttCtrl, (void*)0, &TaskAttCtrlStk[TASK_ATT_CTRL_STACK_SIZE-1],TASK_ATT_CTRL_PRIO);
  OSTaskCreate(TaskMotors, (void*)0, &TaskMotorsStk[TASK_MOTORS_STACK_SIZE-1],TASK_MOTORS_PRIO); 	
  OSStart(); 
  
	

	return 0;
}




