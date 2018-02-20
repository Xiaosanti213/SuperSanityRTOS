/**
 *
 * @file main.c
 *
 * ��ѭ��
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



/*
#define TaskStkLength 64
void Task0(void* pdata)
{
	pdata = pdata;
	controller_init();//�ɿذ��ʼ��
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





*/





int main()
{
  // ���Բ���ϵͳ����	�������
	/*OS_STK TaskStk0[TaskStkLength];
	OS_STK TaskStk1[TaskStkLength];
  OSInit();
	OSTaskCreate(Task0, (void*)0, &TaskStk0[TaskStkLength-1],4);
	OSTaskCreate(Task1, (void*)0, &TaskStk1[TaskStkLength-1],5); 
  OSStart(); 
	*/
	
	
	
	//�ϵ�֮���뱣֤�����������
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	sensors_init();
  sensors_calibration(&calib_data, &sensors_data);//һ��Ҫ��ֹˮƽ�������ᣬҡ���������ϵ�
	while(1)
	{
		get_sensors_data(&sensors_data, &calib_data);
		attitude_estimate(&attitude_data, &sensors_data);
		set_reference(sensors_data.rc_command, reference); //ҡ�˶�������ת��Ϊ���Ʋο�
		attitude_control(attitude_data, reference, output);
		
		mix_table(output, &sensors_data); 
		go_arm_check(sensors_data.rc_command);//����֮ǰ��Ӧ��������಻Ӧ�ж���
		write_mini_motors(sensors_data.motor);
	}



	return 0;
}




