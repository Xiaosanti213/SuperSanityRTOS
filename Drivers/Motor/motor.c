/**
 *
 * @file motor.c
 *
 * �Կ��ı�������ַ���ģʽ����
 *
 **/
 
#include "motor.h"
#include "board_config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "sensors.h"
#include "attitude_control.h" 
#include <stdio.h>
#include "ucos_ii.h"


static void mix_table(od* out_data);
static u8 arm_motors_flag = 0; 						// Ĭ������״̬
static void write_mini_motors(u16* motor);// ���ı����
extern OS_EVENT* mbox_motors;							// �������ⲿattitude_control.c





void TaskMotors(void* pdata)
{
	INT8U* err;
	od* output; 
	pdata = pdata;
	mbox_motors = OSMboxCreate(NULL);
  controller_init();
	
	while(1)
	{
		/* 1 20ms���ڵȴ���̬������Ϣ
		 * 2 ����
		 * 3 �������
		 */ 
		output = OSMboxPend(mbox_motors, OS_TICKS_PER_SEC/50uL, err);
		//TODO:��������ʱ
		mix_table(output);
		write_mini_motors(output->motors); 
	}
}








/**
 *
 * ����: write_mini_motors
 *
 * ���������ı����ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void write_mini_motors(u16* motor)
 {
	 if(arm_motors_flag)
	 {
	 	// ����4�����ı���������
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
	 }
	 else//ǿ�����0
	 {
		TIM_SetCompare1(PWM_TIM, 0);
	  TIM_SetCompare2(PWM_TIM, 0);
	  TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, 0);
	 }

 }
 
 
 
 
 
 /**
 *
 * ����: go_arm_check()
 *
 * �����������������״̬
 *
 */ 
void go_arm_check(u16* rc_command)
{
	static int16_t arming_counter;
	u16 temp;
	OS_CPU_SR cpu_sr=0;
	// ����ͨ���ĸ����������ж� ������Χ
	// ���ڿ��ı������˵���Ѿ�ӳ�䵽[0-2000]
	OS_ENTER_CRITICAL();
	if (rc_command[2] > 30) {
  // ǰ������λ����͵㣬����ֱ���˳�
      arming_counter = 0;
      return;  
   } 
	 temp = rc_command[3];
	 // ȡ���������
	 if (temp > 1970)
	 {
			if(arming_counter < ARM_DELAY)
			{
				arming_counter ++;
				//printf("********************************************\n");
				//printf("��ǰҡ��״̬��������ֵ��%d\n", arming_counter);
			}
			else if(arming_counter == ARM_DELAY)
			{
				arming_counter = 0;
				// ����������
				STATUS_LED_ON;
				// ָʾ�Ƶ���
				arm_motors_flag = 1;
				// ��������ź�
			}
	 }
	 else if(temp < 30)
	 // ���ͨѶ״��������������ź�һֱ��0����֤ARM_MOTORS==0
	 {
			if(arming_counter < DISARM_DELAY)
			{
				arming_counter ++; 
				//printf("********************************************\n");
				//printf("��ǰҡ��״̬��������ֵ��%d\n", arming_counter);
			}
			else if(arming_counter == DISARM_DELAY)
			{
			  arming_counter = 0;
				// ����������
				STATUS_LED_OFF;
				// ����״ָ̬ʾ��Ϩ��
				arm_motors_flag = 0;
				// ��ֹ����ź�
			}
	 }
	 OS_EXIT_CRITICAL();
}


 
 
 
 
 
 
 /**
 *
 * ����: mix_table()
 *
 * ��������������̬������Ӧ
 *
 */ 
void mix_table(od* out_data)
 {
	 u8 i=0;
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  MIX(X,Y,Z)  (u16)(out_data->output[0]*X + out_data->output[1]*Y + out_data->output[2]*Z + out_data->output[3])
	 // ����X������
	 out_data->motors[0] = MIX(-0.02,+0.02,-0.02);  //��ǰ 1�ŵ�� 
	 out_data->motors[1] = MIX(+0.02,+0.02,+0.02);  //��ǰ 2�ŵ��
	 out_data->motors[2] = MIX(+0.02,-0.02,-0.02);  //��� 3�ŵ��
	 out_data->motors[3] = MIX(-0.02,-0.02,+0.02);  //�Һ� 4�ŵ��
	  
	 for(i = 0; i<4; i++)
			{
				if(out_data->motors[i]>2000)
					 out_data->motors[i] = 2000;
			  else;
			}
	/*		
	 printf("1st : %d   \n", out_data->motors[0]);
	 printf("2nd : %d   \n", out_data->motors[1]);
	 printf("3rd : %d   \n", out_data->motors[2]);
	 printf("4th : %d   \n", out_data->motors[3]);
	 */
}
 
 
 
 
 
 
 
 
 
 
 

