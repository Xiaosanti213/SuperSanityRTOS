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

#include <stdio.h>

 
static int  arm_motors_flag = 0; //Ĭ������״̬
static void delay_ms(u16); 

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
 * ����: write_motors
 *
 * ���������ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void write_motors(u16* motor)
 {
	 
	  // ����4����������
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 
/**
 *
 * ����: set_mini_motors
 *
 * �����������ʼ����ʱ
 *
 */  
 void init_recog_motors(void)
 {
	  u8 i = 0;
	  u16 motor[4];
	 for (; i < 4; i++)
	 {
			motor[i] = 1050;
	 }
	  // ��ʼ���ĸ����
	  write_motors(motor);
	  delay_ms(5000);//��ʱ5s
	 
 }
 
 
 
 /**
 *
 * ����: delay_ms()
 *
 * ���������
 *
 */  
 void delay_ms(u16 ms)
 {
	 u16 temp = 10000;
	 //������Ƶ��ʵ����ȷ��
		for (; ms; ms--)
	 {
			for (; temp; temp--);
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
	// ����ͨ���ĸ����������ж� ������Χ
	// ���ڿ��ı������˵���Ѿ�ӳ�䵽[0-2000]
	
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
}





 
 
 
 
 
 
 
 /**
 *
 * ����: mix_table()
 *
 * ��������������̬������Ӧ
 *
 */ 
void mix_table(int16_t* output, sd* s_data)//struct����ʡ��
 {
	 u8 i=0;
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  PIDMIX(X,Y,Z)  output[0]*X + output[1]*Y + output[2]*Z + output[3];
	 // ����X������
	 s_data->motor[0] = PIDMIX(-0.02,+0.02,-0.02);  //��ǰ 1�ŵ�� 
	 s_data->motor[1] = PIDMIX(+0.02,+0.02,+0.02);  //��ǰ 2�ŵ��
	 s_data->motor[2] = PIDMIX(+0.02,-0.02,-0.02);  //��� 3�ŵ��
	 s_data->motor[3] = PIDMIX(-0.02,-0.02,+0.02);  //�Һ� 4�ŵ��
	 
	 for(i = 0; i<4; i++)
			{
				if(s_data->motor[i]>2000)
					 s_data->motor[i] = 2000;
			  else;
			}
			
	 printf("1st : %d   \n", s_data->motor[0]);
	 printf("2nd : %d   \n", s_data->motor[1]);
	 printf("3rd : %d   \n", s_data->motor[2]);
	 printf("4th : %d   \n", s_data->motor[3]);
	 
}
 
 
 
 
 
 
 
 
 
 
 

