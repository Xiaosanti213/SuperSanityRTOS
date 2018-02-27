/**
 *
 * @file motor.c
 *
 * 对空心杯电机几种飞行模式配置
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
static u8 arm_motors_flag = 0; 						// 默认上锁状态
static void write_mini_motors(u16* motor);// 空心杯电机
extern OS_EVENT* mbox_motors;							// 定义于外部attitude_control.c





void TaskMotors(void* pdata)
{
	INT8U* err;
	od* output; 
	pdata = pdata;
	mbox_motors = OSMboxCreate(NULL);
  controller_init();
	
	while(1)
	{
		/* 1 20ms周期等待姿态解算消息
		 * 2 处理
		 * 3 控制输出
		 */ 
		output = OSMboxPend(mbox_motors, OS_TICKS_PER_SEC/50uL, err);
		//TODO:处理出错或超时
		mix_table(output);
		write_mini_motors(output->motors); 
	}
}








/**
 *
 * 名称: write_mini_motors
 *
 * 描述：空心杯电机直接设置四个电机比较寄存器的值
 *
 */ 
 void write_mini_motors(u16* motor)
 {
	 if(arm_motors_flag)
	 {
	 	// 设置4个空心杯电机输出量
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
	 }
	 else//强制输出0
	 {
		TIM_SetCompare1(PWM_TIM, 0);
	  TIM_SetCompare2(PWM_TIM, 0);
	  TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, 0);
	 }

 }
 
 
 
 
 
 /**
 *
 * 名称: go_arm_check()
 *
 * 描述：电机解锁怠速状态
 *
 */ 
void go_arm_check(u16* rc_command)
{
	static int16_t arming_counter;
	u16 temp;
	OS_CPU_SR cpu_sr=0;
	// 下面通过四个舵量进行判断 舵量范围
	// 对于空心杯电机来说，已经映射到[0-2000]
	OS_ENTER_CRITICAL();
	if (rc_command[2] > 30) {
  // 前提油门位于最低点，否则直接退出
      arming_counter = 0;
      return;  
   } 
	 temp = rc_command[3];
	 // 取出副翼舵量
	 if (temp > 1970)
	 {
			if(arming_counter < ARM_DELAY)
			{
				arming_counter ++;
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == ARM_DELAY)
			{
				arming_counter = 0;
				// 计数器清零
				STATUS_LED_ON;
				// 指示灯点亮
				arm_motors_flag = 1;
				// 可以输出信号
			}
	 }
	 else if(temp < 30)
	 // 如果通讯状况不良，则舵量信号一直是0，保证ARM_MOTORS==0
	 {
			if(arming_counter < DISARM_DELAY)
			{
				arming_counter ++; 
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == DISARM_DELAY)
			{
			  arming_counter = 0;
				// 计数器清零
				STATUS_LED_OFF;
				// 上锁状态指示灯熄灭
				arm_motors_flag = 0;
				// 禁止输出信号
			}
	 }
	 OS_EXIT_CRITICAL();
}


 
 
 
 
 
 
 /**
 *
 * 名称: mix_table()
 *
 * 描述：将机型姿态与电机对应
 *
 */ 
void mix_table(od* out_data)
 {
	 u8 i=0;
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  MIX(X,Y,Z)  (u16)(out_data->output[0]*X + out_data->output[1]*Y + out_data->output[2]*Z + out_data->output[3])
	 // 对于X型四轴
	 out_data->motors[0] = MIX(-0.02,+0.02,-0.02);  //右前 1号电机 
	 out_data->motors[1] = MIX(+0.02,+0.02,+0.02);  //左前 2号电机
	 out_data->motors[2] = MIX(+0.02,-0.02,-0.02);  //左后 3号电机
	 out_data->motors[3] = MIX(-0.02,-0.02,+0.02);  //右后 4号电机
	  
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
 
 
 
 
 
 
 
 
 
 
 

