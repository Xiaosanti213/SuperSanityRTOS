/**
 *
 * @file sensors.c
 *
 * 传感器初始化与数据采集
 *
 **/

#include <stm32f10x.h>

#include "board_config.h"
#include "debug.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "nRF24L01.h"
#include "ms5611.h"
#include "sensors.h"
#include "app_cfg.h"
#include "attitude_estimate.h"
#include <stdio.h>
		
extern OS_EVENT	*mbox_sensors;														/* 定义传感器消息邮箱的指针，类似句柄	*/

static void nrf_read_to_motors(u16* rc_command);
static void sensors_calibration(sc* s_calib, sd* s_data);

/**
 *
 * 名称：TaskSensors
 *
 * 描述：传感器数据读取，校准任务
 *
 */
void TaskSensors(void* pdata)
{
	u8 axis = 0;
	float smooth_factor = 1.5;
	static float acc_filtered[3] = {0,0,1};
	//上电之后请保证各个舵程中立
	sd sensors_data;
	sc calib_data;
	
	sensors_calibration(&calib_data, &sensors_data);  // 一定要静止水平放置四轴，摇杆中立再上电，获取校准参数

	pdata = pdata;																		// 防止编译器告警

	//任务大循环
	while(1)
	{
		/* 1 读取并校准传感器数据
		 * 2 发布处理后的传感器消息
		 */
		//1 读取数据，仍然使用sensors_data变量
	  i2c_mpu6050_read_acc_s(sensors_data.acc);
	  i2c_mpu6050_read_gyro_s(sensors_data.gyro);
	  i2c_mpu6050_read_mag_s(sensors_data.mag); 
	  sensors_data.press = i2c_ms5611_calculate_s();
	  nrf_read_to_motors(sensors_data.rc_command);
	
	  //2 加计陀螺仪校准修正
	  for(axis=0; axis<3; axis++)
    {
		  sensors_data.acc[axis] -= calib_data.acc_calib[axis];
		  sensors_data.gyro[axis] -= calib_data.gyro_calib[axis];
			
		//3 迭代加计平滑滤波修正	
			acc_filtered[axis] -= acc_filtered[axis]/smooth_factor;
		  acc_filtered[axis] += sensors_data.acc[axis]/smooth_factor;
		  sensors_data.acc[axis] = acc_filtered[axis];
	  }
	  
	  //4 遥控器舵量修正
	  for(axis=0; axis<4; axis++)
	  {
		  if((calib_data.rc_calib[axis]>50 || calib_data.rc_calib[axis]<-50) && axis!=2)
			  sensors_data.rc_command[axis] = 1000;		// 校准数据出错，三轴舵量全部归中
	    else if((calib_data.rc_calib[axis]>50 || calib_data.rc_calib[axis]<-50) && axis==2)
			  sensors_data.rc_command[axis] = 0;			// 校准数据出错，油门位置最低
		  else																			
		    sensors_data.rc_command[axis] -= calib_data.rc_calib[axis];
	  }
		
		//5 填充并发布消息邮箱
		OSMboxPost(mbox_sensors, &sensors_data);
	  return;
	}
}





	

/**
 *
 * 名称：sensors_init
 *
 * 描述：全部传感器设备初始化
 *
 */ 
void sensors_init(void)
{
	debug_usart_init();						// 调试串口	
	status_led_gpio_config();			// 指示灯LED
	
  i2c_mpu6050_init_s();					// 写入配置参数
	i2c_mpu6050_config_mag_s();		
  i2c_mpu6050_init_mag_s();
  i2c_ms5611_init_s();
	
	spi_nrf_init();								
	spi_nrf_rx_mode();
	tim_config(); 
	
}





/**
 *
 * 名称：sensors_calibration
 *
 * 描述：传感器数据校准
 *
 */ 
void sensors_calibration(sc* s_calib, sd* s_data)
{
	float acc_sum[3] = {0,0,0};
	float gyro_sum[3] = {0,0,0};
	float rc_sum[4] = {0,0,0,0};
	u8 calib_flag;
	u8 axis = 0;
  for(; axis<3 ; axis++)
	{
		s_calib->acc_calib[axis] = 0;
		s_calib->gyro_calib[axis] = 0;
		calib_flag = 0;
		for(;calib_flag<200;)
		{
			i2c_mpu6050_read_acc_s(s_data->acc);//读取原生数据
			i2c_mpu6050_read_gyro_s(s_data->gyro);
			if(axis<2)
				acc_sum[axis] += s_data->acc[axis];
			else
			    acc_sum[axis] += (s_data->acc[axis]-1);
			    gyro_sum[axis] += s_data->gyro[axis];
			    calib_flag++;
		}
		s_calib->acc_calib[axis] = acc_sum[axis]/calib_flag;//20次取平均
		s_calib->gyro_calib[axis] = gyro_sum[axis]/calib_flag;
	}

	
	//遥控器数据校准
	for(axis=0; axis<4; axis++)//为了节省变量，直接使用axis
	{
		s_calib->rc_calib[axis] = 0;
		calib_flag = 0;
		//10次遥控器数据取平均值作为偏移量
		for(;calib_flag<50;)
		{
			nrf_read_to_motors(s_data->rc_command);
			if(s_data->rc_command[axis]<1100 && s_data->rc_command[axis]>900)//遥控数据正常
			{
				rc_sum[axis] += s_data->rc_command[axis]-1000;
			  calib_flag++;
			}
			else 
			{
				s_calib->rc_calib[0] = 100; //接收数据出错，超过阈值退出
				s_calib->rc_calib[1] = 100; 
				s_calib->rc_calib[2] = 100; 
				s_calib->rc_calib[3] = 100; 
			  return;
			}
		}
		s_calib->rc_calib[axis] = rc_sum[axis]/(calib_flag);
		//TODO：如果少于一定次数，校准失败
	}
	
}











/**
 *
 * 名称：nrf_read_to_motors
 *
 * 描述：将u8[4]接收到的数据映射到10000~20000上
 *
 */
void nrf_read_to_motors(u16* rc_command)
{
	 u8 rxbuf[8];
	/*判断接收状态 收到数据*/
	/*while(!spi_nrf_rx_packet(rxbuf))
	{printf("No RC Data%d\n",spi_nrf_rx_packet(rxbuf));}//拿不到则不进行下一步
		*///下面这个步骤已经将motor值映射到了1000~2000上
		rc_command[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/2048*1000;
		rc_command[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/2048*1000;
		rc_command[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/2048*1000;
		rc_command[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/2048*1000;


    //printf("\r\n 从机端接收到遥控器杆量数据：\n");
	  printf("R--ail: %d%s", rc_command[0], "   \n");
	  printf("L--ele: %d%s", rc_command[1], "   \n");
	  printf("R--thr: %d%s", rc_command[2], "   \n");
	  printf("L--rud: %d%s", rc_command[3], "   \n");		
		//printf("数据接收失败，请检查线路连接...\n");
} 





/**
 *
 *  名称： delay_approx
 *
 *  描述： 粗略延时
 *
 */
void delay_approx(u16 time_to_delay)
{
	uint16_t i=0,j=0;
  //延时一段时间，防止数据出错
  for(i=0; i < time_to_delay; i++)
  {
    for(j=0; j < time_to_delay; j++)
    {
      ;
    }
  }
}	









