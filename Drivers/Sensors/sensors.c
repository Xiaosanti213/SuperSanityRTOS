/**
 *
 * @file sensors.c
 *
 * ��������ʼ�������ݲɼ�
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
		
extern OS_EVENT	*mbox_sensors;														/* ���崫������Ϣ�����ָ�룬���ƾ��	*/

static void nrf_read_to_motors(u16* rc_command);
static void sensors_calibration(sc* s_calib, sd* s_data);

/**
 *
 * ���ƣ�TaskSensors
 *
 * ���������������ݶ�ȡ��У׼����
 *
 */
void TaskSensors(void* pdata)
{
	u8 axis = 0;
	float smooth_factor = 1.5;
	static float acc_filtered[3] = {0,0,1};
	//�ϵ�֮���뱣֤�����������
	sd sensors_data;
	sc calib_data;
	
	sensors_calibration(&calib_data, &sensors_data);  // һ��Ҫ��ֹˮƽ�������ᣬҡ���������ϵ磬��ȡУ׼����

	pdata = pdata;																		// ��ֹ�������澯

	//�����ѭ��
	while(1)
	{
		/* 1 ��ȡ��У׼����������
		 * 2 ���������Ĵ�������Ϣ
		 */
		//1 ��ȡ���ݣ���Ȼʹ��sensors_data����
	  i2c_mpu6050_read_acc_s(sensors_data.acc);
	  i2c_mpu6050_read_gyro_s(sensors_data.gyro);
	  i2c_mpu6050_read_mag_s(sensors_data.mag); 
	  sensors_data.press = i2c_ms5611_calculate_s();
	  nrf_read_to_motors(sensors_data.rc_command);
	
	  //2 �Ӽ�������У׼����
	  for(axis=0; axis<3; axis++)
    {
		  sensors_data.acc[axis] -= calib_data.acc_calib[axis];
		  sensors_data.gyro[axis] -= calib_data.gyro_calib[axis];
			
		//3 �����Ӽ�ƽ���˲�����	
			acc_filtered[axis] -= acc_filtered[axis]/smooth_factor;
		  acc_filtered[axis] += sensors_data.acc[axis]/smooth_factor;
		  sensors_data.acc[axis] = acc_filtered[axis];
	  }
	  
	  //4 ң������������
	  for(axis=0; axis<4; axis++)
	  {
		  if((calib_data.rc_calib[axis]>50 || calib_data.rc_calib[axis]<-50) && axis!=2)
			  sensors_data.rc_command[axis] = 1000;		// У׼���ݳ����������ȫ������
	    else if((calib_data.rc_calib[axis]>50 || calib_data.rc_calib[axis]<-50) && axis==2)
			  sensors_data.rc_command[axis] = 0;			// У׼���ݳ�������λ�����
		  else																			
		    sensors_data.rc_command[axis] -= calib_data.rc_calib[axis];
	  }
		
		//5 ��䲢������Ϣ����
		OSMboxPost(mbox_sensors, &sensors_data);
	  return;
	}
}





	

/**
 *
 * ���ƣ�sensors_init
 *
 * ������ȫ���������豸��ʼ��
 *
 */ 
void sensors_init(void)
{
	debug_usart_init();						// ���Դ���	
	status_led_gpio_config();			// ָʾ��LED
	
  i2c_mpu6050_init_s();					// д�����ò���
	i2c_mpu6050_config_mag_s();		
  i2c_mpu6050_init_mag_s();
  i2c_ms5611_init_s();
	
	spi_nrf_init();								
	spi_nrf_rx_mode();
	tim_config(); 
	
}





/**
 *
 * ���ƣ�sensors_calibration
 *
 * ����������������У׼
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
			i2c_mpu6050_read_acc_s(s_data->acc);//��ȡԭ������
			i2c_mpu6050_read_gyro_s(s_data->gyro);
			if(axis<2)
				acc_sum[axis] += s_data->acc[axis];
			else
			    acc_sum[axis] += (s_data->acc[axis]-1);
			    gyro_sum[axis] += s_data->gyro[axis];
			    calib_flag++;
		}
		s_calib->acc_calib[axis] = acc_sum[axis]/calib_flag;//20��ȡƽ��
		s_calib->gyro_calib[axis] = gyro_sum[axis]/calib_flag;
	}

	
	//ң��������У׼
	for(axis=0; axis<4; axis++)//Ϊ�˽�ʡ������ֱ��ʹ��axis
	{
		s_calib->rc_calib[axis] = 0;
		calib_flag = 0;
		//10��ң��������ȡƽ��ֵ��Ϊƫ����
		for(;calib_flag<50;)
		{
			nrf_read_to_motors(s_data->rc_command);
			if(s_data->rc_command[axis]<1100 && s_data->rc_command[axis]>900)//ң����������
			{
				rc_sum[axis] += s_data->rc_command[axis]-1000;
			  calib_flag++;
			}
			else 
			{
				s_calib->rc_calib[0] = 100; //�������ݳ���������ֵ�˳�
				s_calib->rc_calib[1] = 100; 
				s_calib->rc_calib[2] = 100; 
				s_calib->rc_calib[3] = 100; 
			  return;
			}
		}
		s_calib->rc_calib[axis] = rc_sum[axis]/(calib_flag);
		//TODO���������һ��������У׼ʧ��
	}
	
}











/**
 *
 * ���ƣ�nrf_read_to_motors
 *
 * ��������u8[4]���յ�������ӳ�䵽10000~20000��
 *
 */
void nrf_read_to_motors(u16* rc_command)
{
	 u8 rxbuf[8];
	/*�жϽ���״̬ �յ�����*/
	/*while(!spi_nrf_rx_packet(rxbuf))
	{printf("No RC Data%d\n",spi_nrf_rx_packet(rxbuf));}//�ò����򲻽�����һ��
		*///������������Ѿ���motorֵӳ�䵽��1000~2000��
		rc_command[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/2048*1000;
		rc_command[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/2048*1000;
		rc_command[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/2048*1000;
		rc_command[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/2048*1000;


    //printf("\r\n �ӻ��˽��յ�ң�����������ݣ�\n");
	  printf("R--ail: %d%s", rc_command[0], "   \n");
	  printf("L--ele: %d%s", rc_command[1], "   \n");
	  printf("R--thr: %d%s", rc_command[2], "   \n");
	  printf("L--rud: %d%s", rc_command[3], "   \n");		
		//printf("���ݽ���ʧ�ܣ�������·����...\n");
} 





/**
 *
 *  ���ƣ� delay_approx
 *
 *  ������ ������ʱ
 *
 */
void delay_approx(u16 time_to_delay)
{
	uint16_t i=0,j=0;
  //��ʱһ��ʱ�䣬��ֹ���ݳ���
  for(i=0; i < time_to_delay; i++)
  {
    for(j=0; j < time_to_delay; j++)
    {
      ;
    }
  }
}	









