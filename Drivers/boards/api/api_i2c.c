/**
 * @file api_i2c.c
 *
 * ���ģ��i2cͨ��Э��
 * 
 */
 

#include <stm32f10x.h>
#include "board_config.h"
#include "api_i2c.h"
#include "app_cfg.h"
#include <stdio.h> 



//static void i2c_delay_s(void);
static void mpu6050_i2c_ack_s(void);
static void mpu6050_i2c_nack_s(void);





/**
 *
 *  ���ƣ� i2c_delay
 *
 *  ������ I2C�����ӳ�
 *
 */
//void i2c_delay_s(void)
//{
//	uint8_t i;

	/*
	  ������Դ������F103��ս�ߡ�
	 	�����ʱ����ͨ��������AX-Pro�߼������ǲ��Եõ��ġ�
		CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
        
    IAR���̱���Ч�ʸߣ���������Ϊ7
	*/
//	for (i = 0; i < 10; i++);
//}









/**
 *
 *  ���ƣ� mpu6050_i2c_start_s
 *
 *  ������ I2C�����ź�
 *
 */
void mpu6050_i2c_start_s(void)
{
	// ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź�
	MPU6050_I2C_SDA_1_FUN();
	MPU6050_I2C_SCL_1_FUN();
	delay_us(4);
	MPU6050_I2C_SDA_0_FUN();
	delay_us(4); 							//��������ʱ�����ֶ���ʱ��һ�᣿���Ӧ���ǵȴ����ݱ�����
	MPU6050_I2C_SCL_0_FUN();
	//delay_us(4);
}






/**
 *
 *  ���ƣ� mpu6050_i2c_stop_s
 *
 *  ������ I2Cֹͣ�ź�(�ⲿ������)
 *
 */
void mpu6050_i2c_stop_s(void)
{
	// ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź�
	MPU6050_I2C_SCL_0_FUN();
	MPU6050_I2C_SDA_0_FUN();
	delay_us(4);
	MPU6050_I2C_SCL_1_FUN();
  MPU6050_I2C_SDA_1_FUN();//���ʱ��ò��Ҫ�󲢲��Ǻ��ϸ�?
	delay_us(4); 
}





/**
 *
 *  ���ƣ� mpu6050_i2c_send_byte
 *
 *  ������ I2C����һ�ֽ�����
 *
 */
void mpu6050_i2c_send_byte_s(uint8_t byte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (byte & 0x80)									//���ȡ���λ
		{
			MPU6050_I2C_SDA_1_FUN();				//���λ��1�����������λ����1
		}
		else
		{
			MPU6050_I2C_SDA_0_FUN();
		}
		delay_us(1);  										//������ò�Ʋ���Ҫ����Ϊ�˱��������
		MPU6050_I2C_SCL_1_FUN();
		delay_us(1);											//ʱ����Ϊ�ߣ����ݲɼ�
		MPU6050_I2C_SCL_0_FUN();  				//ʱ����Ϊ�ͣ������л�
		if (i == 7)
		{
			 MPU6050_I2C_SDA_1_FUN(); 			// �ͷ����ߣ��ӻ�������ΪӦ��
		}
		byte <<= 1;											  // ����һ����Ҫ���͵�λ��Ϊ���λ
		delay_us(1); 										  // ����ʱ���ߵ͵�ƽһ��ʱ��
	}
}






/**
 *
 *  ���ƣ� mpu6050_i2c_read_byte_s
 *
 *  ������ I2C����һ�ֽ�����
 *
 */
uint8_t mpu6050_i2c_read_byte_s(u8 ack)
{
	uint8_t i;
	uint8_t value;

	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;  							 // ÿ�ν��յ��ĵ�ƽ��������һλ
		MPU6050_I2C_SCL_1_FUN();  	 // ʱ���߸ߵ�ƽʱ��ȡ����
		delay_us(1);
		if (MPU6050_I2C_SDA_READ_FUN())
		{
			value++;
		}
		MPU6050_I2C_SCL_0_FUN(); 
		delay_us(1); 
	}
	if(ack==0)    								 // ѭ������������ʵ���жϵȴ�Ӧ����Ӧ���ź�
		mpu6050_i2c_nack_s();  			 // ������ȡ�������β�Ϊ0��������Ӧ���ź�
	else
		mpu6050_i2c_ack_s();   			 // �β�Ϊ1������Ӧ���ź�
	return value;
}






/**
 *
 *  ���ƣ� mpu6050_i2c_wait_ack
 *
 *  ������ �ȴ�Ӧ��
 *
 */
uint8_t mpu6050_i2c_wait_ack_s(void)
{
	uint8_t re;

	MPU6050_I2C_SDA_1_FUN();	// CPU�ͷ�SDA����
	delay_us(1);
	MPU6050_I2C_SCL_1_FUN();	// CPU����SCL = 1, ��ʱ�����᷵��ACKӦ��
	delay_us(1);
	if (MPU6050_I2C_SDA_READ_FUN())	// �ߵ�ƽʱCPU��ȡSDA����״̬
	{
		re = 1;    // ���յ��õ�ƽ�ź�--û��Ӧ�� ����1
	}
	else
	{
		re = 0;
	}
	MPU6050_I2C_SCL_0_FUN(); // ʱ������׼����һ�����ݴ���
	delay_us(1);
	return re;
}






/**
 *
 *  ���ƣ� mpu6050_i2c_ack
 *
 *  ������ ��������Ӧ���ź�
 *
 */
void mpu6050_i2c_ack_s(void)
{
	MPU6050_I2C_SDA_0_FUN();	// CPU����SDA = 0  Ack�ź�
	MPU6050_I2C_SCL_1_FUN();
	delay_us(1);
	MPU6050_I2C_SCL_0_FUN();	// CPU����1��ʱ��
	delay_us(1);
	MPU6050_I2C_SDA_1_FUN();
	delay_us(1);
}







/**
 *
 *  ���ƣ� mpu6050_i2c_nack_s
 *
 *  ������ �������ͷ�Ӧ���ź�
 *
 */
void mpu6050_i2c_nack_s(void)
{
	MPU6050_I2C_SCL_0_FUN();
	MPU6050_I2C_SDA_1_FUN();	// CPU����SDA = 1  Nack�ź�
	delay_us(1);
	MPU6050_I2C_SCL_1_FUN();	// CPU����1��ʱ�� 
	delay_us(1);
	MPU6050_I2C_SCL_0_FUN();
	delay_us(1);
}









/**
 * ����: i2c_timeout_usercallback
 *
 * �������ȴ���ʱ�ص�����
 *
 */
u8 i2c_timeout_usercallback_s(char* name, uint8_t error_code)
{
	printf("%s I2C�ȴ���ʱ��error_code =%d\n", name, error_code);
	return 0;
}
