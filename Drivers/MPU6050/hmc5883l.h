/**
 * @file hmc5883l.h
 *
 * hmc5883l������غ�
 *
 */
 
#ifndef _HMC5883L_H
#define _HMC5883L_H
 
 
#include "stm32f10x.h" 

#define			HMC5883L_ADDRESS							 0x1E
#define     HMC5883L_SLAVE_ADDRESS				(0x1E<<1)   //�豸��ַȷ��



// �Ĵ�����ַӳ��
#define			HMC5883L_RA_CONFIG_RA					0x00  //Read/Write
#define			HMC5883L_RA_CONFIG_RB					0x01
#define			HMC5883L_RA_MODE_REG					0x02
#define			HMC5883L_RA_DATAOUT_X_MSB			0x03  //���涼��ֻ���Ĵ���
#define			HMC5883L_RA_DATAOUT_X_LSB			0x04
#define			HMC5883L_RA_DATAOUT_Z_MSB			0x05
#define			HMC5883L_RA_DATAOUT_Z_LSB			0x06
#define			HMC5883L_RA_DATAOUT_Y_MSB			0x07
#define			HMC5883L_RA_DATAOUT_Y_LSB			0x08
#define			HMC5883L_RA_STATUS_REG				0x09
#define			HMC5883L_RA_ID_RA							0x0A
#define			HMC5883L_RA_ID_RB							0x0B
#define			HMC5883L_RA_ID_RC							0x0C






// �Ĵ�������
// Configuration Register A ���� ��λֵ0x10
#define			HMC5883L_MA_AVE_OUTPUT1				0x00<<5  //�����������1��ȡ��ֵ(Ĭ��)
#define			HMC5883L_MA_AVE_OUTPUT2				0x01<<5	 //����2��
#define			HMC5883L_MA_AVE_OUTPUT4				0x10<<5	 //����4��
#define			HMC5883L_MA_AVE_OUTPUT8				0x11<<5	 //����8��

#define			HMC5883L_DO_RATE_0P75				0x01<<2		//���������3���Ĵ�������0.75Hz
#define			HMC5883L_DO_RATE_1P5				0x01<<2		//���������3���Ĵ�������1.5Hz
#define			HMC5883L_DO_RATE_3					0x01<<2		//���������3���Ĵ�������3Hz
#define			HMC5883L_DO_RATE_7P5				0x01<<2		//���������3���Ĵ�������7.5Hz
#define			HMC5883L_DO_RATE_15					0x01<<2		//���������3���Ĵ�������15Hz(Ĭ��)
#define			HMC5883L_DO_RATE_30					0x01<<2		//���������3���Ĵ�������30Hz
#define			HMC5883L_DO_RATE_75					0x01<<2		//���������3���Ĵ�������75Hz


#define     HMC5883L_MS_NORMAL					0x00<<0		//�����������(Ĭ��)
#define			HMC5883L_MS_POS_BIAS				0x01<<0		//�����迹�����������
#define			HMC5883L_MS_NEG_BIAS				0x02<<0		//�����迹���Ӹ������



// Configuration Register B �����豸���� ��λֵ0x20
#define			HMC5883L_GN_GAIN_1370			  0x00<<5		//��������1370 LSb/Gauss
#define			HMC5883L_GN_GAIN_1090				0x01<<5		//1090 LSb/Gauss
#define			HMC5883L_GN_GAIN_820				0x02<<5		//820 LSb/Gauss
#define			HMC5883L_GN_GAIN_660				0x03<<5		//660 LSb/Gauss
#define			HMC5883L_GN_GAIN_440				0x04<<5		//440 LSb/Gauss
#define			HMC5883L_GN_GAIN_390				0x05<<5		//390 LSb/Gauss
#define			HMC5883L_GN_GAIN_330				0x06<<5		//330 LSb/Gauss
#define			HMC5883L_GN_GAIN_230				0x07<<5		//230 LSb/Gauss


// Mode Register �����豸����ģʽ ��λֵ0x01
#define			HMC5883L_HS_I2C_3400k				0x01<<7		//ʹ�ܸ���I2C 3400kHz
#define			HMC5883L_MD_CONT_MEAS				0x00<<0		//��������ģʽ
#define			HMC5883L_MD_SING_MEAS				0x01<<0		//���β�������ģʽ
#define			HMC5883L_MD_IDLE_MODE				0x02<<0		//����״̬




void i2c_mpu6050_config_mag(void);
void i2c_mpu6050_init_mag(void);
void i2c_mpu6050_read_mag(float* mag);

// ���ģ��
void i2c_mpu6050_config_mag_s(void);
void i2c_mpu6050_init_mag_s(void);
void i2c_mpu6050_read_mag_s(float* mag);




#endif
