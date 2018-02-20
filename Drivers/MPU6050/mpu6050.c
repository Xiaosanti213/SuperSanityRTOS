/**
 * @file mpu6050.c
 *
 * mpu6050 ģ����̬���
 * 
 */
 
#include "stm32f10x.h" 
 
#include "mpu6050.h"
#include "board_config.h"
#include "hmc5883l.h" 

#include <stdio.h>

 
 
static void i2c_mpu6050_delay(void); 
static uint8_t i2c_send_data_single(uint8_t reg, uint8_t byte);
//static uint8_t i2c_send_data(uint8_t reg, uint8_t* buffer, u8 num);
static uint8_t i2c_receive_data(u8 reg, uint8_t* byte_add, uint8_t num);
static uint8_t i2c_timeout_usercallback(uint8_t error_code);

#define I2C_WAIT_TIMEOUT		((u32)0x1000)






/**
 *  ����: i2c_mpu6050_init
 *
 *  ������mpu6050�豸��ʼ��
 *
 */
void i2c_mpu6050_init(void)
{
	
	//gpio_clk_config();//������
	
  mpu6050_config();//����GPIO��I2C����
	
  i2c_mpu6050_delay();//�ϵ����ʱ��ֹ���ݳ���
	
	i2c_send_data_single(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//�������״̬
	i2c_send_data_single(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    								    //�����ǲ�����1kHz
	i2c_send_data_single(MPU6050_RA_CONFIG , MPU6050_EXT_SYNC_ACCEL_YOUT_L);	  								    //ͬ���ź�MPU6050_EXT_SYNC_ACCEL_YOUT_L
	i2c_send_data_single(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G|MPU6050_DHPF_5);	  				  //���ü��ٶȼ�����+-2G, Digital High Pass Filter
	i2c_send_data_single(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//���������ò��Լ죬����+-2000deg/s
  
	i2c_mpu6050_check();//�����Ƿ���������
}	




/**
 *  ����: i2c_mpu6050_check
 *
 *  ������mpu6050�����豸����
 *
 */

uint8_t i2c_mpu6050_check(void)
{
	uint8_t value = 0;
	i2c_receive_data(MPU6050_RA_WHO_AM_I, &value, 1);
	if(value == 0x68)
	{
		printf("MPU6050 is working...\n");
		return SUCCESS;
	}
	return ERROR;
}






/**
 *
 * ���ƣ�i2c_mpu6050_read_acc
 *
 * ��������ȡ�Ӽ����� 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc(float* acc)
{
	
	u16 acc_temp[3];//���м�����ǰ�λ����ʱ����float�洢��ʽ��һ��
	u8 buffer[6];//��Ϊ����������޷�������ν����Ҫ���Ĳ�������
	i2c_receive_data(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	//��ȡ�Ӽ������׵�ַ
	acc_temp[0] = (buffer[0]<<8) | buffer[1];
	acc_temp[1] = (buffer[2]<<8) | buffer[3];
	acc_temp[2] = (buffer[4]<<8) | buffer[5];
  printf("MPU6050  acc_temp: %d%s%d%s%d%s", acc_temp[0], " acc_tempy: ", acc_temp[1], " acc_tempz: ", acc_temp[2], "\n");
	 
	//ת����g��λ
	acc[0] = (float)acc_temp[0] * 2 / 32768;
	acc[1] = (float)acc_temp[1] * 2 / 32768;
	acc[2] = (float)acc_temp[2] * 2 / 32768;
	printf("MPU6050  accx: %.2f%s%.2f%s%.2f%s", acc[0], " g accy: ", acc[1], " g accz: ", acc[2], " g\n");
	
}







/**
 *
 * ���ƣ�i2c_mpu6050_read_gyro
 *
 * ��������ȡ���������� 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro(float* gyro)
{
	u16 gyro_temp[3];
	uint8_t buffer[6];
	i2c_receive_data(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	//��ȡ�����������׵�ַ
	gyro_temp[0] = (buffer[0]<<8) | buffer[1];
	gyro_temp[1] = (buffer[2]<<8) | buffer[3];
	gyro_temp[2] = (buffer[4]<<8) | buffer[5];
	 
	//ת����dps��λ
	gyro[0] = (float)gyro_temp[0] * 2000/ 32768;
	gyro[1] = (float)gyro_temp[1] * 2000/ 32768;
	gyro[2] = (float)gyro_temp[2] * 2000/ 32768;
	
	printf("MPU6050 gyro: %.2f%s%.2f%s%.2f%s", gyro[0], " dps gyroy: ", gyro[1], " dps gyroz: ", gyro[2], "dps\n");
	
}









/**
 *
 * ���ƣ�i2c_mpu6050_read_temp
 *
 * ��������ȡ�¶�����
 *
 */
void i2c_mpu6050_read_temp(float* temp)
{
	
	uint8_t buffer[2];
	uint16_t temp_quant;
	i2c_receive_data(MPU6050_RA_TEMP_OUT_H, buffer, 2);
	//��ȡ�¶������׵�ַ
	temp_quant = (buffer[0]<<8) | buffer[1];
	//ת���������¶�
	temp[0] = (double)temp_quant/340.0 + 36.53;
	//printf("MPU6050�¶�: %.2f%s", temp[0], "\n");

}









/**
 *
 *  ���ƣ� i2c_mpu6050_delay
 *
 *  ������ mpu6050�ϵ���ʱ
 *
 */
void i2c_mpu6050_delay(void)
{
	uint16_t i=0,j=0;
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0; i < 1000;i++)
  {
    for(j=0; j < 1000;j++)
    {
      ;
    }
  }
}







/**
 *
 * ���ƣ�i2c_mpu6050_config_mag
 *
 * ��������MPU6050���ó�bypassģʽ�����ô����̲���
 *
 */

void i2c_mpu6050_config_mag(void)
{
	 i2c_send_data_single(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050���ó�I2C����ģʽ
	 i2c_send_data_single(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050�ʹӻ�ͨѶ����258KHz��������
	 i2c_send_data_single(MPU6050_RA_I2C_SLV4_CTRL, 0x13);
	 // ���ò���������50Hz
	 i2c_send_data_single(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT);
	 //���ô������ӳ�ʹ��
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RA);
   // ���������üĴ���CRA
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, (u8)(HMC5883L_MA_AVE_OUTPUT4|HMC5883L_DO_RATE_15));
	 // ���������ò���������4��ƽ��ֵ��������ݸ�������15Hz
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
   // ��д��Ĵ�����ַ����д��1���ֽڲ�����EXT_SEN_R 0-1����һ����MPU6050_RA_USER_CTRL ʹ�����ݴ���
	
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RB);
   // ���������üĴ���CRB 
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, HMC5883L_GN_GAIN_330);
	 // ���������С��HMC5883L_GN_GAIN_230
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
	 
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_MODE_REG);
   // ������ģʽ�Ĵ��� 
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, HMC5883L_MD_CONT_MEAS);
	 // ������������ģʽ��HMC5883L_MD_CONT_MEAS
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
}
	







/**
 *
 * ���ƣ�i2c_mpu6050_init_mag
 *
 * ��������MPU6050���ó�����ģʽ
 *
 */ 

void i2c_mpu6050_init_mag(void)
{
   i2c_send_data_single(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050���ó�I2C����ģʽ
	 i2c_send_data_single(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050�ʹӻ�ͨѶ����258KHz
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_READ|HMC5883L_ADDRESS);
   // �����̵�ַ
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_DATAOUT_X_MSB);
	 // ���������üĴ�����ַ
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x06);
	 // ʹ�����ݴ��䣬���ݳ��ȣ�3*2�ֽ�
}

 




/**
 *
 * ���ƣ�i2c_mpu6050_read_mag
 *
 * ��������ȡ����������
 *
 */ 
void i2c_mpu6050_read_mag(float* mag)
{
	u16 mag_temp[3];
	uint8_t buffer[6];
	// ʹ�����ݴ���֮�󣬼���ֱ��ͨ����ȡ6050��Ӧ�Ĵ�����ô���������
	i2c_receive_data(MPU6050_RA_EXT_SENS_DATA_00, buffer, 6);
	// ��ַMPU6050_RA_EXT_SENS_DATA_00: Reg73~96
	mag_temp[0] = (buffer[0]<<8) | buffer[1];   //magX-MB: 0x03 LB: 0x04
	mag_temp[1] = (buffer[2]<<8) | buffer[3];   //magZ-MB: 0x05 LB: 0x06
	mag_temp[2] = (buffer[4]<<8) | buffer[5];   //magY-MB: 0x07 LB: 0x08

	// ��������Ϊ230
  mag[0] = (float)mag_temp[0] * 4.35;
	mag[1] = (float)mag_temp[1] * 4.35;
	mag[2] = (float)mag_temp[2] * 4.35;
	
}




 


/**
 * ����: i2c_send_data
 *
 * ������MPU6050  Single-Byte Write Sequence���ֽڷ���ģʽ
 *
 */
 
uint8_t i2c_send_data_single(uint8_t reg, uint8_t byte)
{
	/* Send STRAT condition */
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // ����Ƿ���æ
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(7);
  // }

	
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);//1 ������ʼ�ź�	
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(0);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MPU6050_I2C, reg);		//3 ����һ���ֽڵ�ַ(MPU6050 Single-Byte Write Sequence)
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}

	I2C_SendData(MPU6050_I2C, byte);		//4 ����һ���ֽ�����
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	  if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
		
	I2C_GenerateSTOP (MPU6050_I2C, ENABLE); //5 ֹͣ�ź�
	
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return SUCCESS;
}





					
/**
 * ����: i2c_send_data
 *
 * ������MPU6050  Burst Write Sequence���ֽڷ���ģʽ
 *
 */
 
uint8_t i2c_send_data(uint8_t reg, uint8_t* buffer, u8 num)
{
	u8 index;
	/* Send STRAT condition */
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // ����Ƿ���æ
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(0);
  // }

	
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);//1 ������ʼ�ź�	
	
	while(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MPU6050_I2C, reg);		//3 ����һ���ֽڵ�ַ(MPU6050 Single-Byte Write Sequence)
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}

	for(index = 0; index < num; index++)
	{
		I2C_SendData(MPU6050_I2C, *buffer);		//4 ����һ���ֽ�����
		while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
		buffer++;//������û�н�����һ��ѭ���Ƿ����Σ��
	}
	
	I2C_GenerateSTOP (MPU6050_I2C, ENABLE); //5 ֹͣ�ź�
	
	return SUCCESS;
}







/**
 * ����: i2c_receive_data
 *
 * ������Burst Read Sequence ���ֽڽ���ģʽ
 *
 */

uint8_t i2c_receive_data(u8 reg, uint8_t* buffer, uint8_t num)
{
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY)) // ����Ƿ���æ  
  {
    if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(0);
  }
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);  // 1 ��ʼ����
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); // 2 ���ʹӻ��豸��ַ
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MPU6050_I2C, ENABLE);
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;	
	I2C_SendData(MPU6050_I2C, reg);		//3 ����һ���ֽڵ�ַ
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);  // 4 Burst Read Sequenceģʽ��ʼ����
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Receiver); // 5 ���ʹӻ��豸��ַ
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(5);
		}
		i2c_wait_timeout--;
	}
	while(num)// ѭ����ȡ
	{
		if (num == 1)//��ȡ���һ�ֽ�����
		{
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);//���һ���յ�����Nack
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE); 
		}
    if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the slave */
      *buffer = I2C_ReceiveData(MPU6050_I2C);
			//��ȡ�����ݣ���ʹ�����ñ�����ָ�룬���Ǳ�֤��������

      /* Point to the next location where the byte read will be saved */
      buffer++; 
      
      /* Decrement the read bytes counter */
      num--;        
    }
	}
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
	
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	
	return SUCCESS;//�ɹ���������
}
	



/**
 * ����: i2c_timeout_usercallback
 *
 * �������ȴ���ʱ�ص�����
 *
 */
uint8_t i2c_timeout_usercallback(uint8_t error_code)
{
	printf("MPU6050 �ȴ���ʱ��errorCode =%d\n", error_code);
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return 0;
}










