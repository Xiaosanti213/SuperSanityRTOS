/**
 *
 * @file ms5611.c
 *
 * ��ѹ��ms5611���������ݴ���
 *
 **/

#include "ms5611.h"
#include "board_config.h" 
#include <stm32f10x_i2c.h>
#include <stdio.h>
#include <stm32f10x.h>


//static void i2c_ms5611_init(void);
//static u8 i2c_ms5611_check(void);
static void i2c_ms5611_read_calibration(uint8_t cx, uint16_t* cx_buffer);
static void i2c_ms5611_read_adc(uint8_t config, uint32_t* adc_buffer);
static void i2c_ms5611_delay(u16); 


static u8 i2c_ms5611_send_cmd(uint8_t cmd); 
static u8 i2c_ms5611_timeout_usercallback(u8 error_code);
static uint8_t i2c_ms5611_receive_data(uint8_t* buffer, uint8_t num);


#define 			I2C_WAIT_TIMEOUT 			((u32)0x1000)



/**
 *  ����: i2c_ms5611_init
 *
 *  ������ms5611�豸��λ��ʼ��
 *
 */
void i2c_ms5611_init(void)
{
	ms5611_config();																	//ms5611���ź�Ƭ����������
  i2c_ms5611_delay(1000); 													//ms5611�ϵ���ʱ
	
	i2c_ms5611_send_cmd(MS5611_RESET);	    					//д�봫������λָ��
  i2c_ms5611_delay(1000); 
}





/**
 *  ����: i2c_ms5611_check
 *
 *  ������ms5611�豸�������״��
 *
 */
u8 i2c_ms5611_check(void)
{
	return SUCCESS; 
}
	









/**
 * 
 * ���ƣ�i2c_ms5611_read_calibration
 *
 * ����������PROM 16bitУ׼��
 *
 */
void i2c_ms5611_read_calibration(uint8_t cx, uint16_t* cx_buffer)
{
	uint8_t cx_pre_get[2] = {1, 1}; 																	// �ֱ��ȡ��8bit ��8bit
	i2c_ms5611_send_cmd(cx);																					// ����ָ��	
	i2c_ms5611_receive_data(cx_pre_get, 2);
	*cx_buffer = ((u16)cx_pre_get[0] << 8) | ((u16)cx_pre_get[1]);    // ���ϵ�u16����
}








/**
 * 
 * ���ƣ�i2c_ms5611_read_adc
 *
 * ����������ADC 24bit�¶�,ѹ������
 *
 */
void i2c_ms5611_read_adc(uint8_t config, uint32_t* adc_buffer)//�β�һ���ֽڣ�������Ҫ
{
	u8 adc_pre[3]; 
	i2c_ms5611_send_cmd(config);																		// ��������ָ��D1��D2�Լ�OSR
	i2c_ms5611_delay(300);																					// �ȴ�ת���������������ԣ���ʱ���ʱ��100~300
	i2c_ms5611_send_cmd(MS5611_ADC_READ); 													// ����MS5611_ADC_READָ��
	i2c_ms5611_receive_data(adc_pre, 3);
	*adc_buffer = (adc_pre[0] << 16) | (adc_pre[1] << 8) | (adc_pre[2]); 
	//printf("adcx: %d\t\n", *adc_buffer);
}














/**
 * 
 * ���ƣ�i2c_ms5611_calculate
 *
 * ���������ϴ��������ݣ������ѹ
 *
 */
int32_t i2c_ms5611_calculate(void)
{
	int32_t difference_temp, temperature, pressure;
	int64_t offset, sensitivity;
	u16 coefficient[6];
	u32 digital_pressure, digital_temp;
	
	i2c_ms5611_init();
	if(i2c_ms5611_check())										// ����״������
	{
		i2c_ms5611_read_calibration(PROM_READ_C1, &(coefficient[0]));	//C1: SENS_T1 u16
		i2c_ms5611_read_calibration(PROM_READ_C2, &(coefficient[1]));	//C2: OFF_T1 u16						
		i2c_ms5611_read_calibration(PROM_READ_C3, &(coefficient[2]));	//C3: TCS u16
		i2c_ms5611_read_calibration(PROM_READ_C4, &(coefficient[3]));	//C4: TCO u16
		i2c_ms5611_read_calibration(PROM_READ_C5, &(coefficient[4]));	//C5: T_REF u16
		i2c_ms5611_read_calibration(PROM_READ_C6, &(coefficient[5]));	//C6: TEMPSENS u16
		
		// printf("MS5611��ȡ�ڲ�У׼ϵ��:\n");
		// printf("C1: %d\n", coefficient[0]);
		// printf("C2: %d\n", coefficient[1]);
		// printf("C3: %d\n", coefficient[2]);
		// printf("C4: %d\n", coefficient[3]);
		// printf("C5: %d\n", coefficient[4]);
		// printf("C6: %d\n", coefficient[5]);
		
		i2c_ms5611_read_adc(MS5611_CONVD1_OSR4096, &digital_pressure);//D1: ѹ��u32
		i2c_ms5611_read_adc(MS5611_CONVD2_OSR4096, &digital_temp);    //D2: �¶�u32
		
		
		// MS5611��ȡADCѹ���¶�ֵ
		// printf("MS5611��ȡѹ���¶�ֵ:\n");
		// printf("digital pressure: %d\n", digital_pressure);
		// printf("digital temperature: %d\n", digital_temp);
		
		difference_temp = (int32_t)digital_temp - ((int32_t)coefficient[4] << 8); 									// ǿ��ת��Ϊ32bit�з�������
		
		// dT = D2 - T_REF = D2 - C5 * 2^8
		temperature = 2000 + (difference_temp * (int64_t)coefficient[5] >> 23);  										// �����㹻��>>���ȼ���* +��ҪС��10^9���� 32bit���ܻ����������int64
		// ʵ���¶ȣ�TEMP = 20C + dT*TEMPSENS = 2000 + dT * C6/2^23
		printf("ms5611�¶ȣ�%.2fC\n", (float)temperature/100);
		
		offset = ((int64_t)coefficient[1] << 16) + ((int64_t)difference_temp * coefficient[3] >> 7 ); 				// int64_t����>>7��ֹ���
		
		// OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
		sensitivity = (((int64_t)coefficient[0]) << 15) + ((int64_t)coefficient[2] * difference_temp >> 8); 	// int64_t ����>>�ᵼ�µõ�0��ԭ��ͬ�ϼ���temperature
		// SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
		
		pressure = (((int64_t)digital_pressure * sensitivity >> 21) - offset) >> 15; 								// �����㹻 -���ȼ�����>>�������ŷ�ֹ����
		// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15 
    printf("ѹ����%.2fmbar\n", (float)pressure/10);
		
		return pressure;
	}
	
	return 0; 												 												 												 						// ʧ��
}









	
/**
 *
 *  ���ƣ� i2c_ms5611_delay
 *
 *  ������ ms5611�ϵ���ʱ
 *
 */
void i2c_ms5611_delay(u16 time_to_delay)
{
	uint16_t i=0,j=0;
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0; i < time_to_delay; i++)
  {
    for(j=0; j < time_to_delay; j++)
    {
      ;
    }
  }
}	











/**
 *
 *  ���ƣ� i2c_ms5611_send_cmd
 *
 *  ������ ms5611����ָ��
 *
 */
u8 i2c_ms5611_send_cmd(u8 cmd)  
{
		u32 i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // ����Ƿ���æ
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(7);
  // }

	
	I2C_GenerateSTART(MS5611_I2C, ENABLE);//1 ������ʼ�ź�	
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(0);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MS5611_I2C, MS5611_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MS5611_I2C, cmd);		//3 ����һ���ֽ�ָ��
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}

	I2C_GenerateSTOP(MS5611_I2C, ENABLE); // 4 ��ֹ�ź�
	return SUCCESS;
}








/**
 * ����: i2c_receive_data
 *
 * ������Burst Read Sequence ���ֽڽ���ģʽ
 *
 */

uint8_t i2c_ms5611_receive_data(uint8_t* buffer, uint8_t num)
{	
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY)) // ����Ƿ���æ  
  {
    if((i2c_wait_timeout--) == 0) return i2c_ms5611_timeout_usercallback(0);
  }
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	
	I2C_GenerateSTART(MS5611_I2C, ENABLE);  // 1 ��ʼ����
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MS5611_I2C, MS5611_SLAVE_ADDRESS, I2C_Direction_Receiver); // 2 ���ʹӻ��豸��ַ
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
	}
	
	
	while(num)// ѭ����ȡ
	{
		if(num == 1)
		{
			I2C_AcknowledgeConfig(MS5611_I2C, DISABLE);//���һ���յ����ݽ���Ack
			I2C_GenerateSTOP(MS5611_I2C, ENABLE); 
		}
		if(I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))  //ע���ǽ��յ����ݲŶ�ȡ���߼���ϵ���Ƿǹ�ϵ
    {      
      /* Read a byte from the slave */
      *buffer = I2C_ReceiveData(MS5611_I2C);
			//��ȡ�����ݣ���ʹ�����ñ�����ָ�룬���Ǳ�֤��������

      /* Point to the next location where the byte read will be saved */
      buffer++; 
      
      /* Decrement the read bytes counter */
      num--;        
    }
	}
	I2C_AcknowledgeConfig(MS5611_I2C, ENABLE);
	
	//��鳢����ձ�־λ
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return SUCCESS;//�ɹ���������
}
	






/**
 *
 *  ���ƣ� i2c_timeout_usercallback
 *
 *  ������ ms5611����ָ��
 *
 */
u8 i2c_ms5611_timeout_usercallback(u8 error_code)
{
	printf("MS5611 �ȴ���ʱ��errorCode =%d\n", error_code); 
	return 0;
}




