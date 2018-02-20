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
#include "api_i2c.h"
#include "sensors.h"


static void i2c_ms5611_read_calibration_s(uint8_t cx, uint16_t* cx_buffer);
static void i2c_ms5611_read_adc_s(uint8_t config, uint32_t* adc_buffer);
static u8 i2c_ms5611_send_cmd_s(uint8_t cmd); 
static uint8_t i2c_ms5611_receive_data_s(uint8_t* buffer, uint8_t num);






/**
 *  ����: i2c_ms5611_init_s
 *
 *  ������ms5611�豸��λ��ʼ��
 *
 */
void i2c_ms5611_init_s(void)
{
	//ms5611_config();																	//ms5611���ź�Ƭ����������
	ms5611_i2c_gpio_config_s();
  delay_approx(1000); 														  	//ms5611�ϵ���ʱ
	
	i2c_ms5611_send_cmd_s(MS5611_RESET);	    					//д�봫������λָ��
  delay_approx(1000); 
}






/**
 * 
 * ���ƣ�i2c_ms5611_read_calibration_s
 *
 * ����������PROM 16bitУ׼��
 *
 */
void i2c_ms5611_read_calibration_s(uint8_t cx, uint16_t* cx_buffer)
{
	uint8_t cx_pre_get[2] = {1, 1}; 															  		// �ֱ��ȡ��8bit ��8bit
	i2c_ms5611_send_cmd_s(cx);																					// ����ָ��	
	i2c_ms5611_receive_data_s(cx_pre_get, 2);
	*cx_buffer = ((u16)cx_pre_get[0] << 8) | ((u16)cx_pre_get[1]);      // ���ϵ�u16����
}





/**
 * 
 * ���ƣ�i2c_ms5611_read_adc
 *
 * ����������ADC 24bit�¶�,ѹ������
 *
 */
void i2c_ms5611_read_adc_s(uint8_t config, uint32_t* adc_buffer)//�β�һ���ֽڣ�������Ҫ
{
	u8 adc_pre[3]; 
	i2c_ms5611_send_cmd_s(config);																		// ��������ָ��D1��D2�Լ�OSR
	delay_approx(300);																	      				// �ȴ�ת���������������ԣ���ʱ���ʱ��100~300
	i2c_ms5611_send_cmd_s(MS5611_ADC_READ); 													// ����MS5611_ADC_READָ��
	i2c_ms5611_receive_data_s(adc_pre, 3);
	*adc_buffer = (adc_pre[0] << 16) | (adc_pre[1] << 8) | (adc_pre[2]); 
}






/**
 * 
 * ���ƣ�i2c_ms5611_calculate_s
 *
 * ���������ϴ��������ݣ������ѹ
 *
 */
int32_t i2c_ms5611_calculate_s(void)
{
	int32_t difference_temp, pressure;
	int64_t offset, sensitivity, temperature;
	u16 coefficient[6];
	u32 digital_pressure, digital_temp;
	
	i2c_ms5611_init_s();
	
		i2c_ms5611_read_calibration_s(PROM_READ_C1, &(coefficient[0]));	//C1: SENS_T1 u16
		i2c_ms5611_read_calibration_s(PROM_READ_C2, &(coefficient[1]));	//C2: OFF_T1 u16						
		i2c_ms5611_read_calibration_s(PROM_READ_C3, &(coefficient[2]));	//C3: TCS u16
		i2c_ms5611_read_calibration_s(PROM_READ_C4, &(coefficient[3]));	//C4: TCO u16
		i2c_ms5611_read_calibration_s(PROM_READ_C5, &(coefficient[4]));	//C5: T_REF u16
		i2c_ms5611_read_calibration_s(PROM_READ_C6, &(coefficient[5]));	//C6: TEMPSENS u16
		
		//printf("MS5611��ȡ�ڲ�У׼ϵ��:\n");
		//printf("C1: %d\n", coefficient[0]);
		//printf("C2: %d\n", coefficient[1]);
		//printf("C3: %d\n", coefficient[2]);
		//printf("C4: %d\n", coefficient[3]);
		//printf("C5: %d\n", coefficient[4]);
		//printf("C6: %d\n", coefficient[5]);
		
		i2c_ms5611_read_adc_s(MS5611_CONVD1_OSR4096, &digital_pressure);//D1: ѹ��u32
		i2c_ms5611_read_adc_s(MS5611_CONVD2_OSR4096, &digital_temp);    //D2: �¶�u32
		
		
		// MS5611��ȡADCѹ���¶�ֵ
		// printf("MS5611��ȡѹ���¶�ֵ:\n");
		// printf("digital pressure: %d\n", digital_pressure);
		// printf("digital temperature: %d\n", digital_temp);
		
		difference_temp = (int32_t)digital_temp - ((int32_t)coefficient[4] << 8); 									// ǿ��ת��Ϊ32bit�з�������
		
		// dT = D2 - T_REF = D2 - C5 * 2^8
		temperature = 2000 + (difference_temp * (int64_t)coefficient[5] >> 23);  										// �����㹻��>>���ȼ���* +��ҪС��10^9���� 32bit���ܻ����������int64
		// ʵ���¶ȣ�TEMP = 20C + dT*TEMPSENS = 2000 + dT * C6/2^23
		//printf("MS5611  Temp  (Ces ): %.2f   \n", (float)temperature/100);
		
		offset = ((int64_t)coefficient[1] << 16) + ((int64_t)difference_temp * coefficient[3] >> 7 ); 				// int64_t����>>7��ֹ���
		
		// OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
		sensitivity = (((int64_t)coefficient[0]) << 15) + ((int64_t)coefficient[2] * difference_temp >> 8); 	// int64_t ����>>�ᵼ�µõ�0��ԭ��ͬ�ϼ���temperature
		// SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
		
		pressure = (((int64_t)digital_pressure * sensitivity >> 21) - offset) >> 15; 								// �����㹻 -���ȼ�����>>�������ŷ�ֹ����
		// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15 
    //printf("MS5611  Press (mbar): %.2f  \n", (float)pressure/10);
		
		return pressure;
	
}

 










/**
 *
 *  ���ƣ� i2c_ms5611_send_cmd
 *
 *  ������ ms5611����ָ��
 *
 */
u8 i2c_ms5611_send_cmd_s(u8 cmd)  
{
	u32 i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	
	ms5611_i2c_start_s();																//1 ������ʼ�ź�	
	
	ms5611_i2c_send_byte_s(MS5611_SLAVE_WRITE_ADDRESS); //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	ms5611_i2c_send_byte_s(cmd);												//3 ����һ���ֽ�ָ��
	
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 2);
		}
		i2c_wait_timeout--;
	}

	ms5611_i2c_stop_s(); 																// 4 ��ֹ�ź�
	return SUCCESS;
}








/**
 * ����: i2c_ms5611_receive_data_s
 *
 * ������ms5611���������ź�
 *
 */

uint8_t i2c_ms5611_receive_data_s(uint8_t* buffer, uint8_t num)
{	
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	ms5611_i2c_start_s();  																// 1 ��ʼ����
	
	ms5611_i2c_send_byte_s(MS5611_SLAVE_READ_ADDRESS); 		// 2 ���ʹӻ��豸��ַ
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 1);
		}
		i2c_wait_timeout--;
	}
	
	
	while(num)																						// 3 ѭ����ȡ
	{
		if(num == 1)
		{
			*buffer = ms5611_i2c_read_byte_s(0);							// 4 ���һ���յ�����Nack����������ֹ�ź�
			ms5611_i2c_stop_s(); 
		}
		else																  							// 4 ���յ����ݲ�����Ack�ź�
    {      
      *buffer = ms5611_i2c_read_byte_s(1);
      buffer++; 
     }
		 num--;        

	}
	

	return SUCCESS;																				// �ɹ���������
}
	




