/**
 * @file api_spi.c
 *
 * spi�ӿں�������
 * 
 */

#include "stm32f10x.h"
#include "api_spi.h"

#include <stdio.h>

	





/**
  * ���ƣ�spi_send_byte
  *
  * ������spi�ӿڷ���һ���ֽ�����
  * 
  */

uint8_t spi_send_byte(SPI_TypeDef* SPIx, uint8_t byte)
{
	
	// ����������ȴ�ʱ��
	uint8_t spi_wait_time = SPI_WAIT_TIMEOUT;
	// �������ݼĴ����Ƿ�Ϊ��
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
		// RESET��ʾ�¼�û����,���ǿ�
		spi_wait_time-- ; 
		if(spi_wait_time == 0)
		{
			// ���ػص��������������ͺ���
			return spi_timeout_usercallback(0);
		}
	}
	// д�����ݼĴ�������Ҫд�������д�뷢�ͻ�����
	SPI_I2S_SendData(SPIx, byte);
	
	
	// ����������ȴ�ʱ��
	spi_wait_time = SPI_WAIT_TIMEOUT;
	
	// �¼�SPI_I2S_FLAG_RXNE
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		{
			spi_wait_time-- ;
			if(spi_wait_time == 0)
			{
				return spi_timeout_usercallback(1);
			}
		}
	// �����ݴӻ���������
	return SPI_I2S_ReceiveData(SPIx);
		
}









/**
  * ���ƣ�spi_timeout_usercallback
  *
  * �������ȴ���ʱ�ص�����
  * 
  */

 uint16_t spi_timeout_usercallback(uint8_t error_code)
{
  // �ȴ���ʱ��Ĵ���,���������Ϣ,������д���е�������Ҫ����
  printf("SPI �ȴ���ʱ! errorCode = %d\n", error_code);
  return 0;
}


