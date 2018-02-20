/**
 * @file api_spi.h
 * 
 * spi�ӿں������� �궨��
 *
 */
 
#ifndef __API_SPI_H
#define __API_SPI_H
 
#include "stm32f10x.h"




 

 
// spi�ӿڷ��ͽ���һ���ֽ����ݵȴ�ʱ��
#define SPI_WAIT_TIMEOUT 100
 
 
 
uint8_t spi_send_byte(SPI_TypeDef* SPIx, uint8_t byte);
uint16_t spi_timeout_usercallback(uint8_t errorCode);
 
 
 
 
#endif
 
 
 
