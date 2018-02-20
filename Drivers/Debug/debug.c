/**
 *
 * @file debug.c
 *
 * ʹ��usart1���ڽ��е�����¼����
 *
 **/


#include "debug.h"
#include "board_config.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#include <stdio.h> 






/**
 *
 * ����: debug_usart_init
 *
 * ���������ڳ�ʼ��
 *
 */
void debug_usart_init(void)
{
	
	//gpio_clk_config();// ������
	//����GPIO�봮������
	usb_config();
	
	//��ӡһ���ַ������Ƿ��������
	debug_usart_check();
}
 




/**
 *
 * ���ƣ�debug_usart_check
 *
 * ����������У��
 *
 **/
void debug_usart_check(void)
{
	printf("Debug-usart1 is working\n");
	//��ʾ����򴮿���������
}
 
 


/**
 *
 * ���ƣ�fputc
 *
 * �������ض���printf������
 *
 **/

int fputc(int ch, FILE *f)
{
		/* �⺯������һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}







