/**
 * @file board_config.h
 *
 * sanity-v1708 �ڲ���������
 * 
 * ��װ������������(��Ŀ��ѡ)��
 *			PWM_TIM_CH1_DUTY_EN_FUN
 *			��������_����Э������_�˿�_˵����;_����/�˿�/����
 *
 */
 #ifndef _BOARD_CONFIG_H
 #define _BOARD_CONFIG_H



 #include "stm32f10x.h"


 
 /**
  * ͨ�ö˿ڣ��������ú�
	*
	*/
 #define						GPIO_CLK_Config_FUN							RCC_APB2PeriphClockCmd
 // ��Ϊͬʱ��Ҫ��������ҲҪ�������GPIOʱ��
 #define						All_Periph_GPIO_CLK								(RCC_APB2Periph_AFIO|\
																						RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|\
																								RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
 
 
 
 
 
 
 

 /************************************************************************************
 * 
 * 10~13 (PA0~PA3)��������
 *
 * ����: ����TIM2ͨ�ö�ʱ�������PWM
 *   
 ************************************************************************************/

 #define						PWM_TIM											TIM2
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						PWM_TIM_APB1Clock_FUN					RCC_APB1PeriphClockCmd
 #define						PWM_TIM_CLK										RCC_APB1Periph_TIM2

 
 // PA0~PA3������Ҫ��ӳ�䵽TIM2��
 #define						PWM_TIM_REMAP_FUN()						GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE)
 
 // �˿���������
 #define						PWM_TIM_PORT									GPIOA
 #define						PWM1_TIM_PIN									GPIO_Pin_0
 #define						PWM2_TIM_PIN									GPIO_Pin_1
 #define						PWM3_TIM_PIN									GPIO_Pin_2
 #define						PWM4_TIM_PIN									GPIO_Pin_3
 
 // ���ŵ�ƽ��װ
 #define						PWM1_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM1_TIM_PIN)
 #define						PWM2_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM2_TIM_PIN)
 #define						PWM3_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM3_TIM_PIN)
 #define						PWM4_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM4_TIM_PIN)
 
 // ��ʼ��ʱ�����ýṹ�庯��
 // #define						PWM_TIM_TimeBaseInit_FUN							TIM_TimeBaseInit
 
 // ��ʼ��TIM2���ĸ�ͨ��CH1~CH4�Ƚ�����ṹ�庯��
 #define						PWM_TIM_CH1_Init_FUN  							TIM_OC1Init
 #define						PWM_TIM_CH2_Init_FUN  							TIM_OC2Init
 #define						PWM_TIM_CH3_Init_FUN  							TIM_OC3Init
 #define						PWM_TIM_CH4_Init_FUN  							TIM_OC4Init
 
 // ʹ��CCR�ȽϼĴ���Ԥװ�غ���,�Ӷ������ⲿ����ռ�ձ�
 #define						PWM_TIM_CH1_DUTY_EN_FUN()						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH2_DUTY_EN_FUN()						TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH3_DUTY_EN_FUN()						TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH4_DUTY_EN_FUN()						TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable)

 // �ȽϼĴ�����װ
 #define						PWM_TIM_CH1_CCR									CCR1		
 #define						PWM_TIM_CH2_CCR									CCR2
 #define						PWM_TIM_CH3_CCR									CCR3
 #define						PWM_TIM_CH4_CCR									CCR4
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 14~18 (PA4~PA7,PB0)��������
 *
 * ����: SPI1�������߽���ģ��
 *   
 ************************************************************************************/
 #define						RC_SPI											SPI1
 
 // ȷ����������λ��APB2��������Ӧ����ʱ��
 #define						RC_SPI_APB2Clock_FUN							RCC_APB2PeriphClockCmd
 #define						RC_SPI_CLK										RCC_APB2Periph_SPI1
 
 
 // �˿���������
 #define						RC_SPI_PORT										      GPIOA
 #define						RC_SPI_NSS_PIN									    GPIO_Pin_4
 #define						RC_SPI_CLK_PIN									    GPIO_Pin_5
 #define						RC_SPI_MISO_PIN									    GPIO_Pin_6
 #define						RC_SPI_MOSI_PIN   							    GPIO_Pin_7
 
 #define						RC_SPI_INT_CE_PORT								GPIOB
 #define						RC_SPI_INT_PIN									  GPIO_Pin_0 
 #define						RC_SPI_CE_PIN									    GPIO_Pin_1
 
 // ��ʼ��SPI���ýṹ�庯��
// #define						RC_SPI_Init_FUN									SPI_Init

 // ʹ����������ߵ͵�ƽ����
 #define  					RC_SPI_NSS_LOW_FUN()     						GPIO_ResetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define  					RC_SPI_NSS_HIGH_FUN()    						GPIO_SetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define						RC_SPI_CE_LOW_FUN()							  	GPIO_ResetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN)
 #define						RC_SPI_CE_HIGH_FUN()								GPIO_SetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN)
 #define						RC_SPI_INT_SCAN_FUN()								GPIO_ReadInputDataBit( RC_SPI_INT_CE_PORT, RC_SPI_INT_PIN)
 #define						RC_SPI_INT_LOW											0





/************************************************************************************
 * 
 * 46, 21~22 (PB9~PB11)��������
 *
 * ����: I2C2����MS5611ģ��
 *   
 ************************************************************************************/
 #define						MS5611_I2C										     I2C2//�ò���ģ��
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						MS5611_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MS5611_I2C_CLK						    			RCC_APB1Periph_I2C2
 
 // PB5~PB7������Ҫ��ӳ�䵽TIM2��
 #define						MS5611_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C2,ENABLE) 
 
 // �˿���������
 #define						MS5611_I2C_PORT										GPIOB
 #define						MS5611_I2C_INT_PIN								GPIO_Pin_9
 #define						MS5611_I2C_SCL_PIN								GPIO_Pin_10
 #define						MS5611_I2C_SDA_PIN								GPIO_Pin_11
 
 // ��ʼ��SPI���ýṹ�庯��
// #define						MS5611_I2C_Init_FUN							  	I2C_Init					 


 // ������ʱ���ߺ͵�ַ�ߵĲ�����װ
 #define MS5611_I2C_SCL_1_FUN()  GPIO_SetBits(MS5611_I2C_PORT, MS5611_I2C_SCL_PIN)		
 #define MS5611_I2C_SCL_0_FUN()  GPIO_ResetBits(MS5611_I2C_PORT, MS5611_I2C_SCL_PIN)		
	
 #define MS5611_I2C_SDA_1_FUN()  GPIO_SetBits(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)		
 #define MS5611_I2C_SDA_0_FUN()  GPIO_ResetBits(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)		
	
 #define MS5611_I2C_SDA_READ_FUN()  GPIO_ReadInputDataBit(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)
 
 
 
 
 



/************************************************************************************
 * 
 * 30~31 (PA9~PA10)��������
 *
 * ����: ���ô��ڹ�����¼�������ʹ��
 *   
 ************************************************************************************/
 
 #define						DEBUG_USART													USART1
 
 // ȷ����������λ��APB2��������Ӧ����ʱ��
 #define						DEBUG_USART_APB2Clock_FUN						RCC_APB2PeriphClockCmd
 #define						DEBUG_USART_CLK											RCC_APB2Periph_USART1
 
 // PA9~PA10������Ҫ��ӳ�䵽TIM2��
 #define						DEBUG_USART_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE) 
 
 // �˿���������
 #define						DEBUG_USART_PORT										GPIOA
 #define						DEBUG_USART_TX_PIN									GPIO_Pin_9
 #define						DEBUG_USART_RX_PIN									GPIO_Pin_10
 
 // ��ʼ��SPI���ýṹ�庯��
// #define						DEBUG_USART_Init_FUN							  USART_Init	
 
 // �жϷ�����









/************************************************************************************
 * 
 * 41~43 (PB5~PB7)��������
 *
 * ����: I2C1����MPU6050ģ��
 *   
 ************************************************************************************/
 #define						MPU6050_I2C													I2C1
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						MPU6050_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MPU6050_I2C_CLK											RCC_APB1Periph_I2C1
 
 // PB5~PB7������Ҫ��ӳ�䵽TIM2��
 #define						MPU6050_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE) 
 
 // �˿���������
 #define						MPU6050_I2C_PORT										GPIOB
 #define						MPU6050_I2C_INT_PIN									GPIO_Pin_5
 #define						MPU6050_I2C_SCL_PIN									GPIO_Pin_7
 #define						MPU6050_I2C_SDA_PIN       					GPIO_Pin_6
 
 
 // ��ʼ��SPI���ýṹ�庯��
// #define						MPU6050_I2C_Init_FUN								I2C_Init



 // ������ʱ���ߺ͵�ַ�ߵĲ�����װ
 #define MPU6050_I2C_SCL_1_FUN()  GPIO_SetBits(MPU6050_I2C_PORT, MPU6050_I2C_SCL_PIN)		
 #define MPU6050_I2C_SCL_0_FUN()  GPIO_ResetBits(MPU6050_I2C_PORT, MPU6050_I2C_SCL_PIN)		
	
 #define MPU6050_I2C_SDA_1_FUN()  GPIO_SetBits(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)		
 #define MPU6050_I2C_SDA_0_FUN()  GPIO_ResetBits(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)		
	
 #define MPU6050_I2C_SDA_READ_FUN()  GPIO_ReadInputDataBit(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)
 
 
 

/************************************************************************************
 * 
 * 45 (PB8)��������
 *
 * ����: ״ָ̬ʾ��
 *   
 ************************************************************************************/
 // ȷ����������λ��APB2, ������Ӧ����ʱ��, �Ѿ�������������

 
 
 // �˿���������
 #define						STATUS_LED_PORT											GPIOB
 #define						STATUS_LED_PIN											GPIO_Pin_8
 #define						STATUS_LED_ON												GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN)
 #define						STATUS_LED_OFF											GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN)
  
 #define						STATUS_LED1_PORT										GPIOB
 #define						STATUS_LED1_PIN											GPIO_Pin_7
 #define						STATUS_LED1_ON											GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN)
 #define						STATUS_LED1_OFF											GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN)


/************************************************************************************
 * 
 * 
 *
 * ����: ��ʼ����������
 *   
 ************************************************************************************/

  void gpio_clk_config(void);
	void tim_config(void);
  void nrf24l01_config(void);
  void usb_config(void);
  void status_led_gpio_config(void);
	
  void ms5611_i2c_gpio_config_s(void);
	void mpu6050_i2c_gpio_config_s(void);



 
 
 #endif



