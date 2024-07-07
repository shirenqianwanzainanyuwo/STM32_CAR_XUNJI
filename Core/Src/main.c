/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t receive_buff[255];
int data_length=7;
#define USART_TX_LEN 7	
uint8_t USART_TX_BUF[USART_TX_LEN];	

///////////////////////////////
void recieve(void);
void send(void);

int ble_send(uint8_t *data,int len);
void Float_to_Byte(float f,uint8_t *byte);
void Short_to_Byte(short s,uint8_t *byte);
void Int_to_Byte(int i,uint8_t *byte);

void limiting_target_rpm_right(void);
void limiting_target_rpm_left(void);

void encoder_speed(void);


typedef struct {
    uint8_t option;
		int xunji;
		char aim_or_not;
} RecData;

 RecData rec_data;//��������ҡ������


float rmp_left,rmp_right;//Ŀǰת��

int target_rpm_right_origin=0;//������Ŀ��ת��
int target_rpm_left_origin=0;//Ŀ��ת��
int target_rpm_right=0;//�����޷����Ŀ��ת��
int target_rpm_left=0;//Ŀ��ת��
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t speed = 60 ;
uint32_t rmp_base = 250 ;


//pid
int Encoder_count_left,Encoder_count_right,Target_Velocity_left,Target_Velocity_right; 
float Velocity_KP=0.6,Velocity_KI=0.06,Velocity_KD; 

int Moto_pwm_left=0;
int Moto_pwm_right=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//����TIM2�ı������ӿ�ģʽ
	  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);//����TIM3�ı������ӿ�ģʽ

		//PWM���
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,speed);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,speed);
		
		car_go_straight();

		
		HAL_TIM_Base_Start_IT(&htim9);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)receive_buff, data_length);
		recieve();//recieve
		//send();//send
		encoder_speed();
		HAL_Delay(15);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void recieve(void){
				rec_data.xunji = *((int *) (&receive_buff[1]));
				rec_data.aim_or_not=*((char *) (&receive_buff[5]));
								
				//printf("decodertest");//1
				//printf("xunji=%d ",rec_data.xunji);
				//printf("aim_or_not=%c\n",rec_data.aim_or_not);
			
}

void send(void){//send speed to phone
		USART_TX_BUF[0] = 0xA5;	
	

		Float_to_Byte(rmp_left,&USART_TX_BUF[1]);
		Float_to_Byte(rmp_right,&USART_TX_BUF[5]);


		//USART_TX_BUF[9] =(uint8_t) ((USART_TX_BUF[1]+USART_TX_BUF[2]+USART_TX_BUF[3]+USART_TX_BUF[4]+USART_TX_BUF[5]+USART_TX_BUF[6]+USART_TX_BUF[7]+USART_TX_BUF[8])&0xff);
	
		//USART_TX_BUF[10] = 0x5A;
		ble_send(USART_TX_BUF,11);
	
	
}


//send data to hc-05
int ble_send(uint8_t *data,int len)
{
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//wait until tx complete
	if(!data)return -1;
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)data,len);	//ʹ��DMAģʽ����
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);
	return 0;
}

//change to byte
void Int_to_Byte(int i,uint8_t *byte)
{

	unsigned long longdata = 0;
	longdata = *(unsigned long*)&i;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}
void Float_to_Byte(float f,uint8_t *byte)
{

	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}

void Short_to_Byte(short s,uint8_t *byte)
{
      
	byte[1] = (s & 0xFF00) >> 8;
	byte[0] = (s & 0xFF);
}


//Ŀ��ֵ�޷�
void limiting_target_rpm_left(void)
{	
	  int maximum=700;    //�������Ϊ
		
		target_rpm_left=target_rpm_left_origin;
	  if(target_rpm_left<-maximum) target_rpm_left=-maximum;	
		if(target_rpm_left>maximum)  target_rpm_left=maximum;	
		
}
void limiting_target_rpm_right(void)
{	
	  int maximum=700;    //�������Ϊ
		
		target_rpm_right=target_rpm_right_origin;
	  if(target_rpm_right<-maximum) target_rpm_right=-maximum;	
		if(target_rpm_right>maximum)  target_rpm_right=maximum;	
		
}


void encoder_speed(void){

	if(rec_data.aim_or_not=='Y'){
		
		//����Ļ��ߣ���գ��ұ߼���
		if(rec_data.xunji<0){
			target_rpm_left_origin=rmp_base-0.25*abs(rec_data.xunji);
			target_rpm_right_origin=rmp_base+0.25*abs(rec_data.xunji);
			limiting_target_rpm_left();
			limiting_target_rpm_right();
		}
		
		//����Ļ�ұߣ��ҹգ���߼���
		if(rec_data.xunji>=0){
			target_rpm_right_origin=rmp_base-0.25*abs(rec_data.xunji);
			target_rpm_left_origin=rmp_base+0.25*abs(rec_data.xunji);
			limiting_target_rpm_left();
			limiting_target_rpm_right();
		}
		
	}
	if(rec_data.aim_or_not=='N'){
		target_rpm_left=0;
		target_rpm_right=0;
	}
		//printf("target_rpm_left=%d ",target_rpm_left);
		//printf("target_rpm_right=%d\n\n ",target_rpm_right);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
