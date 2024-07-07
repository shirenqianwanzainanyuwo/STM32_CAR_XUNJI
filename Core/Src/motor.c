#include "motor.h"
 
//��������ת
void Left_Moto(int mode)//��
{
		if(mode==1)//��
		{
			HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_SET);
 
		}

		if(mode==0)//��
		{
			HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_RESET);
	
		}
}
 
 //��������ת
void Right_Moto(int mode)//��
{
		if(mode==1)//��
		{
			HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_RESET);//��
			HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_SET);
 
		}

		if(mode==0)//��
		{
			HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_RESET);
	
		}
}
 
 
//??
void car_go_straight(void)
{
   HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_RESET);//��
   HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_SET);
 
   HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_RESET);//��
   HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_SET);
 
}

 
//??
void car_go_right(void)
{
   HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_SET);
 
   HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_SET);
   HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_RESET); 
 
}
 

//??
void car_go_left(void)
{
   HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_SET);
   HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_RESET);
 
   HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_SET);
 
}
 


 
//??
void car_go_ahead(void)
{
   HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_RESET);
 
   HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_RESET);
 
}
 
 
//??


void car_go_after(void)
{
   HAL_GPIO_WritePin(motor11_GPIO_Port,motor11_Pin,GPIO_PIN_SET);
   HAL_GPIO_WritePin(motor12_GPIO_Port,motor12_Pin,GPIO_PIN_RESET);
	
   HAL_GPIO_WritePin(motor21_GPIO_Port,motor21_Pin,GPIO_PIN_SET);
   HAL_GPIO_WritePin(motor22_GPIO_Port,motor22_Pin,GPIO_PIN_RESET);
}