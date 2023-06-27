#include "utilFns.h"
#include "main.h"

char Read_inp1Pin(void)
{
	char key = 0;
	if (HAL_GPIO_ReadPin(GPIOA,Inp1_Pin) == 0)
	{
		HAL_Delay(100);
		if (HAL_GPIO_ReadPin(GPIOA,Inp1_Pin) == 0)
			{
				key = 1;

			}
	}
	return key;
}

char Read_inp2Pin(void)
{
	char key = 0;
	if (HAL_GPIO_ReadPin(GPIOC,Inp2_Pin) == 0)
	{
		HAL_Delay(100);
		if (HAL_GPIO_ReadPin(GPIOC,Inp2_Pin) == 0)
			{
				key = 1;

			}
	}
	return key;
}


void MotorDrive(char index) // Corrected - Enable is enable, disable is disable by switching the names. Rest are same
{
	switch(index)
	{
		case DISABLE_D:
			HAL_GPIO_WritePin(GPIOB,RelayJ15_Pin,GPIO_PIN_SET);
		break;

		case ENABLE_D:
			HAL_GPIO_WritePin(GPIOB,RelayJ15_Pin,GPIO_PIN_RESET);
		break;
	}
}
