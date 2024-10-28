#include "stm32f446xx.h"

//STM32F446RE kartındaki PC13 butonu ile LD2 ledini açıp kapama


void delay(uint32_t delayTime);

int main(void){
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx						= GPIOA;
	GpioLed.GPIOPINCONF.GPIO_PINNUM		= GPIO_PIN_5;
	GpioLed.GPIOPINCONF.GPIO_PINMODE	= GPIO_MODE_OUT;
	GpioLed.GPIOPINCONF.GPIO_PINSPEED	= GPIO_SPEED_FAST;
	GpioLed.GPIOPINCONF.GPIO_PINTYPE 	= GPIO_OUT_PP;
	GpioLed.GPIOPINCONF.GPIO_PINPUPD 	= GPIO_NO_PUPD;

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx 					= GPIOC;
	GpioButton.GPIOPINCONF.GPIO_PINNUM	= GPIO_PIN_13;
	GpioButton.GPIOPINCONF.GPIO_PINMODE = GPIO_MODE_IN;

	GPIO_ClockControl(GPIOA, ENABLE);
	GPIO_ClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1){
		if( GPIO_ReadInputPin(GPIOC, GPIO_PIN_13) ){
			GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, DISABLE);
		} else {
			GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, ENABLE);
		}
	}
	return 0;
}


