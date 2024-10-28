#include "stm32f446xx.h"

//STM32F446RE kartındaki LD2 ledini delay ile açıp kapama

#define DELAY_TIME 		600000

void delay(uint32_t delayTime);

int main(void){
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx						= GPIOA;
	GpioLed.GPIOPINCONF.GPIO_PINNUM		= GPIO_PIN_5;
	GpioLed.GPIOPINCONF.GPIO_PINMODE	= GPIO_MODE_OUT;
	GpioLed.GPIOPINCONF.GPIO_PINSPEED	= GPIO_SPEED_FAST;
	GpioLed.GPIOPINCONF.GPIO_PINTYPE 	= GPIO_OUT_PP;
	GpioLed.GPIOPINCONF.GPIO_PINPUPD 	= GPIO_NO_PUPD;

	GPIO_ClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay(DELAY_TIME);
	}

	return 0;
}

void delay(uint32_t delayTime){
	for( uint32_t i = 0; i <= delayTime; i++ );
}
