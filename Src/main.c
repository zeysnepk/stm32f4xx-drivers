#include "stm32f446xx.h"


int main(void){
	return 0;
}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(0);
}
