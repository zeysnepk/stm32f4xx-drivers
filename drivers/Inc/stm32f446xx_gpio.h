#ifndef STM32F446_GPIO_H_
#define STM32F446_GPIO_H_

#include "stm32f446xx.h"

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pinleri
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15


/*
 * @GPIO_PIN_MODES
 * GPIO modları
 */
#define GPIO_MODE_IN		0		//00: Input (reset state)
#define GPIO_MODE_OUT		1		//01: General purpose output mode
#define GPIO_MODE_ALTFUN	2		//10: Alternate function mode
#define GPIO_MODE_ANALOG	3		//11: Analog mode
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_RFT 	6


/*
 * @GPIO_PIN_OUT_TYPES
 * GPIO output type register türleri
 */
#define GPIO_OUT_PP			0		//0: Output push-pull (reset state)
#define GPIO_OUT_OD			1		//1: Output open-drain


/*
 * @GPIO_PIN_SPEED_TYPES
 * GPIO output speed register türleri
 */
#define GPIO_SPEED_LOW		0		//00: Low speed
#define GPIO_SPEED_MED		1		//01: Medium speed
#define GPIO_SPEED_FAST		2		//10: Fast speed
#define GPIO_SPEED_HIGH		3		//11: High speed


/*
 * @GPIO_PIN_PUPD_TYPES
 * GPIO port pull-up/pull-down register türleri
 */
#define GPIO_NO_PUPD		0		//00: No pull-up, pull-down
#define GPIO_PU				1		//01: Pull-up
#define GPIO_PD				2		//10: Pull-down


//Pin özelliklerini tutan bir structure
typedef struct{
	uint8_t GPIO_PINNUM;    		//Pin numarası							--> @GPIO_PIN_NUMBERS
	uint8_t GPIO_PINMODE;			//Pin modu  							--> @GPIO_PIN_MODES
	uint8_t GPIO_PINSPEED;			//Pin hızı								--> @GPIO_PIN_SPEED_TYPES
	uint8_t GPIO_PINPUPD;			//Pin pull down veya pull up kontrolü	--> @GPIO_PIN_PUPD_TYPES
	uint8_t GPIO_PINTYPE;			//Pin çıkış türü (PP veya OD)			--> @GPIO_PIN_OUT_TYPES
	uint8_t GPIO_PINALTFUNC;		//Pin alternate function modu
}GPIO_PinConf_t;


//Pin tanım structure
typedef struct{
	GPIO_Reg_t *pGPIOx;				//Pine ait GPIO portunun adreslerini tutar
	GPIO_PinConf_t GPIOPINCONF;		//Pine ait özellikleri tutar
}GPIO_Handle_t;


/********** API prototipi **********/
//Pin başlatma ve resetleme fonksiyonları
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_t *pGPIOx);

//Clock ayar fonksiyonu
void GPIO_ClockControl(GPIO_Reg_t *pGPIOx, uint8_t EnOrDi);

//okuma ve yazma fonksiyonları
uint8_t GPIO_ReadInputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum);
uint16_t GPIO_ReadInputPort(GPIO_Reg_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum, uint8_t value);
void GPIO_WriteOutputPort(GPIO_Reg_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum);

//Interrupt fonksiyonları / IRQ: Interrupt Configuration
void GPIO_IRQInterruptConfig(uint8_t irqNum, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t irqNum, uint8_t irqPriority);
void GPIO_IRQHandling(uint8_t pinNum);							//trigger algılandığında çalışacak fonksiyon

#endif /* STM32F446_GPIO_H_ */
