#include "stm32f446xx_gpio.h"

/**
 * @fn		GPIO_Init
 * @brief 	Belirli bir GPIO pinini yapılandırır.
 * @param 	pGPIOHandle: GPIO pininin yapılandırma ayarlarını içeren yapı (struct)
 * @retval 	None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	//GPIO pini için mod ayarı
	if ( pGPIOHandle->GPIOPINCONF.GPIO_PINMODE <= GPIO_MODE_ANALOG ){ //interrupt yok
		//mode bloğu 2li bitlerden oluştuğu için pin mod değeri pin mumarasının 2 katı kadar sola kayar
		temp = ( pGPIOHandle->GPIOPINCONF.GPIO_PINMODE << ( 2 * pGPIOHandle->GPIOPINCONF.GPIO_PINNUM ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM ); //bit alanını temizleme
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {
		//interrupt modları
		if( pGPIOHandle->GPIOPINCONF.GPIO_PINMODE == GPIO_MODE_IT_FT ){
			//FTSR ayarı
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
			//RTSR bitini temizleme
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
		} else if( pGPIOHandle->GPIOPINCONF.GPIO_PINMODE == GPIO_MODE_IT_RT ){
			//RTSR ayarı
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
			//FTSR bitini temizleme
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
		} else if( pGPIOHandle->GPIOPINCONF.GPIO_PINMODE == GPIO_MODE_IT_RFT ){
			//hem FTSR hem de RTSR ayarı
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
		}
		//SYSCFG_EXTICR de GPIO port seçimi
		uint8_t temp1 = pGPIOHandle->GPIOPINCONF.GPIO_PINNUM / 4;
		uint8_t temp2 = pGPIOHandle->GPIOPINCONF.GPIO_PINNUM % 4;

		uint8_t portcode = GPIO_ADDRESS_CODE(pGPIOHandle->pGPIOx);


		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4 );


		//IMR kullanarak EXTI interrupt aktif etme
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
	}

	temp = 0;

	//GPIO pini için hız ayarı
	temp = ( pGPIOHandle->GPIOPINCONF.GPIO_PINSPEED << ( 2 * pGPIOHandle->GPIOPINCONF.GPIO_PINNUM ) );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//GPIO pini için pull-up / pull-down ayarı
	temp = ( pGPIOHandle->GPIOPINCONF.GPIO_PINPUPD << ( 2 * pGPIOHandle->GPIOPINCONF.GPIO_PINNUM ) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//GPIO pini için output type ayarı
	temp = ( pGPIOHandle->GPIOPINCONF.GPIO_PINTYPE << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIOPINCONF.GPIO_PINNUM );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//GPIO pini için alternate function mod ayarı
	if ( pGPIOHandle->GPIOPINCONF.GPIO_PINALTFUNC == GPIO_MODE_ALTFUN ){
			uint8_t temp1, temp2;
			temp1 = pGPIOHandle->GPIOPINCONF.GPIO_PINNUM / 8;
			temp2 = pGPIOHandle->GPIOPINCONF.GPIO_PINNUM % 8;

			if ( temp1 == 0 ){
				pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << ( 4 * temp2 ) );
				pGPIOHandle->pGPIOx->AFRL |= pGPIOHandle->GPIOPINCONF.GPIO_PINALTFUNC << ( 4 * temp2 );
			} else {
				pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << ( 4 * temp2 ) );
				pGPIOHandle->pGPIOx->AFRH |= pGPIOHandle->GPIOPINCONF.GPIO_PINALTFUNC << ( 4 * temp2 );
			}
	}

}

/**
 * @fn		GPIO_DeInit
 * @brief	Belirli bir GPIO portunu ilk ayarlarına döndürür (resetler).
 * @param  	pGPIOx: İlk ayarlarına döndürülecek GPIO portunun adresi
 * @retval 	None
 */
void GPIO_DeInit(GPIO_Reg_t *pGPIOx){
	if( pGPIOx == GPIOA ){
		GPIOA_RESET();
	} else if ( pGPIOx == GPIOB ) {
		GPIOB_RESET();
	} else if ( pGPIOx == GPIOC ) {
		GPIOC_RESET();
	} else if ( pGPIOx == GPIOD ) {
		GPIOD_RESET();
	} else if ( pGPIOx == GPIOE ) {
		GPIOE_RESET();
	} else if ( pGPIOx == GPIOF ) {
		GPIOF_RESET();
	} else if ( pGPIOx == GPIOG ) {
		GPIOG_RESET();
	} else if ( pGPIOx == GPIOH ) {
		GPIOH_RESET();
	}
}

/*
 * @fn		GPIO_ClockControl
 * @brief 	GPIO portu için clock sinyalini etkinleştirir veya devre dışı bırakır.
 * @param  	pGPIOx: Saat sinyali etkinleştirilecek veya devre dışı bırakılacak GPIO portunun adresi
 * @param  	EnOrDi: ENABLE veya DISABLE ile etkinleştirme durumu belirlenir
 * @retval 	None
 */
void GPIO_ClockControl(GPIO_Reg_t *pGPIOx, uint8_t EnOrDi){
	if ( EnOrDi == ENABLE ) {
		if( pGPIOx == GPIOA ){
			GPIOA_CLK_EN();
		} else if ( pGPIOx == GPIOB ) {
			GPIOB_CLK_EN();
		} else if ( pGPIOx == GPIOC ) {
			GPIOC_CLK_EN();
		} else if ( pGPIOx == GPIOD ) {
			GPIOD_CLK_EN();
		} else if ( pGPIOx == GPIOE ) {
			GPIOE_CLK_EN();
		} else if ( pGPIOx == GPIOF ) {
			GPIOF_CLK_EN();
		} else if ( pGPIOx == GPIOG ) {
			GPIOG_CLK_EN();
		} else if ( pGPIOx == GPIOH ) {
			GPIOH_CLK_EN();
		}
	} else {
		if( pGPIOx == GPIOA ){
			GPIOA_CLK_DI();
		} else if ( pGPIOx == GPIOB ) {
			GPIOB_CLK_DI();
		} else if ( pGPIOx == GPIOC ) {
			GPIOC_CLK_DI();
		} else if ( pGPIOx == GPIOD ) {
			GPIOD_CLK_DI();
		} else if ( pGPIOx == GPIOE ) {
			GPIOE_CLK_DI();
		} else if ( pGPIOx == GPIOF ) {
			GPIOF_CLK_DI();
		} else if ( pGPIOx == GPIOG ) {
			GPIOG_CLK_DI();
		} else if ( pGPIOx == GPIOG ) {
			GPIOH_CLK_DI();
		}
	}
}

/*
 * @fn		GPIO_ReadInputPin
 * @brief 	Verilen pin numarasının siyanlini okur
 * @param  	pGPIOx: Okunacak pinin GPIO portunun adresi
 * @param  	pinNum: Pin numarası
 * @retval 	Pin durumu (1 / 0)
 */
uint8_t GPIO_ReadInputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum){
	uint8_t value;
	value = ( uint8_t ) ( ( pGPIOx->IDR >> pinNum ) & 0x00000001 );
	return value;
}

/*
 * @fn		GPIO_ReadInputPort
 * @brief 	Verilen portun sinyalini okur
 * @param  	pGPIOx: Okunacak GPIO portunun adresi
 * @retval 	Portun durumu (16 bitlik değer)
 */
uint16_t GPIO_ReadInputPort(GPIO_Reg_t *pGPIOx){
	uint16_t value;
	value = ( uint16_t ) pGPIOx->IDR;
	return value;
}

/*
 * @fn		GPIO_WriteOutputPin
 * @brief 	Verilen pin numarasına sinyal gönderir
 * @param  	pGPIOx: Okunacak pinin GPIO portunun adresi
 * @param  	pinNum: Pin numarası
 * @param  	value: gönderilecek değer (1 / 0)
 * @retval 	None
 */
void GPIO_WriteOutputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum, uint8_t value){
	if( value == GPIO_PIN_SET ){
		pGPIOx->ODR |= ( 0x1 << pinNum );
	} else{
		pGPIOx->ODR &= ~( 0x1 << pinNum );
	}
}

/*
 * @fn		GPIO_WriteOutputPort
 * @brief 	Verilen porta sinyal gönderir
 * @param  	pGPIOx: Okunacak GPIO portunun adresi
 * @param  	value: gönderilecek değer (16 bitlik değer)
 * @retval 	None
 */
void GPIO_WriteOutputPort(GPIO_Reg_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/*
 * @fn		GPIO_ToggleOutputPin
 * @brief 	Verilen pinin değerini değiştirir
 * @param  	pGPIOx: Değiştirelecek pinin GPIO portunun adresi
 * @param  	pinNum: Pin numarası
 * @retval 	None
 */
void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx, uint8_t pinNum){
	pGPIOx->ODR ^= ( 1 << pinNum );
}

/*
 * @fn		GPIO_IRQConfig
 * @brief 	Belirli bir IRQ hattı için yapılandırma yapar.
 * @param  	irqNum: Konfigüre edilecek IRQ numarası
 * @param  	irqPriority: IRQ öncelik seviyesi
 * @param  	EnOrDi: IRQ hattını etkinleştirmek veya devre dışı bırakmak için kullanılır (ENABLE/DISABLE)
 * @retval 	None
 */
void GPIO_IRQConfig(uint8_t irqNum, uint8_t irqPriority, uint8_t EnOrDi){

}

/**
 * @brief	Belirli bir GPIO pini için interrupt(kesme) durumunu ele alır.
 * @param 	pinNum: İşlenecek GPIO pin numarası
 * @retval 	None
 */
void GPIO_IRQHandling(uint8_t pinNum){

}

//GPIO ADRESLERİNİN KODLARI
uint8_t GPIO_ADDRESS_CODE(GPIO_Reg_t *x){
	if( x == GPIOA ){
		return 0;
	} else if( x == GPIOB ){
		return 1;
	} else if( x == GPIOC ){
		return 2;
	} else if( x == GPIOD ){
		return 3;
	} else if( x == GPIOE ){
		return 4;
	} else if( x == GPIOF ){
		return 5;
	} else if( x == GPIOG ){
		return 6;
	} else{
		return 7;
	}
}
