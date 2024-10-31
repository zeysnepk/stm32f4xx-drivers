#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>



#define __vo 			volatile


#define ENABLE 			1
#define DISABLE			0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


/**************** BASE ADRESLER --> REFERENCE MANUEL *******************/

//FLASH, SRAM VE ROM ADRESLERİ
#define FLASH_BASE_ADDR			0X08000000U
#define SRAM1_BASE_ADDR			0x20000000U //112 KB
#define SRAM2_BASE_ADDR			0x2001C000U //SRAM2 = SRAM1 + 112(HEX)
#define ROM_BASE_ADDR			0x1FFF0000U //System Memory
#define SRAM 					SRAM1_BASE_ADDRR

//BUS ADRESLERİ
#define PERI_BASE_ADDR			0x40000000U
#define APB1_BASE_ADDR			PERI_BASE_ADDR
#define APB2_BASE_ADDR			0x40010000U
#define AHB1_BASE_ADDR			0x40020000U
#define AHB2_BASE_ADDR			0x50000000U

//AHB1 GPIO ADRESLERİ
#define GPIOA_BASE_ADDR				AHB1_BASE_ADDR
#define GPIOB_BASE_ADDR			( AHB1_BASE_ADDR + 0x0400 )
#define GPIOC_BASE_ADDR			( AHB1_BASE_ADDR + 0x0800 )
#define GPIOD_BASE_ADDR			( AHB1_BASE_ADDR + 0x0C00 )
#define GPIOE_BASE_ADDR			( AHB1_BASE_ADDR + 0x1000 )
#define GPIOF_BASE_ADDR			( AHB1_BASE_ADDR + 0x1400 )
#define GPIOG_BASE_ADDR			( AHB1_BASE_ADDR + 0x1800 )
#define GPIOH_BASE_ADDR			( AHB1_BASE_ADDR + 0x1C00 )

//RCC ADRESİ
#define RCC_BASE_ADDR			( AHB1_BASE_ADDR + 0x3800 )


//APB1 SPI, USART, UART, I2C ADRESLERİ
#define SPI2_BASE_ADDR			( APB1_BASE_ADDR + 0x3800 )
#define SPI3_BASE_ADDR			( APB1_BASE_ADDR + 0x3C00 )
#define USART2_BASE_ADDR		( APB1_BASE_ADDR + 0x4400 )
#define USART3_BASE_ADDR		( APB1_BASE_ADDR + 0x4800 )
#define UART4_BASE_ADDR			( APB1_BASE_ADDR + 0x4C00 )
#define UART5_BASE_ADDR			( APB1_BASE_ADDR + 0x5000 )
#define I2C1_BASE_ADDR			( APB1_BASE_ADDR + 0x5400 )
#define I2C2_BASE_ADDR			( APB1_BASE_ADDR + 0x5800 )
#define I2C3_BASE_ADDR			( APB1_BASE_ADDR + 0x5C00 )

//APB2 USART, SPI, SYSCFG, EXTI
#define USART1_BASE_ADDR       	( APB2_BASE_ADDR + 0x1000 )
#define USART6_BASE_ADDR       	( APB2_BASE_ADDR + 0x1400 )
#define SPI1_BASE_ADDR       	( APB2_BASE_ADDR + 0x3000 )
#define SYSCFG_BASE_ADDR       	( APB2_BASE_ADDR + 0x3800 )
#define EXTI_BASE_ADDR       	( APB2_BASE_ADDR + 0x3C00 )

//SPI1 İÇİN REGISTER ADRESLERİ
#define SPI1_CR1					SPI1_BASE_ADDR			//control register 1
#define SPI1_CR2				( SPI1_BASE_ADDR + 0x04 )	//control register 2
#define SPI1_SR					( SPI1_BASE_ADDR + 0x08 ) 	//status register
#define SPI1_DR					( SPI1_BASE_ADDR + 0x0C )	//data register
#define SPI1_CRCPR				( SPI1_BASE_ADDR + 0x10 )	//CRC polynomial register
#define SPI1_RXCRCR				( SPI1_BASE_ADDR + 0x14 )	//RX CRC register
#define SPI1_TXCRCR				( SPI1_BASE_ADDR + 0x18 )	//TX CRC register
#define SPI1_I2SCFGR			( SPI1_BASE_ADDR + 0x1C )	//configuration register
#define SPI1_I2SPR				( SPI1_BASE_ADDR + 0x20 )	//prescaler register

//HER GPIO PORTU İÇİN KULLANILABİLİR STRUCTURE
typedef struct{
	__vo uint32_t 		MODER;				//GPIO port mode register 					--> 	Address offset: 0x00
	__vo uint32_t		OTYPER;				//GPIO port output type register 			-->		Address offset: 0x04
	__vo uint32_t		OSPEEDER;			//GPIO port output speed register 			--> 	Address offset: 0x08
	__vo uint32_t		PUPDR;				//GPIO port pull-up/pull-down register		--> 	Address offset: 0x0C
	__vo uint32_t		IDR;				//GPIO port input data register				--> 	Address offset: 0x10
	__vo uint32_t		ODR;				//GPIO port output data register	 		--> 	Address offset: 0x14
	__vo uint32_t		BSRR;				//GPIO port bit set/reset register 			--> 	Address offset: 0x18
	__vo uint32_t		LCKR;				//GPIO port configuration lock register 	--> 	Address offset: 0x1C
	__vo uint32_t		AFRL;				//GPIO alternate function low register 		--> 	Address offset: 0x20
	__vo uint32_t		AFRH;				//GPIO alternate function high register 	--> 	Address offset: 0x24
}GPIO_Reg_t;

//GPIO TANIMLAMALARI
#define GPIOA 				( ( GPIO_Reg_t* ) GPIOA_BASE_ADDR )
#define GPIOB 				( ( GPIO_Reg_t* ) GPIOB_BASE_ADDR )
#define GPIOC 				( ( GPIO_Reg_t* ) GPIOC_BASE_ADDR )
#define GPIOD 				( ( GPIO_Reg_t* ) GPIOD_BASE_ADDR )
#define GPIOE 				( ( GPIO_Reg_t* ) GPIOE_BASE_ADDR )
#define GPIOF 				( ( GPIO_Reg_t* ) GPIOF_BASE_ADDR )
#define GPIOG 				( ( GPIO_Reg_t* ) GPIOG_BASE_ADDR )
#define GPIOH 				( ( GPIO_Reg_t* ) GPIOH_BASE_ADDR )


//HER RCC İÇİN KULLANILABİLİR STRUCTURE
typedef struct{
	__vo uint32_t 		CR;					//RCC clock control register 									--> Address offset: 0x00
	__vo uint32_t		PLLCFGR;			//RCC PLL configuration register 								--> Address offset: 0x04
	__vo uint32_t		CFGR;				//RCC clock configuration register 								--> Address offset: 0x08
	__vo uint32_t		CIR;				//RCC clock interrupt register									--> Address offset: 0x0C
	__vo uint32_t		AHBRSTR[3];			//RCC AHB1, AHB2, AHB3 peripheral reset registers				--> Address offsets: 0x10, 0x14, 0x18
	uint32_t			res;				//reserved							 							--> Address offset: 0x1C
	__vo uint32_t		APBRSTR[2];			//RCC APB1, APB2 peripheral reset register 						--> Address offset: 0x20, 0x24
	uint32_t			res2[2];			//reserved														--> Address offsets: 0x28, 0x2C
	__vo uint32_t		AHBENR[3];			//RCC AHB1, AHB2, AHB3 peripheral clock enable register 		--> Address offset: 0x30, 0x34, 0x38
	uint32_t			res3;				//reserved														--> Address offset: 0x3C
	__vo uint32_t		APBENR[2];			//RCC APB1, APB2 peripheral clock enable register 				--> Address offset: 0x40, 0x44
	uint32_t			res4[2];			//reserved				 										--> Address offset: 0x48, 0x4C
	__vo uint32_t		AHBLPENR[3];		//RCC AHB1,.. peripheral clock enable in low power mode register--> Address offset: 0x50, 0x54, 0x58
	uint32_t			res5;				//reserved				 										--> Address offset: 0x5C
	__vo uint32_t		APBLPENR[2];		//RCC APB1,.. peripheral clock enable in low power mode register--> Address offset: 0x60, 0x64
	uint32_t			res6[2];			//reserved			 											--> Address offset: 0x68, 0x6C
	__vo uint32_t		BDCR;				//RCC Backup domain control register 							--> Address offset: 0x70
	__vo uint32_t		CSR;				//RCC clock control & status register							--> Address offset: 0x74
	uint32_t			res7[2];			//reserved														--> Address offset: 0x78, 0x7C
	__vo uint32_t		SSCGR;				//RCC spread spectrum clock generation register 				--> Address offset: 0x80
	__vo uint32_t		PLLI2SCFGR;			//RCC PLLI2S configuration register 							--> Address offset: 0x84
	__vo uint32_t		PLLSAICFGR;			//RCC PLL configuration register 								--> Address offset: 0x88
	__vo uint32_t		DCKCFGR;			//RCC dedicated clock configuration register 					--> Address offset: 0x8C
	__vo uint32_t		CKGATENR;			//RCC clocks gated enable register								--> Address offset: 0x90
	__vo uint32_t		DCKCFGR2;			//RCC dedicated clocks configuration register 2 				--> Address offset: 0x94
}RCC_Reg_t;

//RCC TANIMLAMASI
#define RCC 				( ( RCC_Reg_t* )RCC_BASE_ADDR )

//INTERRUPT İÇİN STRUCTURE VE MACRO
typedef struct{
	__vo uint32_t		IMR;				//Interrupt mask register	 		--> Address offset: 0x00
	__vo uint32_t		EMR;				//Event mask register				--> Address offset: 0x04
	__vo uint32_t		RTSR;				//Rising trigger selection register	--> Address offset: 0x08
	__vo uint32_t		FTSR;				//Falling trigger selection register--> Address offset: 0x0C
	__vo uint32_t		SWIER;				//Software interrupt event register --> Address offset: 0x10
	__vo uint32_t		PR;					//Pending register					--> Address offset: 0x14
}EXTI_Reg_t;

#define EXTI 				( ( EXTI_Reg_t* )EXTI_BASE_ADDR )

//SYSCFG İÇİN STRUCTURE
typedef struct{
	__vo uint32_t		MEMRMP;				//memory remap register 					 --> Address offset: 0x00
	__vo uint32_t		PMC;				//peripheral mode configuration register	 --> Address offset: 0x04
	__vo uint32_t		EXTICR1;			//external interrupt configuration register 1--> Address offset: 0x08
	__vo uint32_t		EXTICR2;			//external interrupt configuration register 2--> Address offset: 0x0C
	__vo uint32_t		EXTICR3;			//external interrupt configuration register 3--> Address offset: 0x10
	__vo uint32_t		EXTICR4;			//external interrupt configuration register 4--> Address offset: 0x14
	uint32_t			res[2];				//reserved									 --> Address offset: 0x18, 0x1C
	__vo uint32_t		CMPCR;				//Compensation cell control register		 --> Address offset: 0x20
	uint32_t			res2[2];			//reserved								     --> Address offset: 0x24, 0x28
	__vo uint32_t		CFGR;				//configuration register					 --> Address offset: 0x2C
}SYSCFG_Reg_t;

//GPIO PİNLERİNİ RESETLEME
#define GPIOA_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 0 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 1 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 2 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 2 ) ); }while(0)
#define GPIOD_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 3 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 3 ) ); }while(0)
#define GPIOE_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 4 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 4 ) ); }while(0)
#define GPIOF_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 5 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 5 ) ); }while(0)
#define GPIOG_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 6 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 6 ) ); }while(0)
#define GPIOH_RESET()		do{ ( RCC -> AHBRSTR[0] |= ( 1 << 7 ) ); ( RCC -> AHBRSTR[0] &= ~( 1 << 7 ) ); }while(0)

//HER GPIO İÇİN CLOCK ETKİNLEŞTİRME
#define GPIOA_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 0 ) )
#define GPIOB_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 1 ) )
#define GPIOC_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 2 ) )
#define GPIOD_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 3 ) )
#define GPIOE_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 4 ) )
#define GPIOF_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 5 ) )
#define GPIOG_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 6 ) )
#define GPIOH_CLK_EN()		( RCC -> AHBENR[0] |= ( 1 << 7 ) )

//HER I2C İÇİN CLOCK ETKİNLEŞTİRME
#define I2C1_CLK_EN()		( RCC -> APBENR[0] |= ( 1 << 21 ) ) 	//21.bit
#define I2C2_CLK_EN()		( RCC -> APBENR[0] |= ( 1 << 22 ) ) 	//22.bit
#define I2C3_CLK_EN()		( RCC -> APBENR[0] |= ( 1 << 23 ) ) 	//23.bit


//HER SPI İÇİN CLOCK ETKİNLEŞTİRME
#define SPI1_CLK_EN() 		( RCC -> APBENR[1] |= ( 1 << 12 ) ) 	//SPI1 VE SPI4 APB2 BUS İÇİNDE
#define SPI2_CLK_EN() 		( RCC -> APBENR[0] |= ( 1 << 14 ) ) 	//SPI2 VE SPI3 APB1 BUS İÇİNDE
#define SPI3_CLK_EN() 		( RCC -> APBENR[0] |= ( 1 << 15 ) )
#define SPI4_CLK_EN() 		( RCC -> APBENR[1] |= ( 1 << 13 ) )

//HER USART İÇİN CLOCK ETKİNLEŞTİRME
#define USART1_CLK_EN()		( RCC -> APBENR[1] |= ( 1 << 4 ) )  	//USART1 VE USART6 APB2 BUS İÇİNDE
#define USART2_CLK_EN()		( RCC -> APBENR[0] |= ( 1 << 17 ) )		//USART2 VE USART3 APB1 BUS İÇİNDE
#define USART3_CLK_EN()		( RCC -> APBENR[0] |= ( 1 << 18 ) )
#define USART6_CLK_EN()		( RCC -> APBENR[1] |= ( 1 << 5 ) )

//SYSCFG İÇİN CLOCK ETKİNLEŞTİRME
#define SYSCFG_CLK_EN()		( RCC -> APBENR[1] |= ( 1 << 14 ) )		//SYSCFG APB2 BUS İÇİNDE

//HER GPIO İÇİN CLOCK DEVRE DIŞI BIRAKMA
#define GPIOA_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 0 ) )
#define GPIOB_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 1 ) )
#define GPIOC_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 2 ) )
#define GPIOD_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 3 ) )
#define GPIOE_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 4 ) )
#define GPIOF_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 5 ) )
#define GPIOG_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 6 ) )
#define GPIOH_CLK_DI()		( RCC -> AHBENR[0] &= ~( 1 << 7 ) )

//HER I2C İÇİN CLOCK DEVRE DIŞI BIRAKMA
#define I2C1_CLK_DI()		( RCC -> APBENR[0] &= ~( 1 << 21 ) )
#define I2C2_CLK_DI()		( RCC -> APBENR[0] &= ~( 1 << 22 ) )
#define I2C3_CLK_DI()		( RCC -> APBENR[0] &= ~( 1 << 23 ) )

//HER SPI İÇİN CLOCK DEVRE DIŞI BIRAKMA
#define SPI1_CLK_DI() 		( RCC -> APBENR[1] &= ~( 1 << 12 ) )
#define SPI2_CLK_DI() 		( RCC -> APBENR[0] &= ~( 1 << 14 ) )
#define SPI3_CLK_DI() 		( RCC -> APBENR[0] &= ~( 1 << 15 ) )
#define SPI4_CLK_DI() 		( RCC -> APBENR[1] &= ~( 1 << 13 ) )

//HER USART İÇİN CLOCK DEVRE DIŞI BIRAKMA
#define USART1_CLK_DI()		( RCC -> APBENR[1] &= ~( 1 << 4 ) )
#define USART3_CLK_DI()		( RCC -> APBENR[0] &= ~( 1 << 18 ) )
#define USART6_CLK_DI()		( RCC -> APBENR[1] &= ~( 1 << 5 ) )

//SYSCFG İÇİN CLOCK DEVRE DIŞI BIRAKMA
#define SYSCFG_CLK_DI()		( RCC -> APBENR[1] &= ~( 1 << 14 ) )

#include "stm32f446xx_gpio.h"

#endif /* INC_STM32F446XX_H_ */
