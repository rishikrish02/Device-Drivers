/*
 * stm32f407xx.h
 *
 *  Created on: May 17, 2019
 *      Author: rishi
 */
#include<stdint.h>
#include<stddef.h>


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#define __vo  volatile
//#define __weak__attribute_((weak))

//FOR GPIO PINS
#define GPIOA = (GPIO_reg *) GPIOA_baseaddr
#define GPIOB = (GPIO_reg *) GPIOB_baseaddr
#define GPIOC = (GPIO_reg *) GPIOC_baseaddr
#define GPIOD = (GPIO_reg *) GPIOD_baseaddr
#define GPIOE = (GPIO_reg *) GPIOE_baseaddr
#define GPIOF = (GPIO_reg *) GPIOF_baseaddr
#define GPIOG = (GPIO_reg *) GPIOG_baseaddr
#define GPIOH = (GPIO_reg *) GPIOH_baseaddr
#define GPIOI = (GPIO_reg *) GPIOI_baseaddr
#define EXTI  = ((EXTIP_REG*)EXTI1_baseaddr)
#define SYS   = ((SYSCLK_REG*)SYSCFG_baseaddr)
//for SPI//

#define SPI1  ((SPI_reg*) SPI1_baseaddr)
#define SPI2  ((SPI_reg*) SPI2_baseaddr)
#define SPI3  ((SPI_reg*) SPI3_baseaddr)
#define SPI4  ((SPI_reg*) SPI4_baseaddr)

//forI2C//
#define I2C1   ((I2C_Reg*) I2C1_baseaddr)
#define I2C2  ((I2C_Reg*) I2C2_baseaddr)
#define I2C3  ((I2C_Reg*) I2C3_baseaddr)

//RCC
#define RCC   (RCC_reg *) RCC_baseaddr

#define EXTI = ((EXTIP_REG *) EXTI1_baseaddr)
#define SYSCLK = (SYSCLK_REG*) SYSCFG_baseaddr)

//ADDRESS
#define FLASHB_addr  0x800000000U
#define SRAM1b_addr  0x200000000U // WE HAVE SRAM OF 112KB SO WE NEED TO ADD 112KB TO THIS BY 112*1024 =? THEN CONVERT IT INTO HEX DECIMAL FORMAT
#define SRAM2b_addr  0x200c1000CU
#define ROMB_addr	 0x1fff00000U

#define APB1_baseaddr   Periph_baseaddr + 0x7C00
#define APB2_baseaddr   Periph_baseaddr + 0x10000

#define AHB1P_baseaddr   Periph_baseaddr + 0x20000
#define AHB2P_baseaddr	 periph_baseaddr + 0x10000000
#define Periph_baseaddr  0X40000000

#define RCC_baseaddr = AHB1_baseaddr + 0x3800



#define GPIOA_baseaddr  AHB1P_baseaddr +0x00
#define GPIOB_baseaddr  AHB1P_baseaddr +0x0400
#define GPIOC_baseaddr  AHB1P_baseaddr +0x0800
#define GPIOD_baseaddr  AHB1P_baseaddr +0x0C00
#define GPIOE_baseaddr  AHB1P_baseaddr +0x1000
#define GPIOF_baseaddr  AHB1P_baseaddr +0x1400
#define GPIOG_baseaddr  AHB1P_baseaddr +0x1800
#define GPIOH_baseaddr  AHB1P_baseaddr +0x1C00
#define GPIOI_baseaddr  AHB1P_baseaddr +0x2000

#define I2C1_baseaddr	APB1_baseaddr + 5400
#define I2C2_baseaddr	APB1_baseaddr + 5800
#define I2C3_baseaddr   APB1_baseaddr + 5C00
#define SPI2_baseaddr   APB1_baseaddr + 3800
#define SPI3_baseaddr   APB1_baseaddr + 3C00
#define USART2_baseaddr APB1_baseaddr + 4400
#define USART3_baseaddr APB1_baseaddr + 4800
#define UART4_baseaddr  APB1_baseaddr + 4C00
#define UART5_baseaddr  APB1_baseaddr + 5000

#define SPI1_baseaddr   APB2_baseaddr + 3000
#define USART6_baseaddr	APB2_baseaddr + 1400
#define USART1_baseaddr APB2_baseaddr + 1100
#define EXTI1_baseaddr  APB2_baseaddr + 3C00
#define SYSCFG_baseaddr APB2_baseaddr + 3800

#define SPI4_baseaddr   APB2_baseaddr + 3400

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];


}GPIO_reg;

#define NVIC_baseaddr 0xE000E100

//processor specific details for IRQ FOR SET
#define NVIC_ISER0   (__vo uint32_t*)0xE000E100
#define NVIC_ISER1   (__vo uint32_t*)0xE000E104
#define NVIC_ISER2   (__vo uint32_t*)0xE000E108
#define NVIC_ISER3   (__vo uint32_t*)0xE000E10C

//FOR RESET
#define NVIC_ICERO   (__vo uint32_t*)0xE000E180
#define NVIC_ICER1   (__vo uint32_t*)0xE000E184
#define NVIC_ICER2   (__vo uint32_t*)0xE000E188
#define NVIC_ICER3   (__vo uint32_t*)0xE000E18C

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTIP_REG;


typedef struct
{
	__vo uint32_t  CR;
	__vo uint32_t  PLLCFGR;
	__vo uint32_t  CFGR;
	__vo uint32_t  CIR;
	__vo uint32_t  AHB1STR;
	__vo uint32_t  AHB2STR;
	__vo uint32_t  AHB3STR;
	__vo uint32_t  APB1STR;
	__vo uint32_t  APB2STR;
	__vo uint32_t  AHB1ENR;
	__vo uint32_t  AHB2ENR;
	__vo uint32_t  AHB3ENR;
	__vo uint32_t  APB1ENR;
	__vo uint32_t  APB2ENR;
	__vo uint32_t  AHB1LPENR;
	__vo uint32_t  AHB2LPENR;
	__vo uint32_t  AHB3LPENR;
	__vo uint32_t  AHB4LPENR;
	__vo uint32_t  APB1LPENR;
	__vo uint32_t  APB2LPENR;
	__vo uint32_t  BDCR;
	__vo uint32_t  CSR;
	__vo uint32_t  SSCGR;
	__vo uint32_t  PLLI2SCFGR;
	__vo uint32_t  PLLSAICFGR;
	__vo uint32_t  DCKCFGR;
	__vo uint32_t  CKGATENR;
	__vo uint32_t  DCKCFGR2;
	 uint32_t  Reseved1[2];
	 uint32_t  Reserved2[];
	 uint32_t  Reserved3[2];
	 uint32_t  Reserved4;
	 uint32_t Reserved5[2];
	 uint32_t Reserved6[2];

}RCC_reg;

typedef struct
{
	    __vo uint32_t  EXTICR[4];
		__vo uint32_t  SYSCFG_CFGR;
		__vo uint32_t  RESERVED1[2];
		__vo uint32_t  RESERVED[2];
		__vo uint32_t  SYSCFG_CMPCR;
		__vo uint32_t  SYSCFG_PMC
		__vo uint32_t  SYSCFG_MEMRMP;

}SYSCLK_REG;

//to enable the clock in GPIO

#define GPIOA_peri_clock_en()   (RCC->AHB1ENR |= (1<<0))
#define GPIOB_peri_clock_en()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_peri_clock_en()   (RCC->AHB1ENR |= (1<<2))
#define GPIOD_peri_clock_en()   (RCC->AHB1ENR |= (1<<3))
#define GPIOE_peri_clock_en()   (RCC->AHB1ENR |= (1<<4))
#define GPIOF_peri_clock_en()   (RCC->AHB1ENR |= (1<<5))
#define GPIOG_peri_clock_en()   (RCC->AHB1ENR |= (1<<6))
#define GPIOH_peri_clock_en()   (RCC->AHB1ENR |= (1<<7))
#define GPIOI_peri_clock_en()   (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_peri_clock_en()   (RCC->AHB1ENR |= (1<<9))

// CLOCK ENABLE FOR I2C PERIPHERALS

#define I2C1_peri_clock_en()    (RCC->APB1ENR |= (1<<21))
#define I2C2_peri_clock_en()	(RCC->APB1ENR |= (1<<22))
#define I2C3_peri_clock_en()	(RCC->APB1ENR |= (1<<23))

//CLOCK ENABLE FOR SPI PERIPHERALS

#define SPI1_peri_clock_en()      (RCC->APB2ENR |= (1<<12))
#define SPI2_peri_clock_en()      (RCC->APB1ENR |= (1<<14))
#define SPI3_peri_clock_en()      (RCC->APB1ENR |= (1<<15))
#define SPI4_peri_clock_en()      (RCC->APB2ENR |= (1<<13))

//CLOCK DISABLE FOR GPIO

#define GPIOA_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_peri_clock_de()   (RCC->AHB1ENR &= ~(1<<9))

//interupt request for micro controller


//clock disable for i2c and SPI

#define I2C1_peri_clock_de()    (RCC->APB1ENR &= ~(1<<21))
#define I2C2_peri_clock_de()	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_peri_clock_de()	(RCC->APB1ENR &= ~(1<<23))

#define SPI1_peri_clock_de()      (RCC->APB2ENR &= ~(1<<12))
#define SPI2_peri_clock_en()      (RCC->APB1ENR &= ~(1<<14))
#define SPI3_peri_clock_en()      (RCC->APB1ENR &= ~(1<<15))
#define SPI4_peri_clock_en()      (RCC->APB2ENR &= ~(1<<13))

//GPIO reset operation is performed

#define GPIOA_RESET()         do {(RCC->AHB1ENR |= (1<<0))  (RCC->AHB1ENR &= ~(1<<0))}while(0)
#define GPIOB_RESET()         do {(RCC->AHB1ENR |= (1<<1))  (RCC->AHB1ENR &= ~(1<<1))}while(0)
#define GPIOC_RESET()         do {(RCC->AHB1ENR |= (1<<2))  (RCC->AHB1ENR &= ~(1<<2))}while(0)
#define GPIOD_RESET()         do {(RCC->AHB1ENR |= (1<<3))  (RCC->AHB1ENR &= ~(1<<3))}while(0)
#define GPIOE_RESET()         do {(RCC->AHB1ENR |= (1<<4))  (RCC->AHB1ENR &= ~(1<<4))}while(0)
#define GPIOF_RESET()         do {(RCC->AHB1ENR |= (1<<5))  (RCC->AHB1ENR &= ~(1<<5))}while(0)
#define GPIOG_RESET()         do {(RCC->AHB1ENR |= (1<<6))  (RCC->AHB1ENR &= ~(1<<6))}while(0)
#define GPIOH_RESET()         do {(RCC->AHB1ENR |= (1<<7))  (RCC->AHB1ENR &= ~(1<<7))}while(0)

//SPI RESET OPERATION
#define SPI1_RESET()         do {(RCC->APB2ENR |= (1<<12))  (RCC->AHB1ENR &= ~(1<<0))}while(0)
#define SPI2_RESET()         do {(RCC->APB1ENR |= (1<<14))  (RCC->AHB1ENR &= ~(1<<1))}while(0)
#define SPI3_RESET()         do {(RCC->APB1ENR |= (1<<13))  (RCC->AHB1ENR &= ~(1<<2))}while(0)
#define SPI4_RESET()         do {(RCC->APB2ENR |= (1<<13))  (RCC->AHB1ENR &= ~(1<<3))}while(0)

//I2C RESET OPERATION
#define I2C1_RESET()         do {(RCC->APB1ENR |= (1<<21))  (RCC->AHB1ENR &= ~(1<<0))}while(0)
#define I2C2_RESET()         do {(RCC->APB1ENR |= (1<<22))  (RCC->AHB1ENR &= ~(1<<1))}while(0)
#define I2C3_RESET()         do {(RCC->APB1ENR |= (1<<23))  (RCC->AHB1ENR &= ~(1<<2))}while(0)
//#define SPI4_RESET()         do {(RCC->APB2ENR |= (1<<13))  (RCC->AHB1ENR &= ~(1<<3))}while(0)

#define SYS_CLK_EN()  RCC->AHB2ENR |= (1<<14)

//enable and disable

#define enable  				1
#define disable 				0
#define set 				    enable
#define reset                   disable
#define flagreset				reset
#define flagset					set
//---------------------------------------------------------------------------------//
#define  GPIO_address_to_code(x)   ((x==GPIOA)?0 :\
								   (x==GPIOB)?1 :\
									(x==GPIOC)?2 :\
									(x==GPIOD)?3 :\
									(x==GPIOE)?4 :\
									(x==GPIOF)?5 :\
									(x==GPIOG)?6 :\
									(x==GPIOH)?7 :0)
//--------------------------------------------------------------------------------//

		//------------------for SPI----------------------------------------------------//

typedef struct
{
    __vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t RXCRCR;
	__vo uint32_t CRPCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SPR;
	__vo uint32_t I2SCFGR;

}SPI_REG;


//----------------------------------------i2c----------------------------------------------------------//
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t DR;
	__vo uint32_t OAR2;
	__vo uint32_t CCR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t TRSIE;
	__vo uint32_t FLTR;

}I2C_Reg;

#define IRQ_EXTIO		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI9_5		23
#define IRQ_EXTI15_10	40


#include"stm32f4_gpio.h"
#include "stm32f4_spi.h"

#endif /* INC_STM32F407XX_H_ */
