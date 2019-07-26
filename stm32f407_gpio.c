/*
 * stm32f407_gpio.c
 *
 *  Created on: May 17, 2019
 *      Author: rishi
 */

#include "stm32f4_gpio.h"



void GPIO_init(GPIO_hanlder *pgpioh)// to initialize
{
	//	GPIO
	uint32_t temp;
	if(pgpioh->pgpioconfig->GPIO_pinmode <= GPIO_MODE_analog)
		{
			temp=(pgpioh->pgpioconfig->GPIO_pinmode << (2*pgpioh->pgpioconfig->GPIO_pinnumber) );
			pgpioh->pgpio->MODER &= ~(0x3 << pgpioh->pgpioconfig->GPIO_pinnumber);// TO RESET BEFORE SET
			pgpioh->pgpio->MODER |= temp;
			temp=0;

		}
	else
	{
		if(pgpioh->pgpioconfig->GPIO_pinmode == GPIO_MODE_ft)
		{
			//FTSR IS THE REGISTER OF EXTI
			EXTI->FTSR|= (1 << pgpioh->pgpioconfig->GPIO_pinnumber);
			EXTI->RTSR &= ~(1 << pgpioh->pgpioconfig->GPIO_pinnumber);
		}
		else if (pgpioh->pgpioconfig->GPIO_pinmode == GPIO_MODE_rt)
		{
			       EXTI->RTSR|= (1 << pgpioh->pgpioconfig->GPIO_pinnumber);
					EXTI->FTSR &= ~(1 << pgpioh->pgpioconfig->GPIO_pinnumber);
		}
		else  if(pgpioh->pgpioconfig->GPIO_pinmode == GPIO_MODE_rft)
		{
			       EXTI->FTSR|= (1 << pgpioh->pgpioconfig->GPIO_pinnumber);
					EXTI->RTSR |= (1 << pgpioh->pgpioconfig->GPIO_pinnumber);
		}
	}

	// STEP 2
	// ENABLE EXTI INTERUPT DELIVERY USING IMR
	EXTI->IMR|= (1 << pgpioh->pgpioconfig->GPIO_pinnumber);

	//NOW I AM DOING FOR INTERUPTS
	/** so far we have been enabling interupts in the peripheral side and now we have to do in processor side that
	 * is by enabling NVIC based on priority And the IRQ NUMBERS BY DEFUALT ALL WILL BE ZERO
	 */

	uint32_t temp1 = pgpioh->pgpioconfig->GPIO_pinnumber / 4;
	uint32_t temp2 = pgpioh->pgpioconfig->GPIO_pinnumber % 4;

	uint32_t  port = GPIO_address_to_code(pgpioh->pgpio);
	sys_clk_en();
	SYSCLK->EXTICR[temp1] = port << (temp2 *4);

	//gpio_speed
	temp=0;
	temp=pgpioh->pgpioconfig->GPIO_pinspeed << (2*pgpioh->pgpioconfig->GPIO_pinnumber);
	pgpioh->pgpio->MODER &= ~(0x3 << pgpioh->pgpioconfig->GPIO_pinnumber); //to clear the required bit field we use bitwise and
	pgpioh->pgpio->OSPEEDR |= temp; // bitwise or is prefered
	temp=0;

	//pullupdown
	temp=0;
		temp=pgpioh->pgpioconfig->GPIO_pinoptype<< (2*pgpioh->pgpioconfig->GPIO_pinnumber);
		pgpioh->pgpio->MODER &= ~(0x3 << pgpioh->pgpioconfig->GPIO_pinnumber);
		pgpioh->pgpio->PUPDR |= temp;

	//optype
	temp=0;

		temp=pgpioh->pgpioconfig->GPIO_pinoptype << (2*pgpioh->pgpioconfig->GPIO_pinnumber);
		pgpioh->pgpio->MODER &= ~(0x3 << pgpioh->pgpioconfig->GPIO_pinnumber);
		pgpioh->pgpio->OTYPER|= temp;
		temp=0;

	//alr
		uint8_t temp1;
		uint8_t temp2;
		if(pgpioh->pgpioconfig->GPIO_pinmode==GPIO_MODE_altfun)
		{
			if(pgpioh->pgpioconfig->GPIO_pinnumber-8>0 )
			{
				pgpioh->pgpio->MODER &= ~(0xF << pgpioh->pgpioconfig->GPIO_pinnumber);
				temp1= pgpioh->pgpioconfig->GPIO_pinALTfuncmode << (4*pgpioh->pgpioconfig->GPIO_pinnumber);
				pgpioh->pgpio->AFR[1] |= temp1;
			}
			else if(pgpioh->pgpioconfig->GPIO_pinnumber-8<=0 )
			{
				pgpioh->pgpio->MODER &= ~(0xF << pgpioh->pgpioconfig->GPIO_pinnumber);
				temp2=pgpioh->pgpioconfig->GPIO_pinALTfuncmode << (4*pgpioh->pgpioconfig->GPIO_pinnumber);
				pgpioh->pgpio->AFR[0] |= temp2;
			}



}
		else
		{
			/**GPIO PIN INPUT CONFIGURATION
			  1.PIN MUST BE IN INPUT CONFIGURATION
			  2.CONFUGURE IN WHICH TRIGGER
			  3.ENABLE INTERUPT DELIVERY
			  4.IDENTIFY THE IRQ NUMBER WHICH TRIGGERS THE INTERUPTS TO THE PROCESSOR
			  5.CONFIGURE THE IRQ PRIORITY
			  6.ENABLE INTERUPT RECEPTION
			  7.IMPLEMENT IRQ HANDLER**/

		}

void GPIO_deinit(GPIO_reg *pgpio)
{


	if(pgpio== SPI1)
	{
		SPI1_RESET();
	}
	else if(pgpio == SPI2)
	{
		SPI2_RESET();
	}
	else if(pgpio == SPI3)
	{
		SPI3_RESET();
	}
	else if(pgpio == SPI4)
	{
		SPI4_RESET();
	}

}
}
uint8_t GPIO_readpin(GPIO_reg *pgpio, uint8_t pinnumber)
{
		uint32_t value;
		value =pgpio->IDR >> pinnumber &0x00000001;
		return value;
}
void GPIO_writepin(GPIO_reg *pgpio, uint8_t pinnumber, uint8_t value)
{
	if(value==)
	{
		// write to corresponding pin
		pgpio->ODR |=(1 << pinnumber );
	}
	else
	{
		pgpio->ODR |=(0<<pinnumber);
	}
}
uint16_t GPIO_readport(GPIO_reg *pgpio, uint8_t pinnumber)
{
		uint16_t value;
		value=pgpio->IDR;
		return value;
}
void GPIO_writeport(GPIO_reg *pgpio, uint8_t value)
{
		 pgpio->ODR =value;
}
void GPIO_peripheralcock(GPIO_reg *pgpio, uint8_t ena)
{
	if(ena == enable)
	{
		if(pgpio == GPIOA)
		{
			GPIOA_peri_clock_en();
		}
		else if(pgpio == GPIOB)
		{
			GPIOB_peri_clock_en();
		}
		else if(pgpio == GPIOC)
		{
			GPIOC_peri_clock_en();
		}
		else if(pgpio == GPIOD)
		{
			GPIOD_peri_clock_en();
		}
		else if(pgpio == GPIOE)
		{
			GPIOE_peri_clock_en();
		}
	}
	else
	{

	}


void GPIO_irqconfig(uint8_t IRQ, uint8_t priority, uint8_t, ena)
{
// THIS API IS USED FOR NVIC REGISTER FOR ARM CORTEX PROCSSOR IS PROCESSOR SPECIFIC
	//EACH IRQ NUMBER HAS ITS OWN SET OF REGISTERS WHICH WE NEED TO ENABLE OR DISABLE

	if(ena== enable)
	{
		if(IRQ <31)
		{
			*NVIC_ISER0 |= (1<<IRQ);
		}
		else if(IRQ >31 && IRQ <64)

		{
			uint8_t t = IRQ%32;
		     *NVIC_ISER1 |= (1<<t);
		}
		else if(IRQ >64 && IRQ <96)
		{
			*NVIC_ISER2 |= (1<<IRQ%64);
		}
		else
		{
			if(IRQ <31)
				{
				   *NVIC_ISER0 &= ~(1<<IRQ);
				}
				else if(IRQ >31 && IRQ <64)

				{
					*NVIC_ISER0 &= ~(1<<IRQ%32);
				}
				else if(IRQ >64 && IRQ <96)
				{
					*NVIC_ISER0 &= ~(1<<IRQ%64);
				}
		}
}
	/** we have interupt priority registers
	 * each priority register is for 32 bits which we can use for setting IRQ NUMBERS **/
void GPIO_priorityconfig(uint8_t IRQ_PRIORITY, uint8_t IRQ)
{
	//FIRST LETS FIND OUT IPR REGISTER
	uint8_t i = IRQ_PRIORITY/4;
	uint8_t j = IRQ/4;
	*(NVIC_baseaddr+i*4)|= (IRQ<<(8*j));

}

/**gpio overfiew of interupt delivering
 * 1. when a particular gpio pin generates the interupts it goes through EXTI BLOCK which is connected to the NVIC , IT SEES WHETHER IT IS RISING EDGE OR FALING EDGE
 * AND ENABLES THE INTERUPT DELIVERY OF THE PROCESSOR, ENABLE ALL THE IRQS OF THE REGSITER, THEN WE HAVE FIXED VECTOR ADDRESS WHICH IS ACCESED OF IRQ API IS NOT USED,IF
 * A IRQ IS ALREADY RUNNING THEN COMPARE THE PRIORITY OFBOTHTHE INTERUPTS
 */
void GPIO_irqhandler(uint8_t pinnumber)
{
	EXTI->PR |= (1<<pinnumber);
}
void GPIO_TOGGLE(uint8_t pinnumber, GPIO_reg *pgpio)
{
	pgpio->ODR ^=(1<<pinnumber);
}
#endif
