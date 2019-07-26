/*
 * stm32f407_spi.c
 *
 *  Created on: Jun 2, 2019
 *      Author: rishi
 */

#include "stm32f4_spi.h"
#include "stm32f407xx.h"

void SPI_init(SPI_handle *pspioh)
{
// lets configure SPI - CR1 Register
	 uint32_t tempreg;
	 // configure the mode - master or slave
	  tempreg |= pspioh->pspio->SPI_mode << 2;

	  //configuration the bus if bus mode is half duplex or full duplex

	  if(pspioh->pspio->SPI_bus_config == FULLD)
	  {
		   //THE OTHER MODE HAS TO BE CLEARED
		  tempreg &= ~(1<<15);
	  }
	  else if(pspioh->pspio->SPI_bus_config == HALFD)
	  {
		  //BEFORE ONE HAS TO BE CLEARED
		  tempreg |=(1<<15);
	  }
	  else if(pspioh->pspio->SPI_bus_config == SIMPLEXR)
	  {
		  // NEED TO MAKE THE MOSI AS NULL
		  //ONLY MASTER RECIEVER NEED TO BE ACTIVATED
		  tempreg |=(1<<15);
		  tempreg &= ~(1<<10);
	  }
	  //DFF
	  pspioh->pspio->SPI_Dff &= ~(1<<11);
	  //ssm
	  pspioh->pspio->SPI_SSM &= ~(1<<9);
	  //baudrate
	  pspioh->pspio->SPI_SSM &= ~(3<<9);
	  //cpol
	  pspioh->pspio->SPI_SSM &= ~(1<<1);
	  //cpha
	  pspioh->pspio->SPI_SSM &= ~(1<<0);

	  pspioh->pspiox->CR1 =tempreg;

}

void SPI_peripheralcock(SPI_REG *pspiox, uint8_t ena)
{
	if(ena == enable)
	{
		if(pspiox == SPI1)
		{
			SPI1_peri_clock_en();
		}
		else if(pspiox == SPI2)
		{
			SPI1_peri_clock_en();
		}
		else if(pspiox == SPI3)
		{
			SPI1_peri_clock_en();
		}
		else if(pspiox == SPI4)
		{
			SPI1_peri_clock_en();
		}

	}
	else
	{

	}

}
void SPI_deinit(SPI_REG *pspiox)
{
	if(pspiox== SPI1)
	{
		GPIOA_RESET();
	}
	else if(pspiox == SPI2)
	{
		GPIOB_RESET();
	}
	else if(pspiox == SPI3)
	{
		GPIOC_RESET();
	}
	else if(pspiox == SPI4)
	{
		GPIOD_RESET();
	}
}
uint32_t getflagstatus(SPI_REG *pspiox,uint32_t flagname)
		{
			if(pspiox->SR & flagname)
			{
				return flagset;
			}
			return flagreset;
		}
uint32_t getflagstatus1(SPI_REG *pspiox,uint32_t flagname)
		{
			if(pspiox->SR & flagname)
			{
				return flagset;
			}
			else
			{
				return flagreset;
			}
		}
void SPI_sendata(SPI_REG *pspiox, uint32_t len, uint8_t *ptxbuffer)
{
	while(len>0)
	{
		//1.wait until tx buffer is set to 1
		getflagstatus((pspiox,txeflag)==flagreset);
		//check dff bit in cr1
		if(pspiox->CR1 & (1<<11))
		{
			pspiox->DR = *(uint16_t*)ptxbuffer;
			len--;
			len--;
		}
		else if(pspiox->CR1 & (0<<11))
		{
			pspiox->DR = *ptxbuffer;
		    len--;
		}
	}
}
void SPI_recdata(SPI_REG *pspiox, uint32_t len, uint8_t *rxbuffer)
{
	while(len!=0)
	{
	//1.wait until RX buffer is set
		while(getflagstatus1((pspiox, rxeflag)==flagreset));
		//check if DFFbit is 1
		if(pspiox->CR1 & (1<<11))
				{
					 //load the data into it
					len--;
					len--;
					(uint16_t)*rxbuffer++
				}
				else if(pspiox->CR1 & (0<<11))
				{
					 *(uint16_t*)rxbuffer= pspiox->DR;
				    len--;
				    rxbuffer++;
				}
	}
}
//-------------------for interupt based-----------------------//
uint8_t SPI_sendatair(SPI_handle *pspioh, uint32_t len, uint8_t *ptxbuffer)
{
	uint8_t status;
	status=pspioh->txstate;
	if(status !=spitxbusy)
	{
	pspioh->ptxbuffer =ptxbuffer;
	pspioh->txlength=len;
	//mark the spi status as busy
	pspioh->txstate=spitxbusy;
	// enabling txece control bit to get interupt request
	pspioh->pspiox->SR |= (1<<7)
	}
	return status;
}
uint8_t SPI_recdatair(SPI_handle *pspioh, uint32_t len, uint8_t *prxbuffer)
{
	uint8_t status;
	status=pspioh->rxstate;
	if(status !=spirxbusy)
	{
	pspioh->prxbuffer =prxbuffer;
	pspioh->rxlength=len;
	//mark the spi status as busy
	pspioh->rxstate=spirxbusy;
	// enabling txece control bit to get interupt request
	pspioh->pspiox->SR |= (1<<7)
	}
	return status;
}
void SPI_irqintconfig(SPI_handle *pspioh)
{
	//lets checkfor txe
	uint8_t temp1,temp2;
	temp1= pspioh->pspiox->SR & (1<<0);
	temp2 = pspioh->pspiox->CR2 & (1<<7);
	if(temp1==temp2)
	{
		//handle tx buffer
		spi_txe_interupthanlder(pspioh);
	}

	//check for rxe
	uint8_t temp3= pspioh->pspiox->SR & (1<<1);
	uint8_t temp4= pspioh->pspiox->CR2 & (1<<6);
	if(temp3==temp4)
	{
		//handle tx buffer
		spi_rxe_interupthanlder(pspioh);
	}
}
static void spi_txe_interupthanlder(SPI_handle *pspioh)
{
	if(pspioh->pspiox->CR1 & (1<<11))
			{
				pspioh->pspiox->DR = *(uint16_t*)pspioh->ptxbuffer;
				pspioh->txlength--;
				pspioh->txlength--;
			}
			else if(pspioh->pspiox->CR1 & (0<<11))
			{
				pspioh->pspiox->DR = pspioh->ptxbuffer;
			    pspioh->txlength--;
			}
	if(!pspioh->txlength)
	{
		//close spi communication and inform spi that tx communication is shut
		//this condition prevent from txi flag
		spiclosetransmissions(pspioh);
		//to inform the application i need to create a callback mechanism
		SPIapplicationeventcallback(pspioh , txcomplete);
	}
}
static void spi_rxe_interupthanlder(SPI_handle *pspioh)
{
	//16bit
	if(pspioh->pspiox->CR1 & (1<<11))
	{
		*(uint16_t *)pspioh->prxbuffer=(uint16_t)pspioh->pspiox->DR ;
		pspioh->rxlength--;
		pspioh->rxlength--;
		pspioh->rxstate=rxcomplete;
	}
	else if(pspioh->pspiox->CR1 & (0<<11))
	{
		*(uint8_t *)pspioh->prxbuffer=(uint8_t)pspioh->pspiox->DR ;
		pspioh->rxlength--;
		pspioh->rxstate=rxcomplete;
	}
	if(!pspioh->rxlength)
	{
		spiclosereception(pspioh);
		SPIapplicationeventcallback(pspioh , rxcomplete);
	}
}
static void spi_overrun_interupthanlder(SPI_handle *pspioh)
{
	//clear the ovr flag
	//inform the application
	uint8_t temp;
	if(pspioh->ptxbuffer != spitxbusy)
	{
		temp=pspioh->pspiox->DR;
		temp=pspioh->pspiox->SR;
	}

		SPIapplicationeventcallback(pspioh , ovrcomplete);

}
void spiclosetransmissions(SPI_handle *pspioh)
{
		pspioh->pspiox->CR2 &= ~(1<<7);
		pspioh->ptxbuffer=NULL;
		pspioh->txlength=0;
		pspioh->txstate=spiready;
}
void spiclosereception(SPI_handle *pspioh)
{
			pspioh->pspiox->CR2  &= ~(1<<6);
			pspioh->rxlength=0;
			pspioh->rxstate=spiready;
			pspioh->prxbuffer =NULL;
}
//--------------------------------------------------------//
__attribute__((weak))void SPIapplicationeventcallback(SPI_handle *pspioh, uint8_t event)
{
	//this function overridesit
}
