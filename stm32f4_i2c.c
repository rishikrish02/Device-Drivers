/*
 * stm32f4_i2c.c
 *
 *  Created on: Jun 4, 2019
 *      Author: rishi
 */


#include "stm32f4_i2c.h"
#include "stm32f407xx.h"
#define apbscalar[8] {2,4,8,16,64,128,257,512} uint16_t
#define systemclk  uint16_t
#define apbh[4] {4,5,6,7} uint16_t
void I2C_deinit(I2C_Reg *pi2cr)
{
	if(pi2cr == I2C1)
	{
		I2C1_RESET();
	}
	else if(pi2cr == I2C2)
		{
			I2C2_RESET();
		}
	else if(pi2cr == I2C3)
		{
			I2C3_RESET();
		}

}
void I2C_peripheralcock(I2C_Reg *pi2cr, uint8_t ena)
{
	if(ena==enable){


	if(pi2cr == I2C1)
	{
		I2C1_peri_clock_en();
	}
	else if(pi2cr == I2C2)
	{
		I2C2_peri_clock_en();
	}
	else if(pi2cr == I2C3)
		{
		I2C3_peri_clock_en();
		}
	}
	else
	{

	}

}
uint16_t findingclockused()
{
	uint16_t pclk, clksrc,temps, apbscal, apb1,apbp1;

	clksrc = (RCC->CFGR >> 2) & 0x03;
	if(clksrc==0)
	{
		systemclk =16000000;
	}
	else if(clksrc ==1)
	{
		systemclk =8000000;
	}
	else if(clksrc ==2)
	{

	}
	temps=(RCC->CFGR >> 4) & 0xff;
	if(temps < 8){
		apbscal =1;
	}
	else
	{
		apbscalar[temps-8];
	}
	apb1=(RCC->CFGR >> 10) & 7;
	if(apb1 <4)
	{
		apbp1 = 1;
	}
	else
	{
		apb1=apbh[apb1-1];
	}
	pclk=(systemclk/apbscal)/apb1;
	return pclk;
}
void I2C_init(I2C_handle *pi2coh)
{
	uint16_t temp=0;
	temp |= pi2coh->pi2cr->CR1 <<10;     //acknowledge

	//clock;
	temp=0;
	temp|=findingclockused()/1000000U;
	pi2coh->pi2cr->CR2= temp & 0x3f;
	//address
	temp=0;
	temp|=pi2coh->pi2cr->OAR1 >> 1;\
	pi2coh->pi2cr->OAR1|=(1<<14);
	pi2coh->pi2cr->OAR1=temp;
//RegisterCCR mode;
	uint16_t ccrreg;
	temp=0;
	if(pi2coh->Pi2cx->I2Cspeed<=normalspeed)
	{
		// normal mode
		ccrreg=(findingclockused()/ 2*pi2coh->Pi2cx->I2Cspeed);
		pi2coh->pi2cr->CCR|= ccrreg & 0XFFF;
	}
	else
	{
		//fast mode
		pi2coh->pi2cr->CCR|=(1<<15);
		pi2coh->pi2cr->CCR|=(1<<14);
		if(pi2coh->Pi2cx->I2Cspeed==fastspeed2)
		{
			ccrreg=(findingclockused()/ 3*pi2coh->Pi2cx->I2Cspeed);
		}
		else
		{
			ccrreg=(findingclockused()/ 25*pi2coh->Pi2cx->I2Cspeed);
		}
		pi2coh->pi2cr->CCR|= ccrreg &oxFFF;
	}
}
uint8_t i2c_flag(I2C_Reg *Pi2cr, uint8_t flagname)
{
	if(Pi2cr->SR1 & flagname)
	{
		return flagset;
	}
	else
	{
		return flagreset;
}

uint8_t masterstop(I2C_Reg *pi2cx,uint8_t flagname)
{
	pi2cx->CR1|=(1<<9);
}

void I2Cexecuteaddress(I2C_Reg *pi2cr, uint8_t addr)
{
	addr=addr <<1;
	addr&=(~1);
	pi2cr->DR=addr;
}
void I2Cexecuteraddress(I2C_Reg *pi2cr, uint8_t addr)
{
	addr=addr <<1;
	addr&=(~0);
	pi2cr->DR=addr;
}
void I2Cclearadrflag(I2C_handle *pi2coh)
{
	uint32_t dummy;

	//check for device mode
	if( pi2coh->pi2cr->SR2 & (1<<0))
	{
		if(pi2coh->txrxstate==i2c_ready_r)
		{
			//disable ack
			pi2coh->pi2cr->CR1|=(1<<10);
		}
		else
		{
			//clearing the adr flag
			dummy=pi2coh->pi2cr->SR1;
			dummy=pi2coh->pi2cr->SR2;
		}
	}
	else if( pi2coh->pi2cr->SR2 & ~(1<<0))
	{
		dummy=pi2coh->pi2cr->SR1;
		dummy=pi2coh->pi2cr->SR2;
	}
}
void mastersenddata(I2C_handle *pi2coh, uint8_t len, uint16_t *txbuffer,uint8_t addr)
{
	//1.generate start condition
	masterstart(pi2coh->Pi2cx);
	//2.confirm that start bit is generated by checking sb bit in status register(untill sb is cleared it will be streched to low)
	while(!i2c_flag(pi2coh->pi2cr, sbflag));
	//3.send the address of the slave with readwrite bit
	I2Cexecuteaddress(pi2coh->pi2cr, addr);
	//4.confirm the addrflag is sent or not
	while(!i2c_flag(pi2coh->pi2cr, addflag));

	I2Cclearadrflag(pi2coh->pi2cr);//mistake has been made here

	//5.send the data until the length is zero
while(len>0)
{

	pi2coh->pi2cr->DR=*txbuffer
	 txbuffer++;
	len--;

//7. when len becomes zero check for data register is empty or not or byte transfer is empty or not before generting stop (also means buffer is emppty so that next transmission begins)
//8. generate stop condition
	while(!i2c_flag(pi2coh->pi2cr,txeflag ));

	while(!i2c_flag(pi2coh->pi2cr,rxeflag ));

	masterstop(pi2coh->pi2cr);
}
void masterstart(I2C_Reg *pi2cx)
{
	pi2cx->CR1|=pi2cx->CR1(1<<8);
}
//-------------------------------------------I2C---------------------------------------------------------------/////////////////

void manageack(I2C_Reg *pi2cr, uint8_t ena)
{
	if(ena == ackena)
	{
		pi2cr->CR1|= (1<10);
	}
	else
	{
		pi2cr->CR &= ~(1<10);
	}
}
void masterrecievedata(I2C_handle *pi2coh, uint8_t len, uint16_t *rxbuffer,uint8_t addr)
{
	//generate start condition
	masterstart(pi2coh->Pi2cx);
	//confirm start generation by checking the sb flag in SR1//UNTILL SB IS CLEARED CLOCK WILL BE STREACHED TO LOW
	while(!i2c_flag(pi2coh->pi2cr, sbflag));
	//SEND THE ADDRESS OF THE SLAVE WITH READ OR WRITE BIT
	I2Cexecuteraddress(pi2coh->pi2cr, addr);
	//once the address is being sent, check  the addr flag-- here the clock is streched to low
	while(i2c_flag(pi2coh->pi2cr,addflag));
	if(len==1)
	{
		manageack(pi2coh->pi2cr,0); //disabled  the ack and make it nak
		//clear the addr

		while(!i2c_flag(pi2coh->pi2cr,rxeflag ));
		//wait untill rxne becomes 1
		I2Cclearadrflag(pi2coh->pi2cr);
		// generate the stop condition

		masterstop(pi2coh->pi2cr);
		// read the data from rx buffer

		rxbuffer= pi2coh->pi2cr->DR;

	}
	else if(len>1)
	{
		//clear the adr flag
		I2Cclearadrflag(pi2coh->pi2cr);

		for(uint32_t i=len; i >0 ;i--)
		{
			I2Cclearadrflag(pi2coh->pi2cr);
			if(i==2)
			{
				//clear the ack bit and generate stop condition
				manageack(pi2coh->pi2cr,0);
				masterstop(pi2coh->pi2cr);
			}

			rxbuffer= pi2coh->pi2cr->DR;
		}

	}
	manageack(pi2coh->pi2cr,1);
}
//---------------interupt based---/

void mastersenddata(I2C_handle *pi2coh, uint8_t len, uint16_t *txbuffer,uint8_t addr,uint8_t sr)
{
	uint8_t temp =pi2coh->txrxstate
			if((temp!= i2c_ready_t)&&(temp!= i2c_ready_t))
			{
				pi2coh->ptxbuffer=txbuffer;
				pi2coh->txlen=len;
				pi2coh->txrxstate=i2c_ready_t;
				pi2coh->devaddr=addr;
				pi2coh->sr=sr;
				masterstart(pi2coh->Pi2cx);// generate start condition -sb flag is sent
				 // now enable buffer, event and error interupy mode
				pi2coh->pi2cr->CR2|=(1<<8);
				pi2coh->pi2cr->CR2|=(1<<9);
				pi2coh->pi2cr->CR2|=(1<<10);

			}
			return temp;
}

void mastersenddata(I2C_handle *pi2coh, uint8_t len, uint16_t *rxbuffer,uint8_t addr,uint8_t sr)
{
	uint8_t temp =pi2coh->txrxstate
			if((temp!= i2c_ready_r)&&(temp!= i2c_ready_r))
			{
				pi2coh->prxbuffer=rxbuffer;
				pi2coh->rxlen=len;
				pi2coh->txrxstate=i2c_ready_r;
				pi2coh->devaddr=addr;
				pi2coh->sr=sr;
				masterstart(pi2coh->Pi2cx);// generate start condition -sb flag is sent
				 // now enable buffer, event and error interupy mode
				pi2coh->pi2cr->CR2|=(1<<8);
				pi2coh->pi2cr->CR2|=(1<<9);
				pi2coh->pi2cr->CR2|=(1<<10);

			}
	return temp;
}

void i2c_ehandling(I2C_handle *pi2coh)
{
	//interupt handling is done for both master and slave device
	//1.handle for interupt generated for sb event
	uint32_t temp1;
	uint32_t temp2;
	uint32_t temp3;
	 temp1=pi2coh->pi2cr->CR2(1<<9);
	 temp2=pi2coh->pi2cr->CR2(1<<10);
	 temp3=pi2coh->pi2cr->SR(1<<0);
	 if(temp1 && temp3)
	 {
		 //sb flag is set//if flag is set execeute the address the flag
		 if(pi2coh->txrxstate== i2c_ready_t)
		 {
			 I2Cexecuteraddress(pi2coh->pi2cr, pi2coh->devaddr);
		 }
		 else if(pi2coh->txrxstate== i2c_ready_r)
		 {
			 I2Cexecuteaddress(pi2coh->pi2cr, pi2coh->devaddr);
		 }
	 }
	 //handle interupt generated for ADR event
	 temp3=pi2coh->pi2cr->SR1 &(1<<1);
	 if(temp1 && temp3)
	 {
		 //addr flag is set
		 I2Cclearadrflag(pi2coh->pi2cr);
	 }
	 // handle generated when byte transfer is done       //" we use the btx flag inorder to end the transmission"//
	 temp3=pi2coh->pi2cr->SR1 &(1<<2);
	 if(temp1&& temp3)
	 {
			 if(pi2coh->txrxstate== I2C_EV_TX)
					 {
						 //make sure txe is set
				 	 	 if(pi2coh->pi2cr->SR1 &(1<<2))
				 	 	 {
				 	 		 //now both txe and and bxe is set
				 	 		masterstop(pi2coh->Pi2cx);
				 	 		 I2Cclosesenddata();
				 	 	 }
					 }

					 else if(pi2coh->txrxstate== i2c_ready_r)
					 {
						 // reset all the member elements of handle structures
						 //set the stop condition
						 I2Ccloserecievedata();
					 }
	 }
	//handle for interupt generated by stop event
	 temp3=pi2coh->pi2cr->SR1 &(1<<4);
	 if(temp3 && temp1)
	 {
		 //stopf is set when you read to sr and write to cr1
		 pi2coh->pi2cr->SR1 |= OXOOOO;

		 //CREATE A CALL BACK FUNCTION AND NOTIFY THE USER
	 }
	 //handle for interupt generated by TXE event
	 temp3=pi2coh->pi2cr->SR1(1<<7);
	 if(temp3 && temp1 && temp2)
	 {
		 // we need to check if it is in master mode or slave mode
		if( pi2coh->pi2cr->SR2 & (1<<0))
		{

		 //when the txe flag is set what we do is let the transmissionhappne
		 if(pi2coh->txrxstate==I2C_EV_TX)
		 {
			 if(pi2coh->txlen >0)
			 {
				 pi2coh->pi2cr->DR= pi2coh->ptxbuffer;
				 pi2coh->ptxbuffer++;
				 pi2coh->txlen--;
			 }
		 }
		 else
		 {
			 //slave mode programming
		 }
	 }

	 //interupt generated by rxne flag
	 temp3=pi2coh->pi2cr->SR1(1<<6);
	 if(temp3 && temp1 && temp2)
	 {
		i2crxneinterupt(pi2coh);
	 }


}
void i2c_errhandling(I2C_handle *pi2coh)
{
// here we are writing i2cerrorhandling
}
void i2crxneinterupt(I2C_handle *pi2coh)
{
	 if(pi2coh->txrxstate==I2C_EV_RX)
			 {
				 if(pi2coh->rxlen ==1)
				 {
					 pi2coh->prxbuffer=pi2coh->pi2cr->DR;
				 }
				 else if(pi2coh->rxlen  >1)
				 {
						if(pi2coh->rxlen==2)
								{
									//clear the ack bit and generate stop condition
									manageack(pi2coh->pi2cr,0);
									masterstop(pi2coh->pi2cr);
								}

							pi2coh->prxbuffer= pi2coh->pi2cr->DR;						}

				 }
			 else if(pi2coh->rxlen  ==0)
			 {
				 //close the stop
				 //close the rxne
				 //notify the application
			 }
			 }
}
void closesenddata(I2C_handle *pi2coh)
{
	pi2coh->pi2cr->CR2 &= ~(1<<9);
	pi2coh->pi2cr->CR2 &= ~(1<<10);
	pi2coh->ptxbuffer=NULL;
	pi2coh->txlen=0;
	pi2coh->txsize=0;

	// the enabe the ack
	//similar thing for closerecieve data
}