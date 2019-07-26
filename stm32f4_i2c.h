/*
 * stm32f4_i2c.h
 *
 *  Created on: Jun 4, 2019
 *      Author: rishi
 */
#include "stm32f407xx.h"

typedef struct
{
	uint32_t add;
	uint32_t aclctrl;
	uint32_t dutycycle;
	uint32_t I2Cspeed;

}I2C_CONFIG;


typedef struct
{
	I2C_CONFIG *Pi2cx;
	I2C_Reg    *pi2cr;
	uint16_t *ptxbuffer;
	uint16_t *prxbuffer;
	uint16_t txlen;
	uint16_t rxlen;
	uint16_t devaddr;
	uint16_t txsize;
	uint16_t sr;
	uint16_t txrxstate;
}I2C_handle;

#define ackena 	enable
#define ackdna 	disable

#define normalspeed   100000
#define fastspeed2	  200000
#define fastspeed4    400000

#define i2c_ready    0
#define i2c_ready_t  1
#define i2c_ready_r  2

#define dutycycle	enable
#define dutycycle2  disable
//----I2C RELATED FLAGSSSSSSSSSS-------------------///////////////////////////
#define txeflag    (1<<6)
#define rxeflag 	(1<<7)
#define sbflag      (1<<0)
#define arloflag   (1<<9)
#define afflag	  (1<<10)
#define ovrflag  (1<<11)
#define timeoutflag  (1<<14)
#define brrflag  (1<<8)
#define stopf    (1<<<4)
#define addflag  (1<<3)

#define  I2C1_EV  31
#define  I2C1_ER  32

#define I2C_EV_TX  0
#define I2C_EV_RX  1
#define I2C_STOP   2

void I2C_init(I2C_handle *pi2coh); // to initialize

void I2C_deinit(I2C_Reg *pi2cr); //to reset

void I2C_peripheralcock(I2C_Reg *pi2cr, uint8_t ena);
//--- data send and receive for I2C--------------------

//interupt base//

void I2C_irqconfig(uint8_t IRQ, uint8_t priority);

void I2C_irqintconfig(uint8_t IRQ, uint8_t enad);

uint8_t I2C_flag(I2C_Reg *Pi2cr, uint8_t flagname);

void I2Capplicationeventcallback(I2C_handle *pi2coh, uint8_t event);

void mastersenddata(I2C_handle *pi2coh, uint8_t len, uint16_t *txbuffer,uint8_t addr);

uint8_t mastersenddatatr(I2C_handle *pi2coh, uint8_t len, uint16_t *txbuffer,uint8_t addr, uint8_t sr);

uint8_t mastersenddatarr(I2C_handle *pi2coh, uint8_t len, uint16_t *txbuffer,uint8_t addr,uint8_t sr);

void i2cinteruptconfig();
void i2cprorityconfig();

void i2c_ehandling(I2C_handle *pi2coh);
void i2c_errhandling(I2C_handle *pi2coh);
