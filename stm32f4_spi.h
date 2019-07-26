/*
  * stm32f4_spi.h
 *
 *  Created on: Jun 2, 2019
 *      Author: rishi
 */

#include <stm32f407xx.h>
#ifndef INC_STM32F4_SPI_H_
#define INC_STM32F4_SPI_H_


typedef struct
{
	uint8_t SPI_mode;
	uint8_t SPI_bus_config;
	uint8_t SPI_Dff;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_CLK;
}SPI_CONFIG;

typedef struct
{
	SPI_CONFIG *pspio;
	SPI_REG  *pspiox;
	uint8_t	*ptxbuffer;
	uint8_t *prxbuffer;
	uint8_t txstate;
	uint8_t rxstate;
	uint8_t txlength;
	uint8_t rxlength;
}SPI_handle;


#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

#define FULLD		0
#define HALFD		1

#define SIMPLEXR	2

#define SPI_CLK_SPEED2		0
#define SPI_CLK_SPEED4		1
#define SPI_CLK_SPEED8		2
#define SPI_CLK_SPEED16		3
#define SPI_CLK_SPEED32		4
#define SPI_CLK_SPEED64		5
#define SPI_CLK_SPEED128	6
#define SPI_CLK_SPEED256	7

#define SPI_SIZE8 		0
#define SPI_SIZE16		1

#define CPOL_LOW		0
#define CPOH_HIGH		1

#define CPHA_LOW		0
#define CPHA_HIGH		1

#define SSM_HW		0
#define SSM_SW		1
// peripheral and init clock setup

static void spi_txe_interupthanlder(SPI_handle *pspioh);
static void spi_rxe_interupthanlder(SPI_handle *pspioh);

void SPI_init(SPI_handle *pspioh); // to initialize

void SPI_deinit(SPI_REG *pspiox); //to reset

void SPI_peripheralcock(SPI_REG *pspiox, uint8_t ena);
//--- data send and receive for spi--------------------
void SPI_sendata(SPI_REG *pspiox, uint32_t len, uint8_t *ptxbuffer);
void SPI_recdata(SPI_REG *pspiox, uint32_t len, uint8_t *prxbuffer);
//interupt base//
void SPI_sendatair(SPI_handle *pspioh, uint32_t len, uint8_t *ptxbuffer);
void SPI_recdatair(SPI_handle *pspioh, uint32_t len, uint8_t *prxbuffer);

void SPI_irqconfig(uint8_t IRQ, uint8_t priority);
void SPI_irqhandler(SPI_handle *pspioh);
void SPI_irqintconfig(uint8_t IRQ, uint8_t enad);
void spiclearovr(SPI_REG *pspiox);
void spiclosetransmissions(SPI_handle *pspioh);
void spiclosereception(SPI_handle *pspioh);
void SPIapplicationeventcallback(SPI_handle *pspioh, uint8_t event);
#define txeflag    (1<<1)
#define rxeflag 	(1<<0)

//spi api status

#define spiready    0
#define spitxbusy 	1
#define spirxbusy   2

#define txcomplete 	1
#define rxcomplete 	2
#define ovrcomplete 3

#endif /* INC_STM32F4_SPI_H_ */
