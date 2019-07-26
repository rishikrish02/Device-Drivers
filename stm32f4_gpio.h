/*
 * stm32f4_gpio.h
 *
 *  Created on: May 17, 2019
 *      Author: rishi
 */
#include "stm32f407xx.h"

#ifndef INC_STM32F4_GPIO_H_
#define INC_STM32F4_GPIO_H_
//this is created so that when user interacts with the config it calls the driver to let know of it
typedef struct
{
	uint8_t GPIO_pinnumber;
	uint8_t GPIO_pinmode;                 //<*@gpiopinmodes*>/
	uint8_t GPIO_pinspeed;
	uint8_t GPIO_pincontrol;
	uint8_t GPIO_pinoptype;
	uint8_t GPIO_pinALTfuncmode;



}GPIO_pinconfig;


typedef struct
{
	GPIO_reg *pgpio;
	GPIO_pinconfig  *pgpioconfig;
	EXTIP_REG *EXT;

}GPIO_hanlder;


//gpio_pinmode possible values
#define GPIO_MODE_in   		0
#define GPIO_MODE_out  		1
#define GPIO_MODE_altfun	2
#define GPIO_MODE_analog    3
#define GPIO_MODE_rt	    4  // rising edge trigger
#define GPIO_MODE_ft	    5	// falling edge trigger
#define GPIO_MODE_rft	    6	//rising edge and falling edge trigger

//gpio_output types
#define GPIO_op				0
#define GPIO_OPN			1

//GPIO speed
#define GPIO_Lowspeed		0
#define GPIO_medspeed		1
#define GPIO_highspeed		2
#define GPIO_vhighspeed		3

//GPIO pull up/down
#define GPIO_pullup 		0
#define GPIO_pulldown       1
#define GPIO_nopullupdown   2

//GPIO pin

#define gpio_pin_no_1 		1
#define gpio_pin_no_2 		2
#define gpio_pin_no_3		3
#define gpio_pin_no_4 		4
#define gpio_pin_no_5 		5
#define gpio_pin_no_6 		6
#define gpio_pin_no_7 		7
#define gpio_pin_no_8 		8
#define gpio_pin_no_9		9
#define gpio_pin_no_10 		10
#define gpio_pin_no_11 		11
#define gpio_pin_no_12 		12
#define gpio_pin_no_13 		13
#define gpio_pin_no_14 		14
#define gpio_pin_no_15 		15
#define gpio_pin_no_16 		16
void GPIO_init(GPIO_hanlder *pgpioh); // to initialize

void GPIO_deinit(GPIO_reg *pgpio); //to reset
uint8_t GPIO_readpin(GPIO_reg *pgpio, uint8_t pinnumber);  // read pin
void GPIO_writepin(GPIO_reg *pgpio, uint8_t pinnumber, uint8_t value);   // write pin
uint16_t GPIO_readport(GPIO_reg *pgpio, uint8_t pinnumber);    //read port
void GPIO_writeport(GPIO_reg *pgpio, uint8_t value);//write port
void GPIO_peripheralcock(GPIO_reg *pgpiox, uint8_t ena);
void GPIO_TOGGLE(GPIO_reg *pgpiox, uint8_t pinnumber);
void GPIO_irqconfig(uint8_t IRQ, uint8_t priority, uint8_t, ena);
void GPIO_irqhandler(uint8_t pinumber);

#endif /* INC_STM32F4_GPIO_H_ */
