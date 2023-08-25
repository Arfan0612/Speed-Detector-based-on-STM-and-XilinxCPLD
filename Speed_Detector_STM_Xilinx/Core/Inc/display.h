
/*
 * display.h
 *
 *  Created on: Dec 18, 2022
 *      Author: Arfan Danial
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

//defining the transmitted frequency of Doppler Radar Module
#define f_transmit 10525000000.0

//defining speed of light in different units
#define c_mph 671080887.616
#define c_mps 300000000.0
#define c_kph 1080000000.0

#include "main.h"
#include "lcd16x2_v2.h"
#include "comparator.h"
#include "Decimal_BCD_Converter.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

void LCD_init(GPIO_TypeDef* port_rs, uint16_t rs_pin, GPIO_TypeDef* port_e, uint16_t e_pin,
		   GPIO_TypeDef* port_d4, uint16_t d4_pin, GPIO_TypeDef* port_d5, uint16_t d5_pin, GPIO_TypeDef* port_d6, uint16_t d6_pin, GPIO_TypeDef* port_d7, uint16_t d7_pin,
		   GPIO_TypeDef* port_d10, uint16_t d10_pin);

void keypad_read_key(uint32_t adc_readout, UART_HandleTypeDef *huart);

void checking();

int cutOff();

void printing_speed(int frequency);

void KPH(int frequency);

void MPS(int frequency);

void MPH(int frequency);

void setTX(int v);

void Output(UART_HandleTypeDef *huart);

#endif /* INC_DISPLAY_H_ */
