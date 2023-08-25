/*
 * Decimal_BCD_converter.c
 *
 *  Created on: Feb 22, 2023
 *      Author: Arfan Danial
 */

#include "Decimal_BCD_converter.h"

uint8_t Decimal_To_BCD(uint8_t decimal){
	//make sure the format for transmitting is similar to this
	//HAL_UART_Transmit_DMA(&huart4,&BCD,sizeof(BCD));

	uint8_t MSB=0, LSB=0, BCD_8bit=0;

	//get the MSB
	//divide by 10 to get the MSB
	MSB = decimal/10;

	//shift the MSB by 4 bits to get (4 bit BCD of MSB) with 0000
	//Example MSB=1, we shift 4 bits to get 00010000
	MSB = MSB<<4;

	//get the LSB
	//do %10 to get the LSB
	LSB = decimal%10;

	//get the decimal representation of the 8-bit BCD conversion
	//adding the MSB with LSB by using OR bit operation
	BCD_8bit = MSB | LSB;

	//return the new decimal that represents 8-bit of the input argument's decimal
	return BCD_8bit;
}


