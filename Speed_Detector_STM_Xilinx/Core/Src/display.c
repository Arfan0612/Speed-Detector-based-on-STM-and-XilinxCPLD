/*
 * display.c
 *
 *  Created on: Dec 18, 2022
 *      Author: Arfan Danial
 */
#include "display.h"

char LCD_1st_Row[50] = "\0"; //string for used for 1st row of LCD
char LCD_2nd_Row[50] = "\0"; //string for used for 2nd row of LCD

//speed_unit = 1 is M/S
//speed_unit = 2 is KM/H
//speed_unit = 3 is MPH
int speed_unit = 1;

//mode = 0 is Menu
//mode = 1 is ADC
//mode = 2 is COMP
int mode = 0;

//to store speed for TX
uint8_t v_xilinx = 0;

void LCD_init(GPIO_TypeDef* port_rs, uint16_t rs_pin, GPIO_TypeDef* port_e, uint16_t e_pin,
		   GPIO_TypeDef* port_d4, uint16_t d4_pin, GPIO_TypeDef* port_d5, uint16_t d5_pin, GPIO_TypeDef* port_d6, uint16_t d6_pin, GPIO_TypeDef* port_d7, uint16_t d7_pin,
		   GPIO_TypeDef* port_d10, uint16_t d10_pin){
	lcd16x2_init_4bits(GPIOA, RS_Pin, GPIOC, E_Pin, GPIOB, D4_Pin, GPIOB, D5_Pin, GPIOB, D6_Pin, GPIOA, D7_Pin, GPIOB, D10_Pin);
}

void keypad_read_key(uint32_t adc_readout, UART_HandleTypeDef *huart){
	//for when up button is pressed
	if (adc_readout > 700 && adc_readout < 800){
		//reset button to show menu
		mode=0;
	}

	//for when down button is pressed
	else if (adc_readout > 1700 && adc_readout < 1900 ){
		//change the speed unit each time the down button is pressed
		if(speed_unit==3){
			//reset back to M/S
			speed_unit=1;
		}
		else{
			speed_unit++;
		}
	}

	//for when left button is pressed
	else if (adc_readout > 2000 && adc_readout < 3000){
		//change to ADC mode
		mode = 1;
	}

	//for when right button is pressed
	else if (adc_readout == 0){
		//change to COMP mode
		mode = 2;
	}

	checking();
	Output(huart);
}

void checking(){
	int frequency = 0;

	//get the frequency of the current mode used
	frequency = cutOff();

	//check for which mode the LCD is current in
	if(mode==0){
		//main menu
		sprintf(LCD_1st_Row, "Press button:");
		sprintf(LCD_2nd_Row, "L=ADC | R=COMP");
		setTX(0); //set data transmit to Xilinx to be 0
	}
	else if(mode==1){
		//adc
		sprintf(LCD_1st_Row, "ADC F: %u Hz",frequency);
		printing_speed(frequency);
	}
	else if(mode==2){
		//comp
		sprintf(LCD_1st_Row, "COMP F: %u Hz",frequency);
		printing_speed(frequency);
	}
}

int cutOff(){
	int frequency = 0;

	//getting ADC frequency
	if(mode==1){
		frequency = get_adc_frequency();
	}
	//getting COMP frequency
	else if(mode==2){
		frequency = get_comp_frequency();
	}

	//further filtering to only output frequency within BW
	if(frequency>=100 && frequency<=600){
		//if within BW, return the detected frequency
		return frequency;
	}
	else{
		//if out of BW, return 0 Hz
		return 0;
	}
}

void printing_speed(int frequency){
	if(speed_unit==1){
		MPS(frequency);
	}
	else if(speed_unit==2){
		KPH(frequency);
	}
	else if(speed_unit==3){
		MPH(frequency);
	}
}

void KPH(int frequency){
	float v_kph = ((c_kph)*((float)frequency/(2*f_transmit)));
	sprintf(LCD_2nd_Row, "V: %.2f km/h",v_kph); //-u_printf_float enabled
	setTX(round(v_kph)); //set TX value for transmission to Xilinx
}

void MPS(int frequency){
	float v_mps = ((c_mps)*((float)frequency/(2*f_transmit)));
	sprintf(LCD_2nd_Row, "V: %.2f m/s",v_mps); //-u_printf_float enabled
	setTX(round(v_mps)); //set TX value for transmission to Xilinx
}

void MPH(int frequency){
	float v_mph = ((c_mph)*((float)frequency/(2*f_transmit)));
	sprintf(LCD_2nd_Row, "V: %.2f mph",v_mph); //-u_printf_float enabled
	setTX(round(v_mph)); //set TX value for transmission to Xilinx
}

void setTX(int v){
	//set a limiter so that the xilinx never output above the value 99 in any speed unit
	if (v > 99) {
		v = 99;
	}

	//convert the speed unit to BCD before transmission
	v_xilinx = Decimal_To_BCD((uint8_t) v);
}

void Output(UART_HandleTypeDef *huart) {
	lcd16x2_clear();
	lcd16x2_1stLine();
	lcd16x2_printf(LCD_1st_Row);
	lcd16x2_2ndLine();
	lcd16x2_printf(LCD_2nd_Row);
	HAL_UART_Transmit (huart, &v_xilinx, 1, 1);
	HAL_Delay(400);
}

