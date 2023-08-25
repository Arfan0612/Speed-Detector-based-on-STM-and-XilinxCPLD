/*
 * comparator.c
 *
 *  Created on: Dec 18, 2022
 *      Author: Arfan Danial
 */

#include "comparator.h"
uint32_t comp_frequency = 0;
uint32_t T1 = 0;
uint32_t T2 = 0;
uint16_t TIM16_OVC = 0;
uint8_t state = 0;
uint32_t ticks = 0;

void rising_edge_trigger(COMP_HandleTypeDef *hcomp, COMP_HandleTypeDef *hcomp1, TIM_HandleTypeDef* htim16){
	//if the trigger callback is caused by comparator 1
	if (hcomp == hcomp1){
		if(state == 0)
		{
			T1 = __HAL_TIM_GET_COUNTER(htim16);
			TIM16_OVC = 0;
			state = 1;
		}
		else if(state == 1)
		{
			T2 = __HAL_TIM_GET_COUNTER(htim16);
			ticks = (T2+(TIM16_OVC*65536)) - T1;
			//ticks = (T2 + (TIM2_OVC * 65536)) - T1;
			comp_frequency = (uint32_t)round(1/(CLK_Period*ticks));
			state = 0;
		}
	}
}

void Update_rollover(TIM_HandleTypeDef* htim, TIM_HandleTypeDef* htim16){
	if(htim==htim16){
		TIM16_OVC++;
	}
}

uint32_t get_comp_frequency(){
	return comp_frequency; //just for testing NOT actual return value
}
