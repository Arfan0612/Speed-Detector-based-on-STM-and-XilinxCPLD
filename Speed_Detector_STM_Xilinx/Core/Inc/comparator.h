/*
 * comparator.h
 *
 *  Created on: Dec 18, 2022
 *      Author: Arfan Danial
 */

#ifndef INC_COMPARATOR_H_
#define INC_COMPARATOR_H_
#define CLK_Period 0.000001 //after prescaler, the frequency of timer is 1220.703 Hz

#include "main.h"
#include "math.h"
#include <stdio.h>
void rising_edge_trigger(COMP_HandleTypeDef *hcomp, COMP_HandleTypeDef *hcomp1, TIM_HandleTypeDef* htim16);
void Update_rollover(TIM_HandleTypeDef* htim, TIM_HandleTypeDef* htim16);
uint32_t get_comp_frequency();
#endif /* INC_COMPARATOR_H_ */
