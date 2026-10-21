/*
 * sensorup.c
 *
 *  Created on: Oct 20, 2026
 *      Author: Arnav Patil
 */
#include "sensorup.h"

#include "main.h"
#include <stdlib.h>
#include <stdint.h>


uint16_t senval[8];

void senup(){

	    for (int i = 0; i < 7; i++)
	    {
	        senval[i] =
	            (GPIOA->IDR & (1U << (i + 1U))) != 0U;
	    }

	    senval[7] =
	        (GPIOB->IDR & (1U << 0U)) != 0U;
	}
