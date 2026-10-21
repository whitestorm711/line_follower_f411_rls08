/*
 * motordrive.h
 *
 *  Created on: Oct 20, 2026
 *      Author: Arnav Patil
 */
#include "main.h"


#ifndef SRC_MOTORDRIVE_H_
#define SRC_MOTORDRIVE_H_



void LF_SetMotorSpeed(uint16_t left, uint16_t right);
void LF_SetMotorDirection(uint8_t left_forward, uint8_t right_forward);

#endif /* SRC_MOTORDRIVE_H_ */
