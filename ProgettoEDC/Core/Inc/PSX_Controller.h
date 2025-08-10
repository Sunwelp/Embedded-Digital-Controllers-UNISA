/*
 * PSX_Controller.h
 *
 *  Created on: Feb 21, 2024
 *      Author: User
 */

#ifndef INC_PSX_CONTROLLER_H_
#define INC_PSX_CONTROLLER_H_

#include "spi.h"
#include "gpio.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

struct controller_s {
	uint8_t PSX_AD;
	uint8_t PSX_MOD;
	double speed;
};
typedef struct controller_s controller_t;

extern controller_t PSX;

void PSX_Init();
void GET_PSX_DATA(uint8_t *, uint8_t *);
void DATA_DIGEST(uint8_t *);

#endif /* INC_PSX_CONTROLLER_H_ */
