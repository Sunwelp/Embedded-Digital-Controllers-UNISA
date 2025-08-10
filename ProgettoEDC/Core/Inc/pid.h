/*
 * pid.h
 *
 *  Created on: Jul 22, 2024
 *      Author: User
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include "tim.h"

// Struttura per il controller PID
struct pid{
	double Kp;        // Guadagno proporzionale
    double Ki;        // Guadagno integrale
    double Kd;        // Guadagno derivativo
    double prevError; // Errore precedente
    double integral;  // Parte integrale
    double setpoint;  // Setpoint
    double last_u;
    double u;
    uint8_t IDController;
};

typedef struct pid PID_t;

void PID_Init(PID_t *, double, double, uint8_t);
void PID_Compute(PID_t *, double);
void PWM_generate (double , TIM_HandleTypeDef *, uint16_t);
void switchKcontrol (PID_t *, PID_t *, PID_t *, PID_t *, PID_t *, PID_t *);
void automatic_switch_control (PID_t *, PID_t *, PID_t *, PID_t *, PID_t *, PID_t *);
double rpm_to_voltage(int );

#endif /* INC_PID_H_ */
