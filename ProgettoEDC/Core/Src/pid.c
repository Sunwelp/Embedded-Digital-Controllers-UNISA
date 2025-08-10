/*
 * pid.c
 *
 *  Created on: Jul 22, 2024
 *      Author: User
 */

#include "pid.h"
#include "math.h"

#define max_out (12)
#define min_out (-12)

extern char msg[100];

double val=1574.25;


void PID_Init(PID_t *pid, double Kp, double Ki, uint8_t ID) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = 0.0;
    pid->prevError = 0.0;
    pid->integral = 0.0;
    pid->setpoint = 0.0;
    pid->last_u = 0.0;
    pid->u = 0.0;
    pid->IDController = ID;
}



void PID_Compute(PID_t *pid, double currentValue) {
    // Calcolo dell'errore
    double error = pid->setpoint - currentValue;

    // Calcolo della parte proporzionale
    double Pout = pid->Kp * error;

    // Calcolo della parte derivativa
    double derivative = (error - pid->prevError);
    double Dout = pid->Kd * derivative;

    // Calcolo della parte integrale
    if ((pid->last_u < max_out && error > 0)||(pid->last_u > min_out && error < 0)){ //antiwindup
    	pid->integral += error;
    }
    double Iout = pid->Ki* pid->integral;

    // Output totale
        pid->u = Pout + Iout + Dout;

    // Applica il limite all'output (saturazione)
    if (pid->u > max_out) {
    	pid->u = max_out;
    }
    else if (pid->u < min_out) {
    	pid->u = min_out;
    }

    // Salvataggio dell'errore per il prossimo calcolo
    pid->prevError = error;

    pid->last_u = pid->u;
}
/*
void automatic_switch_control( PID_t * pid_RF, PID_t * pid_LF, PID_t * pid_RR, PID_t * pid_LR, PID_t * pid_FAx, PID_t * pid_RAx){

	if(fabs(pid_RF->prevError) >= 180 && pid_RF->IDController == 0){ //Passo al controllo Slow
		//aggiorno i guadagni proporzionali
		pid_RF->Kp = 0.019625;
		pid_LF->Kp = 0.021725;
		pid_RR->Kp = 0.024046;
		pid_LR->Kp = 0.020418;
		pid_FAx->Kp = 0.021725;
		pid_RAx->Kp = 0.020418;

		//aggiorno la somma integrale
		pid_RF->integral = pid_RF->integral * pid_RF->Ki/0.002588;
		pid_LF->integral = pid_LF->integral * pid_LF->Ki/0.00244;
		pid_RR->integral = pid_RR->integral * pid_RR->Ki/0.002703;
		pid_LR->integral = pid_LR->integral * pid_LR->Ki/0.002516;
		pid_FAx->integral = pid_FAx->integral * pid_FAx->Ki/0.00244;
		pid_RAx->integral = pid_RAx->integral * pid_RAx->Ki/0.002516;

		//aggiorno i guadagni integrali
		pid_RF->Ki = 0.002588;
		pid_LF->Ki = 0.00244;
		pid_RR->Ki = 0.002703;
		pid_LR->Ki = 0.002516;
		pid_FAx->Ki = 0.00244;
		pid_RAx->Ki = 0.002516;

		pid_RF->IDController = 1;
		pid_LF->IDController = 1;
		pid_RR->IDController = 1;
		pid_LR->IDController = 1;
		pid_FAx->IDController = 1;
		pid_RAx->IDController = 1;
	}

	else{  //Passo al controllo Fast
		//aggiorno i guadagni proporzionali
		pid_RF->Kp = 0.054164;
		pid_LF->Kp = 0.059952;
		pid_RR->Kp = 0.066368;
		pid_LR->Kp = 0.056354;
		pid_FAx->Kp = 0.059952;
		pid_RAx->Kp = 0.056354;

		//aggiorno la somma integrale
		pid_RF->integral = pid_RF->integral * pid_RF->Ki/0.0071;
		pid_LF->integral = pid_LF->integral * pid_LF->Ki/0.0067;
		pid_RR->integral = pid_RR->integral * pid_RR->Ki/0.0075;
		pid_LR->integral = pid_LR->integral * pid_LR->Ki/0.0069;
		pid_FAx->integral = pid_FAx->integral * pid_FAx->Ki/0.0067;
		pid_RAx->integral = pid_RAx->integral * pid_RAx->Ki/0.0069;

		//aggiorno i guadagni integrali
		pid_RF->Ki = 0.0071;
		pid_LF->Ki = 0.0067;
		pid_RR->Ki = 0.0075;
		pid_LR->Ki = 0.0069;
		pid_FAx->Ki = 0.0067;
		pid_RAx->Ki = 0.0069;

		pid_RF->IDController = 0;
		pid_LF->IDController = 0;
		pid_RR->IDController = 0;
		pid_LR->IDController = 0;
		pid_FAx->IDController = 0;
		pid_RAx->IDController = 0;
	}
}
*/

void switchKcontrol (PID_t *pid_RF, PID_t * pid_LF, PID_t * pid_RR, PID_t * pid_LR, PID_t * pid_FAx, PID_t * pid_RAx){
	if(pid_RF->IDController == 1){ //passiamo al controllo fast
		//aggiorno i guadagni proporzionali
		pid_RF->Kp = 0.054164;
		pid_LF->Kp = 0.059952;
		pid_RR->Kp = 0.066368;
		pid_LR->Kp = 0.056354;
		pid_FAx->Kp = 0.059952;
		pid_RAx->Kp = 0.056354;

		//aggiorno la somma integrale
		pid_RF->integral = pid_RF->integral * pid_RF->Ki/0.0071;
		pid_LF->integral = pid_LF->integral * pid_LF->Ki/0.0067;
		pid_RR->integral = pid_RR->integral * pid_RR->Ki/0.0075;
		pid_LR->integral = pid_LR->integral * pid_LR->Ki/0.0069;
		pid_FAx->integral = pid_FAx->integral * pid_FAx->Ki/0.0067;
		pid_RAx->integral = pid_RAx->integral * pid_RAx->Ki/0.0069;

		//aggiorno i guadagni integrali
		pid_RF->Ki = 0.0071;
		pid_LF->Ki = 0.0067;
		pid_RR->Ki = 0.0075;
		pid_LR->Ki = 0.0069;
		pid_FAx->Ki = 0.0067;
		pid_RAx->Ki = 0.0069;

		pid_RF->IDController = 0;
		pid_LF->IDController = 0;
		pid_RR->IDController = 0;
		pid_LR->IDController = 0;
		pid_FAx->IDController = 0;
		pid_RAx->IDController = 0;

	}
	else if( pid_RF->IDController == 0){ //passiamo al controllo slow
		pid_RF->Kp = 0.019625;
		pid_LF->Kp = 0.021725;
		pid_RR->Kp = 0.024046;
		pid_LR->Kp = 0.020418;
		pid_FAx->Kp = 0.021725;
		pid_RAx->Kp = 0.020418;

		//aggiorno la somma integrale
		pid_RF->integral = pid_RF->integral * pid_RF->Ki/0.002588;
		pid_LF->integral = pid_LF->integral * pid_LF->Ki/0.00244;
		pid_RR->integral = pid_RR->integral * pid_RR->Ki/0.002703;
		pid_LR->integral = pid_LR->integral * pid_LR->Ki/0.002516;
		pid_FAx->integral = pid_FAx->integral * pid_FAx->Ki/0.00244;
		pid_RAx->integral = pid_RAx->integral * pid_RAx->Ki/0.002516;

		//aggiorno i guadagni integrali
		pid_RF->Ki = 0.002588;
		pid_LF->Ki = 0.00244;
		pid_RR->Ki = 0.002703;
		pid_LR->Ki = 0.002516;
		pid_FAx->Ki = 0.00244;
		pid_RAx->Ki = 0.002516;

		pid_RF->IDController = 1;
		pid_LF->IDController = 1;
		pid_RR->IDController = 1;
		pid_LR->IDController = 1;
		pid_FAx->IDController = 1;
		pid_RAx->IDController = 1;
	}

}

// Funzione per mappare la velocità in RPM nella tensione
double rpm_to_voltage(int rpm) {
    // Coefficiente di conversione (0.075 V/RPM)
    const double conversion_factor = 0.075;

    // Calcolo della tensione corrispondente
    double voltage = conversion_factor * rpm;

    return voltage;
}


void PWM_generate (double voltage, TIM_HandleTypeDef *htim, uint16_t channel){

	if(voltage>12)
		voltage=12;
	if(voltage<-12)
		voltage=-12;

    voltage = 2.5 + (voltage)*0.625/12;
	val = (voltage/3.3)*((double)htim->Init.Period);
	__HAL_TIM_SET_COMPARE(htim, channel, val);

}
