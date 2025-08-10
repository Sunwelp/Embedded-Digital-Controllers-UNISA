#include "encoder.h"
#include <math.h>
#include <stdlib.h>

/**
 * @brief Private function that obtain RPM from count, it considers the gearbox
 *
 * @param encoder encoder's reference
 * @return int8_t Status code ENCODER_OK if successful, ENCODER_ERR otherwise.
 */

/**
 * @brief Function to initialize the encoder.
 *
 * @param encoder Pointer to the encoder_t struct.
 * @param htim Pointer to the timer's handler.
 * @return int8_t Status code ENCODER_OK if successful, ENCODER_ERR otherwise.
 */
void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *htim) {
	if (encoder) {
		encoder->htim = htim;
		encoder->dir = 0;
		encoder->cnt = 0;
		encoder->last_tick = 0;
		encoder->current_tick = 0;
		encoder->speed = 0;
	}
}


double getTicksDelta(double currentTicks, double lastTicks, double Ts) {
	double delta;

	if (abs(currentTicks - lastTicks) <= ceil(6936 * Ts))
		delta = currentTicks - lastTicks;
	else {
		if (lastTicks > currentTicks)
			delta = currentTicks + pow(2, 16) - 1 - lastTicks;
		else
			delta = currentTicks - pow(2, 16) + 1 - lastTicks;
	}
	return delta;
}

double getSpeedByDelta(double ticksDelta, double Ts) {
	return ticksDelta * 60 / (48 * 51 * Ts);
}


