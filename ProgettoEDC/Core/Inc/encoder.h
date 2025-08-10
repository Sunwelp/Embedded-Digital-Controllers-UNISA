
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "gpio.h"
#include "tim.h"

/**
 * @brief Status code for successful encoder operations.
 */
#define ENCODER_OK				(0)

/**
 * @brief Status code for failed encoder operations.
 */
#define ENCODER_ERR				(-1)

/**
 * @brief Counts per revolution for the encoder sensor.
 */
#define ENCODER_CPR				(48)

/**
 * @brief Maximum value for the encoder count.
 */
#define ENCODER_MAX_VAL			(0xffff)

/**
 * @brief Maximum tick value for the encoder.
 */
#define ENCODER_MAX_TICK_VAL 	(0xffff)

/**
 * @brief Gear ratio for the encoder sensor.
 */
#define ENCODER_GEAR_RATIO		(51)

/**
 * @brief Time constant for the encoder sensor.
 */
#define ENCODER_TIME_CONSTANT	(60000)

/**
 * @brief Minimum waiting time for the encoder sensor.
 */
#define ENCODER_MIN_WAITING_TIME		(1)

/**
 * @brief Forward direction value
 */
#define ENCODER_FORWARD_DIRECTION	(1)

/**
 * @brief Backward direction value
 */
#define ENCODER_BACKWARD_DIRECTION	(0)

/**
 * @brief Struct to hold the information of the encoder sensor.
 */
struct encoder_s {
	TIM_HandleTypeDef *htim;		//timer's handler
	uint8_t dir;					//rotation direction read (0 or 1)
	uint16_t cnt;					//values of encoder's count
	double last_tick;					//values of ticks
	double current_tick;
	double speed;				//rotation speed read from the sensor, in RPM
};

typedef struct encoder_s encoder_t;

/**
 * @brief Function to initialize the encoder.
 *
 * @return int8_t Status code ENCODER_OK if successful, ENCODER_ERR otherwise.
 */
void encoder_init(encoder_t*, TIM_HandleTypeDef*);

double getTicksDelta(double, double, double);

double getSpeedByDelta(double, double);

#endif /* INC_ENCODER_H_ */
