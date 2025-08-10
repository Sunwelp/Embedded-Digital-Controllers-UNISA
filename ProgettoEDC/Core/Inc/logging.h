/*
 * logging.h
 *
 *  Created on: Jul 26, 2024
 *      Author: User
 */

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include "stdbool.h"
#include "math.h"
#include "string.h"
#include  <errno.h>
#include  <sys/unistd.h>
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"

struct circular_buffer {
	void *buffer;     // data buffer
	void *buffer_end; // end of data buffer
	size_t capacity;  // maximum number of items in the buffer
	size_t count;     // number of items in the buffer
	size_t sz;        // size of each item in the buffer
	void *head;       // pointer to head
	void *tail;       // pointer to tail
	bool writing;  // signals if the buffer is being written
};

typedef struct circular_buffer circular_buffer_t;


struct record {
	double current_u_rf; // value of the current controller output
	double current_u_lf; // value of the current controller output
	double current_u_rr; // value of the current controller output
	double current_u_lr; // value of the current controller output
	double current_u_rax; // value of the current controller output
	double current_u_fax; // value of the current controller output
	double current_y_rf; // value of the current motor output (speed)
	double current_y_lf; // value of the current motor output (speed)
	double current_y_rr; // value of the current motor output (speed)
	double current_y_lr; // value of the current motor output (speed)
	double current_y_rax; // value of the current motor output (speed)
	double current_y_fax; // value of the current motor output (speed)
	uint32_t cycleCoreDuration; // time needed to read, compute and actuate
	uint32_t cycleBeginDelay; // difference between the actual and the expected absolute start time of the cycle
	uint32_t currentTimestamp; // current timestamp in millis
};

typedef struct record record_t;


void cb_init(circular_buffer_t *, size_t, size_t);
void cb_free(circular_buffer_t *);
void cb_push_back(circular_buffer_t *, const void *);
void cb_pop_front(circular_buffer_t *, void *);

int _write(int, char *, int);

#endif /* INC_LOGGING_H_ */
