/*
 * logging.c
 *
 *  Created on: Jul 26, 2024
 *      Author: User
 */


#include "logging.h"


void cb_init(circular_buffer_t *cb, size_t capacity, size_t sz) {
	cb->buffer = calloc(capacity, sz);
	if (cb->buffer == NULL)
		printf("ALLOCATED NULL\n\r");
	// handle error
	cb->buffer_end = (char*) cb->buffer + capacity * sz;
	cb->capacity = capacity;
	cb->count = 0;
	cb->sz = sz;
	cb->head = cb->buffer;
	cb->tail = cb->buffer;
	cb->writing = false;

}

void cb_free(circular_buffer_t *cb) {
	free(cb->buffer);
    cb->buffer = NULL;
    cb->buffer_end = NULL;
    cb->capacity = 0;
    cb->count = 0;
    cb->sz = 0;
    cb->head = NULL;
    cb->tail = NULL;
    cb->writing = false;
}

void cb_push_back(circular_buffer_t *cb, const void *item) {
	if (cb->count == cb->capacity) {
		printf("ERROR PUSH BACK \n\r");
		// handle error
		return;
	}
	cb->writing = 1;
	memmove(cb->head, item, cb->sz);
	cb->head = (char*) cb->head + cb->sz;
	if (cb->head == cb->buffer_end)
		cb->head = cb->buffer;
	cb->count++;
	cb->writing = 0;
}

void cb_pop_front(circular_buffer_t *cb, void *item) {
	if (cb->count == 0) {
		printf("ERROR POP FRONT \n\r");
		// handle error
		return;
	}
	memmove(item, cb->tail, cb->sz);
	cb->tail = (char*) cb->tail + cb->sz;
	if (cb->tail == cb->buffer_end)
		cb->tail = cb->buffer;
	while ((cb->writing))
		;
	cb->count--;
}


/* BEGIN USART WRITE FUNCTION (used by printf)*/
int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) data, len, 1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}
