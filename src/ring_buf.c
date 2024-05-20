#include "../inc/ring_buf.h"

void insert_val(float accel, ring_buf* b) {
    if (b->curr_index < BUF_SIZE) {
		b->buffer[b->curr_index] = accel;
		b->curr_index++;
	} else { //curr_index >= 10
		b->curr_index = 0;
		b->buffer[b->curr_index] = accel;
		b->curr_index++;
	}
}

float get_rolling_avg(ring_buf* b) {
    float average = 0;
	for (int i = 0; i < BUF_SIZE; i++) {
		average += b->buffer[i];
	}
	return (average / BUF_SIZE);
}