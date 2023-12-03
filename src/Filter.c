#include "Filter.h"


void Filter_Init(filter_data_t *filter, int divider, int max_index) {
	for(int i = 0; i < FILTER_MAX_SIZE; i++) {
		filter->buffer[i] = 0;
	}
	filter->divider = divider;
	filter->max_index = max_index;
	if(filter->max_index > FILTER_MAX_SIZE) {
		filter->max_index = FILTER_MAX_SIZE;
	}
	filter->index = 0;
	filter->accumulator = 0;
}


short Filter_Run(filter_data_t *filter, short new_value) {
	short old_value = filter->buffer[filter->index];
	filter->buffer[filter->index] = new_value;
	filter->index++;
	if(filter->index >= filter->max_index) {
		filter->index = 0;
	}
	filter->accumulator -= old_value;
	filter->accumulator += new_value;
	return (short)(filter->accumulator >> filter->divider);
}

