#ifndef _FILTER_H
#define _FILTER_H

#define FILTER_MAX_SIZE 128

typedef struct {
	short buffer[FILTER_MAX_SIZE];
	int index;
	int max_index;
	int divider;
	int accumulator;
} filter_data_t;

void Filter_Init(filter_data_t *filter, int divider, int max_index);
short Filter_Run(filter_data_t *filter, short new_value);

#endif //_FILTER_H 
