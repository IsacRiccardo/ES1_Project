#include <MKL25Z4.h>

#include "Recorder.h"
#include "i2c.h"
#include "mma8451.h"
#include "Filter.h"


static int16_t recorder_buffer[MAX_BUFFER_SIZE];
static volatile uint8_t recording;
static volatile uint8_t use_x;
static volatile uint8_t use_y;
static volatile uint8_t use_z;
static volatile uint32_t recorder_index;

static filter_data_t filter16, filter64;


void RecorderInit(uint8_t x, uint8_t y, uint8_t z) {
	use_x = x;
	use_y = y;
	use_z = z;
	recorder_index = 0;
	recording = 0;
	Filter_Init(&filter16, 4, 16);
	Filter_Init(&filter64, 6, 64);
}

void RecorderStart(void) {
	recorder_index = 0;
	recording = 1;
}

void RecorderCallback(void) {
	if(recording) {
		read_full_xyz();
		if(use_x)
			recorder_buffer[recorder_index++] = acc_Z;
		if(use_y)
			recorder_buffer[recorder_index++] = Filter_Run(&filter16, acc_Z);
		if(use_z)
			recorder_buffer[recorder_index++] = Filter_Run(&filter64, acc_Z);
		if(recorder_index >= MAX_BUFFER_SIZE)
			recording = 0;	
	}	
}


uint8_t RecorderFinished(void) {
	return !recording;
}

uint8_t* RecorderGetBuffer(void) {
	return (uint8_t*)recorder_buffer;
}

uint32_t RecorderGetSize(void) {
	return MAX_BUFFER_SIZE;
}
