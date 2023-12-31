#include <MKL25Z4.h>
#include "mma8451.h"
#include "i2c.h"
#include <math.h>

int16_t acc_X=0, acc_Y=0, acc_Z=0;
int roll=0, pitch=0;

//mma data ready
extern uint32_t DATA_READY;

void Delay (uint32_t);

//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	  //check for device
		if(i2c_read_byte(MMA_ADDR, REG_WHOAMI) == WHOAMI)	{
			Delay(10);
			//set active mode, 14 bit samples and 100 Hz ODR (0x19)
			
			i2c_write_byte(MMA_ADDR, REG_CTRL1, 0x19); //0x01 800Hz
			return 1;
		}
		//else error
		return 0;
}

void read_full_xyz()
{
	int i;
	uint8_t data[6];
	int16_t temp[3];
	
	i2c_start();
	i2c_read_setup(MMA_ADDR , REG_XHI);
	
	// Read five bytes in repeated mode
	for( i=0; i<5; i++)	{
		data[i] = i2c_repeated_read(0);
	}
	// Read last byte ending repeated mode
	data[i] = i2c_repeated_read(1);
	
	for ( i=0; i<3; i++ ) {
		temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
	}

	// Align for 14 bits
	acc_X = temp[0]/4;
	acc_Y = temp[1]/4;
	acc_Z = temp[2]/4;
}


void read_xyz(void)
{
	// sign extend byte to 16 bits - need to cast to signed since function
	// returns uint8_t which is unsigned
	acc_X = (int8_t) i2c_read_byte(MMA_ADDR, REG_XHI);
	Delay(100);
	acc_Y = (int8_t) i2c_read_byte(MMA_ADDR, REG_YHI);
	Delay(100);
	acc_Z = (int8_t) i2c_read_byte(MMA_ADDR, REG_ZHI);

}

void Delay (uint32_t dly) {
  volatile uint32_t t;

	for (t=dly*10000; t>0; t--)
		;
}

