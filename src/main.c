#include <MKL25Z4.h>

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "PeriodicTimer.h"
#include "LED.h"
#include "UARTCom.h"
#include "i2c.h"
#include "mma8451.h"
#include "Recorder.h"


static char buff[150];

static int offsetX, offsetY, offsetZ;
static int minX, minY, minZ;
static int maxX, maxY, maxZ;
static float amplification;
static int calibration_counter;
int direction = 0;	// variable used to determine the direction of pitch and roll (1 -> pos, -1 -> neg)

void tx_call(void);
void print_float(char* str, float tmpVal, int new_line); 
void periodic_task(void);
void __calibrate(void);
void calibrate_task(void);

int main(void) {
	int calibrate = 1;

	int periodic = 0;
	int periodic_task_cnt = 0;
	int roll_counter = 0;
	int pitch_counter = 0;
	roll = 0;
	
	config_sys_clock();
	
	Com_Init(9600);
	i2c_init();
	init_mma();
	
	init_RGB();
	toggle();
	
	amplification = 1/4096.0f;
	offsetX = 40000;
	
	//RecorderInit(1, 1, 1);
	
  timer0_init(240000); // 0.01s
	
	while(1) {
		// calibrate if enabled
		if(calibrate)
		{
			calibrate_task();
			if(calibration_counter >= 256) {
				calibrate = 0;
				periodic = 1;
				// toggle green led to let know cal is done
				
			}	
		}
		
		if(timerEvent) {
			
			timerEvent = 0;
			periodic_task_cnt++;
			
			// Executed once every 100 ms, once calibration is done
			if(periodic && periodic_task_cnt > 10) {
				periodic_task();	
				periodic_task_cnt	= 0;			
			}
			
			// Set by periodic task
			// If roll detected
			if(roll){
				roll_counter++;
				switch(roll){ //switch to determine led freq
					case 1:
						if(roll_counter >= 10) //100 ms
							{
								if(direction==1)
									toggle_g();
								else
									toggle_gb();
								
								roll_counter = 0;
							}
						break;
					case 2:
						if(roll_counter >= 8) //80 ms
							{
								if(direction==1)
									toggle_g();
								else
									toggle_gb();
								
								roll_counter = 0;
							}
						break;
					case 3:
						if(roll_counter >= 6) //60 ms
							{
								if(direction==1)
									toggle_g();
								else
									toggle_gb();
								
								roll_counter = 0;
							}
						break;
				}
			}
			else{
				roll_counter = 0;
				direction = 0;
				set_g(0);
				set_b(0);
			}
		
		}
		
		if(Com_HasByte()) {
			char c = (char)Com_GetByte();
			}
		
		__WFI();
	}
	
	return 0;
}

void periodic_task(void) {
	float val;
	read_full_xyz();
	
	__calibrate();
		
	sprintf(buff, "x=%d y=%d z=%d\n\r", acc_X, acc_Y, acc_Z);
	sprintf(buff, "roll=%d\n", roll); 
	sprintf(buff, "direction=%d\n", direction);
	stdout_putstr(buff, 50);
	val = acc_X * amplification;
	print_float("X [G]", val, 0);
	val = acc_Y * amplification;
	print_float("Y [G]", val, 0);
	val = acc_Z * amplification;
	print_float("Z [G]", val, 1);
	
	// roll detection
	if(acc_X)
	{
		// Equal intervals of 0.3 to determine which stage of roll is present
		// 0.1 -> 410
		// 0.4 -> 1639 + 410
		// 0.7 -> 2867 + 410
		// 1 -> 4096
		if(acc_X >= 410 && acc_X <= 2049) 
		{	
			roll = 1; direction = 1;
		}
		if(acc_X <= -410 && acc_X >= -2049)
		{
			roll = 1; direction = 2;
		}
		if(acc_X >= 2049 && acc_X <= 3277)
		{
			roll = 2; direction = 1;
		}
		if(acc_X <= -2049 && acc_X >= -3277)
		{
			roll = 2; direction = 2;
		}
		if(acc_X >= 3277 && acc_X <= 4096) 
		{
			roll = 3; direction = 1;
		}	
		if(acc_X <= -3277 && acc_X >= -4096)
		{
			roll = 3; direction = 2;
		}
	}
	else
		roll = 0;
}

void __calibrate(void) {
	if(offsetX < 30000) {
		acc_X -= offsetX;
		acc_Y -= offsetY;
		//acc_Z -= offsetZ;
		
		if(acc_X < maxX && acc_X > minX) {
			acc_X = 0;
		}
		if(acc_Y < maxY && acc_Y > minY) {
			acc_Y = 0;
		}
		if(acc_Z < maxZ && acc_Z > minZ) {
			acc_Z = (short)offsetZ;
		}
	}
}

void calibrate_task(void) {
	
	read_full_xyz();
	
	if(calibration_counter < 0) {
		offsetX = 0;
    	offsetY = 0; 
		offsetZ = 0;
		minX = 30000;
		minY = 30000;
		minZ = 30000;
		maxX = -30000; 
		maxY = -30000;
		maxZ = -30000;
		amplification = 1.0;
	} else if (calibration_counter < 128) {
		offsetX += acc_X;
		offsetY += acc_Y;
		offsetZ += acc_Z;
	} else if (calibration_counter == 128) {
		offsetX >>= 7;
		offsetY >>= 7;
		offsetZ >>= 7;
		amplification = 1.0f/(float)offsetZ;
	} else {
		acc_X -= offsetX;
		if(acc_X > maxX)
			maxX = acc_X;
		if(acc_X < minX)
			minX = acc_X;
		
		acc_Y -= offsetY;
		if(acc_Y > maxY)
			maxY = acc_Y;
		if(acc_Y < minY)
			minY = acc_Y;
		
		//acc_Z -= offsetZ;
		if(acc_Z > maxZ)
			maxZ = acc_Z;
		if(acc_Z < minZ)
			minZ = acc_Z;		
	}
	
	calibration_counter++;
}

void print_float(char* str, float tmpVal, int new_line) {
    char *tmpSign = (tmpVal < 0) ? "-" : "";
    tmpVal = (tmpVal < 0) ? tmpVal * -1.0f : tmpVal;
    int tmpInt1 = (int)tmpVal;                  // Get the integer (678).
    float tmpFrac = tmpVal - (float)tmpInt1;      // Get fraction (0.0123).
    tmpVal = truncf(tmpFrac * 10000);  // Turn into integer (123).
	int tmpInt2 = (int) tmpVal;

    // Print as parts, note that you need 0-padding for fractional bit.
	if(new_line) {
		sprintf ((char*)buff, "%s = %s%d.%04d\n\r", str, tmpSign, tmpInt1, tmpInt2);
	} else {
		sprintf ((char*)buff, "%s = %s%d.%04d ", str, tmpSign, tmpInt1, tmpInt2);
	}
    stdout_putstr(buff, (char)strlen((char*)buff));
}

void tx_call(void) {
}
