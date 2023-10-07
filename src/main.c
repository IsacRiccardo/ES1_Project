#include <MKL25Z4.h>

#include "PeriodicTimer.h"
#include "LED.h"
#include "UARTCom.h"
#include "i2c.h"
#include "mma8451.h"
#include "Recorder.h"


void tx_call(void);

void tx_call(void) {
}

int main(void) {
	int i = 0, j = 0;
	int working = 0;
	
	config_sys_clock();
	
	Com_Init(9600);
	i2c_init();
	init_mma();
	
	init_RGB();
	toggle();
	
	RecorderInit(0, 0, 1);
	
    timer0_init(240000);
	
	while(1) {
		
		if(timerEvent) {
			timerEvent = 0;
			
			if(j > 100) {
				if(i & 0x01)
					set_b(1);
				else
					set_b(0);
				
				if(i & 0x02)
					set_g(1);
				else
					set_g(0);
				
				if(i & 0x04) 
					set_r(1);
				else
					set_r(0);	
				i++;
				j = 0;
			}
			j++;
			RecorderCallback();
		}
		
		if(Com_HasByte()) {
			char c = (char)Com_GetByte();
			if(c == 'r' || c=='R') {
				RecorderStart();
				working = 1;
			}
		}
		
		if(working && RecorderFinished()) {
			Com_WriteByteArray(RecorderGetBuffer(), RecorderGetSize() * 2, &tx_call);
			working = 0;
		}		
		
		__WFI();
	}
	
	return 0;
}

