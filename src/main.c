#include "MKL25Z4.h"

#include "LED.h"

int main(void) {
	init_RGB();

    while(1){
        toggle_r();
    }
    
	return 0;
}

