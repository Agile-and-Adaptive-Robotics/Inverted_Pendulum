/*
 * main.c
 *
 * Created: 8/29/2022 8:59:26 AM
 *  Author: Sunil
 */ 

#include <xc.h>

int main(void) {
	DDRB = 1 << PINB0;

	uint8_t levels = 127;

	while(1) {
		static uint8_t currentLevel = 0;
		currentLevel = SPDR;

		int onTotal = currentLevel;
		int offTotal = levels - currentLevel;

		static int counter = 0;
		counter++;

		static int on = 0;

		if (on) {
			if (counter == onTotal) {
				counter = 0;
				on = 0;
			}
		} else {
			if (counter == offTotal) {
				counter = 0;
				on = 1;
			}

		}
	} // while
		

} // main