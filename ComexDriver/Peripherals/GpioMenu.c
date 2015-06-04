/*
 * GpioMenu.c
 *
 *  Created on: May 22, 2015
 *	  Author: jad140230
 */

#include "DSP28x_Project.h"





void doGpioMenu(void) {
	int gpioStatus0 = 0;
	int gpioStatus1 = 0;
	int gpioStatus2 = 0;
	int gpioStatus3 = 0;
	int breakOutOfLoop = 0;
	while(breakOutOfLoop == 0) {
	scia_msg(clearScreen);
	printMenu(gpioMenu, 5);
	read = scia_read();
	switch (read) {
		case 0x30:
			breakOutOfLoop = 1;
			break;
		case 0x31:
			GpioDataRegs.GPATOGGLE.all = 0x0001;
			if (gpioStatus0 == 0) {
				gpioString3[10] = 'N';
				gpioString3[11] = ' ';
			}
			else {
				gpioString3[10] = 'F';
				gpioString3[11] = 'F';
			}
			gpioStatus0 = !gpioStatus0;
			break;
		case 0x32:
			GpioDataRegs.GPATOGGLE.all = 0x0002;
			if (gpioStatus1 == 0) {
					gpioString3[24] = 'N';
					gpioString3[25] = ' ';
			}
			else {
					gpioString3[24] = 'F';
					gpioString3[25] = 'F';
			}
			gpioStatus1 = !gpioStatus1;
			break;
		case 0x33:
			GpioDataRegs.GPATOGGLE.all = 0x0004;
			if (gpioStatus2 == 0) {
					gpioString3[38] = 'N';
					gpioString3[39] = ' ';
			}
			else {
					gpioString3[38] = 'F';
					gpioString3[39] = 'F';
			}
			gpioStatus2 = !gpioStatus2;
			break;
		case 0x34:
			GpioDataRegs.GPATOGGLE.all = 0x0008;
			if (gpioStatus3 == 0) {
					gpioString3[52] = 'N';
					gpioString3[53] = ' ';
			}
			else {
					gpioString3[52] = 'F';
					gpioString3[53] = 'F';
			}
			gpioStatus3 = !gpioStatus3;
			break;

		default:
			scia_msg(nope);
			break;
	}
  }
  return;
}

