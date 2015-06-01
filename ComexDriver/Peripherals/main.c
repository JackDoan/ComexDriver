#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DSP28x_Project.h"
#include "MenuStrings.h"
#include "bootstrap.h"
#include "AdcMenu.h"
#include "PwmMenu.h"
#include "GpioMenu.h"

void main()
{
  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
  InitSysCtrl();
  InitFlash();
  bootstrap();
  while(1) {
	scia_msg(clearScreen);
    printMenu(mainMenu, 4);
	read = scia_read();
	switch (read) {
		case 0x31:
			doPwmMenu();
			break;
		case 0x32:
			doAdcMenu();
			break;
		case 0x33:
			doGpioMenu();
			break;
		default:
			scia_msg(nope);
			read = scia_read();
			break;
	}
  }

}
