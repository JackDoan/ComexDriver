#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DSP28x_Project.h"
#include "libSCI.c"
#include "MenuStrings.c"
#include "bootstrap.c"
#include "AdcMenu.c"
#include "PwmMenu.c"
#include "GpioMenu.c"

void main()
{

  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
  InitSysCtrl();
  InitFlash();
  bootstrap();
  scia_init();
  while(1) {
	scia_msg(clearScreen);
    printMenu(mainMenu, 5);
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





