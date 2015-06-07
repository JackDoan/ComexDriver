/*
 * bootstrap.c
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */

#include "DSP28x_Project.h"

void bootstrap() {

	  EALLOW;
	  IER = 0x0000; // Disable CPU interrupts
	  IFR = 0x0000; // Clear all CPU interrupt flags
	  DINT;
	  InitPieCtrl();
	  //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	  InitPieVectTable();
	  //PieVectTable.TINT0 = &cpu_timer0_isr;
	  IER |= M_INT1;
	  //InitCpuTimers();
	  //ConfigCpuTimer(&CpuTimer0, 60,500000);
	  //CpuTimer0Regs.TCR.bit.TSS = 0;              //CpuTimer0 Start/ReStart
	  EnableInterrupts();

	  EDIS;
	  scia_init();
}

