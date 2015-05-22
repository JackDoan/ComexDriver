/*
 * PwmMenu.c
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */

#include "DSP28x_Project.h"

void epwmInit(int freq, int duty) {
	//InitGpio();
  	InitEPwmGpio();
	// EPWM Module 1 config
	EPwm1Regs.TBPRD = freq; // Period = 1500 TBCLK counts
	EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm1Regs.DBFED = 20; // FED = 20 TBCLKs
	EPwm1Regs.DBRED = 20; // RED = 20 TBCLKs

	// EPWM Module 2 config
	EPwm2Regs.TBPRD = freq; // Period = 1500 TBCLK counts
	EPwm2Regs.TBPHS.half.TBPHS = 300; // Phase = 300/1500 * 360 = 120 deg
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN; // Count DOWN on sync (=120 deg)
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi Complementary
	EPwm2Regs.DBFED = 20; // FED = 20 TBCLKs
	EPwm2Regs.DBRED = 20; // RED = 20 TBCLKs

	EPwm1Regs.CMPA.half.CMPA = (freq / duty); // adjust duty for output EPWM1A
	EPwm2Regs.CMPA.half.CMPA = (freq / duty); // adjust duty for output EPWM2A
	return;
}


void doPwmMenu(void) {
	int dutyCycleMult = 2;
	int epwmFreq = 1500;
	int pwmStatus = 0;
	int breakOutOfLoop = 0;
        while(breakOutOfLoop == 0) {
                scia_msg(clearScreen);
                printMenu(pwmMenu, 6);
                read = scia_read();
                switch (read) {
                        case 0x30: //go back
                                breakOutOfLoop = 1;
                                break;
			case 0x31: //enable/disable pwm
				if (pwmStatus == 0) {
					epwmInit(epwmFreq, dutyCycleMult);
					pwmStatus = 1;
					pwmString2[25] = ' '; //[1) Toggle PWM:           OFF]
					pwmString2[26] = 'O';
					pwmString2[27] = 'N';
				}
				else {
					pwmStatus = 0;
					EALLOW;
					GpioCtrlRegs.GPADIR.all = 0xFFFF;
					GpioCtrlRegs.GPAMUX1.all = 0x0000;
  					GpioDataRegs.GPASET.all = 0xFFFF;
					EDIS;
					pwmString2[25] = 'O';
					pwmString2[26] = 'F';
					pwmString2[27] = 'F';
				}
				break;
			case 0x32:
				if (pwmStatus == 1) {
					char dutyCyclePrompt[] = "Enter the new duty cycle:";
					int i = 0;
					int mult = 10;
					int newDutyCycleMult = 0;
					char digits[] = "00";
					scia_msg(dutyCyclePrompt);
					while (i < 2) {
						read = scia_read();
						if (read < 0x3A || read > 0x2F) {
							newDutyCycleMult = newDutyCycleMult + (mult*(read-48));
							digits[i] = read;
							i++;
							mult = mult - 9;
						}
						else {
							scia_msg(nope);
							i = 0;
							break;
						}
					}
					if (i != 0) {
						dutyCycleMult = newDutyCycleMult;
						//"2) Duty Cycle:           50% ";
						pwmString3[25] = digits[0];
						pwmString3[26] = digits[1];
						i = ((epwmFreq/100)*dutyCycleMult);
						EPwm1Regs.CMPA.half.CMPA = (epwmFreq-i); // adjust duty for output EPWM1A
       						EPwm2Regs.CMPA.half.CMPA = (epwmFreq-i);
					}
				}
				else {
					scia_msg(pwm_is_kill);
					read = scia_read();
				}
				break;
			case 0x33:
				if (pwmStatus == 1) {

				}
				else {
					scia_msg(pwm_is_kill);
					read = scia_read();
				}
				break;
        }
	}
}



