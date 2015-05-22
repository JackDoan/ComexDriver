/*
 * AdcMenu.c
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */

#include "DSP28x_Project.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void doAdcMenu(void) {
	Uint16 valueADC = 0;
	char adcPrint[20] = {0};
	EALLOW;
	InitAdc();
	//ConfigAdc();
	InitAdcAio();
	AdcOffsetSelfCal();
	AdcChanSelect(4); //J1(6)
	int ACQPS_Value = 6;

	AdcRegs.ADCSOC0CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC1CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC2CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC3CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC4CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC5CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC6CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC7CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC8CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC9CTL.bit.ACQPS= ACQPS_Value;
	AdcRegs.ADCSOC10CTL.bit.ACQPS = ACQPS_Value;
	AdcRegs.ADCSOC11CTL.bit.ACQPS = ACQPS_Value;
	AdcRegs.ADCSOC12CTL.bit.ACQPS = ACQPS_Value;
	AdcRegs.ADCSOC13CTL.bit.ACQPS = ACQPS_Value;
	AdcRegs.ADCSOC14CTL.bit.ACQPS = ACQPS_Value;
	AdcRegs.ADCSOC15CTL.bit.ACQPS = ACQPS_Value;

	EDIS;
	scia_msg(clearScreen);
	printMenu(adcMenu, 4);
	read = scia_read();

	while (SciaRegs.SCIFFRX.bit.RXFFST == 0) {
		//do stuff
		scia_PrintLF();
		scia_msg(clearScreen);
		//scia_msg(menuDivider);
		//scia_PrintLF();

		// Configure the ADC to sample the temperature sensor
		EALLOW;
		AdcRegs.ADCCTL1.bit.TEMPCONV = 0; //Connect A5 - temp sensor
		AdcRegs.ADCSOC0CTL.bit.CHSEL = 14; //Set SOC0 to sample A5
		AdcRegs.ADCSOC1CTL.bit.CHSEL = 14; //Set SOC1 to sample A5
		AdcRegs.ADCSOC0CTL.bit.ACQPS = 6; //Set SOC0 ACQPS to 7 ADCCLK
		AdcRegs.ADCSOC1CTL.bit.ACQPS = 6; //Set SOC1 ACQPS to 7 ADCCLK
		AdcRegs.INTSEL1N2.bit.INT1SEL = 1; //Connect ADCINT1 to EOC1
		AdcRegs.INTSEL1N2.bit.INT1E = 1; //Enable ADCINT1
		EDIS;
		// Sample the temperature sensor
		AdcRegs.ADCSOCFRC1.all = 0xFF; //Sample temp sensor
		while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){} //Wait for ADCINT1
		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Clear ADCINT1
		valueADC = AdcResult.ADCRESULT1; //Get temp sensor sample result
		snprintf(adcPrint, 20, "%d\r\n", valueADC);
		scia_msg(adcPrint);
		/*DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);
		DELAY_US(32000);*/
	}
	return;
}


