//! This ADC example uses ePWM1 to generate a periodic ADC SOC - ADCINT1.
//! Two channels are converted, ADCINA4 and ADCINA2.
//! 
//! \b Watch \b Variables \n
//! - Voltage1[10]    - Last 10 ADCRESULT0 values
//! - Voltage2[10]    - Last 10 ADCRESULT1 values
//! - result1[10]     - Last 10 actual voltage values
//! - result2[10]     - Last 10 actual voltage values
//! - ConversionCount - Current result number 0-9

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

__interrupt void adc_isr(void);
void ConfigureADC(void);
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];


void ConfigureADC() {

// Configure ADC
	EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;	// Enable non-overlap mode
	AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	// ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E     = 1;	// Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	// Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL 	= 1;    // setup EOC1 to trigger ADCINT1 to fire
    AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 4;    // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 2;    // set SOC1 channel select to ADCINA2
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	EDIS;

	ConversionCount = 0;
    EPwm4Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
    EPwm4Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
    EPwm4Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
    EPwm4Regs.CMPA.half.CMPA 	= 0x0080;	// Set compare A value
    EPwm4Regs.TBPRD 				= 0xFFFF;	// Set period for ePWM4
    EPwm4Regs.TBCTL.bit.CTRMODE 	= 0;		// count up and start

}

__interrupt void  adc_isr(void)
{

  Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
  Voltage2[ConversionCount] = AdcResult.ADCRESULT1;

 // result1[ConversionCount] = (Voltage1[ConversionCount]/4096);
 // result2[ConversionCount] = (Voltage2[ConversionCount]/4096);

  // If 20 conversions have been logged, start over
  if(ConversionCount == 9)
  {
     ConversionCount = 0;
  }
  else ConversionCount++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}


