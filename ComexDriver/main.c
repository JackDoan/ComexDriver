//! Note:
//!    - Maximum speed is configured to 3200rpm(BaseRpm)
//!    - Minimum speed is assumed at 10rpm for capture pre-scalar selection
//!    - Pole pair is configured to 12 (pole_pairs)
//!    - QEP Encoder resolution is configured to 25600counts/revolution (mech_scaler)
//!    - which means: 25600/4 = 6400 line/revolution quadrature encoder (simulated by EPWM1)
//!    - EPWM1 (simulating QEP encoder signals) is configured for 5kHz frequency or 300 rpm 
//!      (=4*5000 cnts/sec * 60 sec/min)/25600 cnts/rev)
//!    - SPEEDRPM_FR: High Speed Measurement is obtained by counting the QEP input pulses
//!                   for 10ms (unit timer set to 100Hz). 
//!    - SPEEDRPM_FR = (Position Delta/10ms) * 60 rpm 
//!    - SPEEDRPM_PR: Low Speed Measurement is obtained by measuring time period of QEP edges.
//!                   Time measurement is averaged over 64edges for better results and 
//!                   capture unit performs the time measurement using pre-scaled SYSCLK
//!    - pre-scaler for capture unit clock is selected such that capture timer does not 
//!      overflow at the required minimum RPM speed.
//!
//!  External Connections
//!   - Connect eQEP1A(GPIO20) J5-45
//!   - Connect eQEP1B(GPIO21) J5-48
//!
//!  Watch Variables
//!  - qep_posspeed.SpeedRpm_fr - Speed meas. in rpm using QEP position counter
//!  - qep_posspeed.SpeedRpm_pr - Speed meas. in rpm using capture unit
//!  - qep_posspeed.theta_mech  - Motor mechanical angle (Q15)
//!  - qep_posspeed.theta_elec  - Motor electrical angle (Q15)
//!
//!  more info: http://www.ti.com/lit/an/spraah1/spraah1.pdf
//!				http://www.ti.com/lit/ug/spru790d/spru790d.pdf
//!  ePWM: http://www.ti.com/lit/ug/spru791f/spru791f.pdf
//!  pins http://www.ti.com/lit/ug/sprui11/sprui11.pdf
//! ECAP1 = J6-55
//! ECAP2 = J6-54
//! ECAP3 = J6-58

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "posspeed.h"   // Example specific Include file
#include "ecap.h"
#include "libSCI.h"
#include "libSPI.h"
#include "ePWM.h"
#include "adc.h"



POSSPEED qep_data=POSSPEED_DEFAULTS;
volatile struct drv8301_t drv8301;

void main(void) {
   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
   InitSysCtrl();
   SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;    // eQEP2
   SysCtrlRegs.PCLKCR0.bit.SPIBENCLK = 0;     // SPI-B
   InitFlash();
// Step 2. Initalize GPIO: 
// InitGpio();  // Skipped; not needed
   InitEQep1Gpio();
   //InitEPwm1Gpio();
   //InitEPwm2Gpio();
   //InitEPwm3Gpio();
   InitECap1Gpio();
   InitECap2Gpio();
   InitECap3Gpio();
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;
   GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;
   setupDrv8301();
   setupSpiA();
   //DRV8301_setupSpi();
   EDIS;
   DINT;
   InitPieCtrl(); // The default state is all PIE interrupts disabled and flags are cleared.
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell ISRs.
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to our ISR functions
   EALLOW;
   PieVectTable.ECAP1_INT = &ecap1_isr;  // Group 4 PIE Peripheral Vectors
   PieVectTable.ECAP2_INT = &ecap2_isr;	 // ''
   PieVectTable.ECAP3_INT = &ecap3_isr;  // ''
   PieVectTable.ADCINT1 = &adc_isr; //
   //PieVectTable.SCIRXINTA = &scia_isr;
   EDIS;

// Step 4. Initialize all the Device Peripherals:
   InitECapRegs();
   scia_init();
   epwmInit(1,2,0); //10kHz, 50% duty, no chop
   InitAdc();
   AdcOffsetSelfCal();
   
// Step 5. Enable interrupts:
   IER |= M_INT1; // Enable CPU Interrupt 1 (connected to ADC)
   IER |= M_INT4; // Enable CPU INT4 which is connected to ECAP1-4 INT
   IER |= M_INT3; // Enable CPU INT1 which is connected to CPU-Timer 0:

   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	   // INT1.1 for ADC
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // INT4.1 for ecap1
   PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // INT4.2 for ecap2
   PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // INT4.3 for ecap3



// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
   
   qep_data.init(&qep_data);
   int printData = 10001;
   readHallStateFlag = 1;
   char writeBuffer[80] = {0};
   int lastPhase = 0;
   gogo = 1;
   DRV8301_enable();
   DRV8301_setupSpi();
   i = 0;
	while(1) {
		qep_data.calc(&qep_data);
		if (readHallStateFlag)
			updateHallState();
		if ((lastPhase != Phase) && (printData)) {
			//\033[2J\033[0;0H\r
			sprintf(writeBuffer, "Hall State: %d\n\r", (int)Phase);
			scia_msg(writeBuffer);
		    sprintf(writeBuffer, "Velocity: %d rpm\n\r", qep_data.SpeedRpm_fr);
			scia_msg(writeBuffer);
			//sprintf(writeBuffer, "Mechanical Angle: %f degrees\n\r", qep_data.theta_mech*360);
			//scia_msg(writeBuffer);
			//sprintf(writeBuffer, "Electrical Angle: %f\n\r", qep_data.theta_elec);
			//scia_msg(writeBuffer);
			lastPhase = Phase;
		}
		//DRV8301_readData();
		//if (!Phase /*|| drv8301.fault || drv8301.OverTempShutdown || drv8301.OverTempWarning*/) {
			//while (!Phase || drv8301.fault || drv8301.OverTempShutdown || drv8301.OverTempWarning){
				//GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
				///DELAY_US(32000);
				//DELAY_US(32000);
				//sprintf(writeBuffer, "\aERROR DECTECTED: \n\r Hall State: %d %d %d\n\r", CoilA, CoilB, CoilC);
				//scia_msg(writeBuffer);
				//sprintf(writeBuffer, "Fault Bit: %d\n\rOverTempShutdown: %d\n\rOverTempWarning%d\n\r", drv8301.fault, drv8301.OverTempShutdown, drv8301.OverTempWarning);
				//scia_msg(writeBuffer);
			//}
		//}
		//else {
		    //GpioDataRegs.GPBSET.bit.GPIO50 = 1;
		//}

	}

} 


