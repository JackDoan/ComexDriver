//! Note:
//!    - Maximum speed is configured to 6000rpm(BaseRpm)
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
//!   - Connect eQEP1A(GPIO20) to ePWM1A(GPIO0)(simulates eQEP Phase A signal) J4-40 -> J5-45
//!   - Connect eQEP1B(GPIO21) to ePWM1B(GPIO1)(simulates eQEP Phase B signal) J4-41 -> J5-48
//!   - Connect eQEP1I(GPIO23) to GPIO4 (simulates eQEP Index Signal)
//!
//!  Watch Variables
//!  - qep_posspeed.SpeedRpm_fr - Speed meas. in rpm using QEP position counter
//!  - qep_posspeed.SpeedRpm_pr - Speed meas. in rpm using capture unit
//!  - qep_posspeed.theta_mech  - Motor mechanical angle (Q15)
//!  - qep_posspeed.theta_elec  - Motor electrical angle (Q15)
//!
//!  more info: http://www.ti.com/lit/an/spraah1/spraah1.pdf
//!				http://www.ti.com/lit/ug/spru790d/spru790d.pdf
//!  pins http://www.ti.com/lit/ug/sprui11/sprui11.pdf

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "posspeed.h"   // Example specific Include file

void initEpwm();
__interrupt void prdTick(void);

POSSPEED qep_posspeed=POSSPEED_DEFAULTS;
Uint16 Interrupt_Count = 0;

void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
   InitSysCtrl();
// Step 2. Initalize GPIO: 
// InitGpio();  // Skipped for this example  

// For this case only init GPIO for eQEP1 and ePWM1
// This function is found in F2806x_EQep.c
   InitEQep1Gpio();
   InitEPwm1Gpio();
   EALLOW;
   GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;    // GPIO4 as output simulates Index signal
   GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;  // Normally low  
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
   PieVectTable.EPWM1_INT= &prdTick;
   EDIS;

// Step 4. Initialize all the Device Peripherals:
   initEpwm();  // This function exists in Example_EPwmSetup.c
   
// Step 5. User specific code, enable interrupts:
// Enable CPU INT1 which is connected to CPU-Timer 0:
   IER |= M_INT3;

// Enable TINT0 in the PIE: Group 3 interrupt 1
   //PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx1 = 0;
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
   
   qep_posspeed.init(&qep_posspeed);
  	
	for(;;)
	{
		qep_posspeed.calc(&qep_posspeed);
	}

} 

__interrupt void prdTick(void)                  // EPWM1 Interrupts once every 4 QCLK counts (one period)
{
   // Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
   EPwm1Regs.ETCLR.bit.INT=1;
}

