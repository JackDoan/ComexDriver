//###########################################################################
// Description:
//! 
//! eCAP1/2/3 is configured to capture the time between rising
//! and falling edge of the Hall sensor signal A & B & C
//!
//! \b External \b Connections \n
//! - eCAP1 is on GPIO5
//! - eCAP2 is on GPIO
//! - eCAP3 is on GPIO
//!
//! \b Watch \b Variables \n
//! - \b ECap1IntCount , Interrupt counts
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $
// $Release Date: January 19, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
__interrupt void ecap1_isr(void);
void InitECapture(void);

// Global variables used in this example
Uint32  ECap1IntCount;
int HallsensorA;
int HallsensorB;
int HallsensorC;

void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO: 
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example  
   InitECap1Gpio();
//   InitECap2Gpio();
//   InitECap3Gpio();
   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.ECAP1_INT = &ecap1_isr;
 //  PieVectTable.ECAP2_INT = &ecap2_isr;
 // PieVectTable.ECAP3_INT = &ecap3_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example
   InitECapture();

// Step 5. User specific code, enable interrupts:

// Initialize counters:   
   ECap1IntCount = 0;
   HallsensorA = 0;

// Enable CPU INT4 which is connected to ECAP1-4 INT:
   IER |= M_INT4;

// Enable eCAP INTn in the PIE: Group 3 interrupt 1-6
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
      __asm("          NOP");
   }

} 


void InitECapture()
{
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads at capture event time
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   
   ECap1Regs.ECCTL2.bit.CAP_APWM = 0;         // capture mode enable
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;      // continuous mode enable
   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // circular buffer wraps around and starts again at 4 events
   ECap1Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation
   ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap1Regs.ECEINT.bit.CEVT1 = 1;            // Capture Event 1 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap1Regs.ECEINT.bit.CEVT2 = 1;            // Capture Event 2 Interrupt Enable----capture the falling edge of the hall sensor signal----
   ECap1Regs.ECEINT.bit.CEVT3 = 1;            // Capture Event 3 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap1Regs.ECEINT.bit.CEVT4 = 1;            // Capture Event 4 Interrupt Enable----capture the falling edge of the hall sensor signal----
}

__interrupt void ecap1_isr(void)
{

  if(ECap1Regs.ECFLG.bit.CEVT1 || ECap1Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
   {
     HallsensorA = 1;
     ECap1Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
     ECap1Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   }

   else if(ECap1Regs.ECFLG.bit.CEVT2 || ECap1Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
   {
      HallsensorA = 0;
	  ECap1Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
	  ECap1Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   }
   
   ECap1IntCount++;
   ECap1Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global


   // Acknowledge this interrupt to receive more interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

//===========================================================================
// No more.
//===========================================================================
