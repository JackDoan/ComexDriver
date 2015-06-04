//###########################################################################
// Description:
//! 
//! eCAP1/2/3 is configured to capture the time between rising
//! and falling edge of the Hall sensor signal A & B & C
//!
//! \b External \b Connections \n
//! - eCAP1 is on GPIO5
//! - eCAP2 is on GPIO25
//! - eCAP3 is on GPIO26
//!
//! \b Watch \b Variables \n
//! - \b ECap1/2/3IntCount , Interrupt counts 1/2/3
//! - \b Hallsensor state A/B/C
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
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
void InitECapture1(void);
void InitECapture2(void);
void InitECapture3(void);


// Global variables used in this example
Uint32  ECap1IntCount;
Uint32  ECap2IntCount;
Uint32  ECap3IntCount;


int HallsensorA;
int HallsensorB;
int HallsensorC;
int Electrical_angle;

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
   InitECap2Gpio();
   InitECap3Gpio();
   
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

// Interrupts are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.ECAP1_INT = &ecap1_isr;  //             Group 4 PIE Peripheral Vectors
   PieVectTable.ECAP2_INT = &ecap2_isr;
   PieVectTable.ECAP3_INT = &ecap3_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example
   InitECapture1();
   InitECapture2();
   InitECapture3();

// Step 5. User specific code, enable interrupts:

// Initialize counters:   
   ECap1IntCount = 0;
   ECap2IntCount = 0;
   ECap3IntCount = 0;
   HallsensorA = 0;
   HallsensorB = 0;
   HallsensorC = 0;
   Electrical_angle = 0;
// Enable CPU INT4 which is connected to ECAP1-4 INT:
   IER |= M_INT4;


// Enable eCAP INTn in the PIE:
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // INT4.1 for ecap1
   PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      // INT4.2 for ecap2
   PieCtrlRegs.PIEIER4.bit.INTx3 = 1;      // INT4.3 for ecap3

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
	   if(HallsensorA == 1 && HallsensorB == 0 && HallsensorC == 0)
	       {
		   Electrical_angle = 0;
	       }
	   if(HallsensorA == 1 && HallsensorB == 1 && HallsensorC == 0)
	  	   {
	  	   Electrical_angle = 60;
	  	   }
	   if(HallsensorA == 0 && HallsensorB == 1 && HallsensorC == 0)
	  	   {
	  	   Electrical_angle = 120;
	  	   }
	   if(HallsensorA == 0 && HallsensorB == 1 && HallsensorC == 1)
	  	   {
	  	   Electrical_angle = 180;
	  	   }
	   if(HallsensorA == 0 && HallsensorB == 0 && HallsensorC == 1)
	  	   {
	  	   Electrical_angle = 240;
	  	   }
	   if(HallsensorA == 1 && HallsensorB == 0 && HallsensorC == 1)
	  	   {
	  	   Electrical_angle = 300 ;
	  	   }
	   else
		   Electrical_angle = 0;
 //     __asm("          NOP");
   }

} 


void InitECapture1()
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


void InitECapture2()
{
   ECap2Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap2Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap2Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads at capture event time
   ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

   // Configure peripheral registers

   ECap2Regs.ECCTL2.bit.CAP_APWM = 0;         // capture mode enable
   ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0;      // continuous mode enable
   ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;        // circular buffer wraps around and starts again at 4 events
   ECap2Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap2Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap2Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap2Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap2Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation
   ECap2Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation
   ECap2Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation
   ECap2Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation
   ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
   ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap2Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

   ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap2Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap2Regs.ECEINT.bit.CEVT1 = 1;            // Capture Event 1 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap2Regs.ECEINT.bit.CEVT2 = 1;            // Capture Event 2 Interrupt Enable----capture the falling edge of the hall sensor signal----
   ECap2Regs.ECEINT.bit.CEVT3 = 1;            // Capture Event 3 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap2Regs.ECEINT.bit.CEVT4 = 1;            // Capture Event 4 Interrupt Enable----capture the falling edge of the hall sensor signal----
}


void InitECapture3()
{
   ECap3Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap3Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap3Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads at capture event time
   ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

   // Configure peripheral registers

   ECap3Regs.ECCTL2.bit.CAP_APWM = 0;         // capture mode enable
   ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;      // continuous mode enable
   ECap3Regs.ECCTL2.bit.STOP_WRAP = 3;        // circular buffer wraps around and starts again at 4 events
   ECap3Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap3Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap3Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap3Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap3Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation
   ECap3Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation
   ECap3Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation
   ECap3Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation
   ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
   ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap3Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

   ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap3Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap3Regs.ECEINT.bit.CEVT1 = 1;            // Capture Event 1 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap3Regs.ECEINT.bit.CEVT2 = 1;            // Capture Event 2 Interrupt Enable----capture the falling edge of the hall sensor signal----
   ECap3Regs.ECEINT.bit.CEVT3 = 1;            // Capture Event 3 Interrupt Enable----capture the rising edge of the hall sensor signal----
   ECap3Regs.ECEINT.bit.CEVT4 = 1;            // Capture Event 4 Interrupt Enable----capture the falling edge of the hall sensor signal----
}


__interrupt void ecap1_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;	         //  GPIO5 as I/O
   EDIS;
   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO5;  // read th signal from GPIO5

   ECap1IntCount++;
   ECap1Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap1Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap1Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap1Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap1Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global

   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3; // Back to capture mode   GPIO5 as ecap
   // Acknowledge this interrupt to receive more interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap2_isr(void)
{

  if(ECap2Regs.ECFLG.bit.CEVT1 || ECap2Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
   {
     HallsensorB = 1;
     ECap2Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
     ECap2Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   }

  if(ECap2Regs.ECFLG.bit.CEVT2 || ECap2Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
   {
      HallsensorB = 0;
	  ECap2Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
	  ECap2Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   }

   ECap2IntCount++;
   ECap2Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global


   // Acknowledge this interrupt to receive more interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap3_isr(void)
{

  if(ECap3Regs.ECFLG.bit.CEVT1 || ECap3Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
   {
     HallsensorC = 1;
     ECap3Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
     ECap3Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   }

   if(ECap3Regs.ECFLG.bit.CEVT2 || ECap3Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
   {
      HallsensorC = 0;
	  ECap3Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
	  ECap3Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   }

   ECap3IntCount++;
   ECap3Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global


   // Acknowledge this interrupt to receive more interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

//===========================================================================
// No more.
//===========================================================================
