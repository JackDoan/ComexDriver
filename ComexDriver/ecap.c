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

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
void InitECapRegs(void);

int HallsensorA;
int HallsensorB;
int HallsensorC;
int Electrical_angle;
int readHallStateFlag;
// Initialize counters:


void updateHallState() {
	readHallStateFlag = 0;
	   if(HallsensorA == 0)
	   {
		   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0)
				   Electrical_angle = 0; //000
			   else //HallsensorC == 1
				   Electrical_angle = 240; //001
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0)
				   Electrical_angle = 120; //010
			   else
				   Electrical_angle = 180; //011
		   }
	   }
	   else //HallsensorA == 1
	   {
	   	   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0)
				   Electrical_angle = 0; //100
			   else //HallsensorC == 1
				   Electrical_angle = 300; //101
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0)
				   Electrical_angle = 60; //110
			   else
				   Electrical_angle = 0; //111
		   }

	   }
}

void InitECapRegs()
{
   HallsensorA = 0;
   HallsensorB = 0;
   HallsensorC = 0;
   Electrical_angle = 0;
///////////////////////////////////////////////
/////// ECAP 1
///////////////////////////////////////////////
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads at capture event time
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

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
/////////////////////////////////////////////////
/////////  ECAP 2
/////////////////////////////////////////////////
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
////////////////////////////////////////////////////////
///////  ECAP3
////////////////////////////////////////////////////////
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
   GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;
   EDIS;
   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO5;  // read th signal from GPIO5

   ECap1Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap1Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap1Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap1Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap1Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global

   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3; // Back to capture mode   GPIO5 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap2_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;	         //  GPIO25 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
   EDIS;
   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO25;  // read th signal from GPIO25

   ECap2Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap2Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap2Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap2Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap2Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global

   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1; // Back to capture mode   GPIO25 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap3_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;	         //  GPIO5 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;
   EDIS;
   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO26;  // read th signal from GPIO5

   ECap3Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap3Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap3Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap3Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap3Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global

   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1; // Back to capture mode   GPIO26 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
