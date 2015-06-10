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
int Phase;
int Electrical_angle;
int readHallStateFlag;

//How to interpret int Phase:
// 0b|xx|yy|zz
// xx = phase A | yy = phase B | zz = phase C
// 10 = high | 00 = off | 01 = low | 11 = illegal

void updateHallState() {
	readHallStateFlag = 0;
	   if(HallsensorA == 0)
	   {
		   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 0; //000
				   Phase = 0;//illegal state!
				   }
			   else {//HallsensorC == 1
				   Electrical_angle = 240; //001
				   //Phase = 0b000110;
				   Phase = 0b011000;
		   			}
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 120; //010
				   //Phase = 0b011000;
				   Phase = 0b100001;
				   }
			   else{
				   Electrical_angle = 180; //011
				   //Phase = 0b010010;
				   Phase = 0b001001;
				   }
		   }
	   }
	   else //HallsensorA == 1
	   {
	   	   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 0; //100
				   //Phase = 0b100001;
				   Phase = 0b000110;
				   }
			   else {//HallsensorC == 1
				   Electrical_angle = 300; //101
				   //Phase = 0b100100;
				   Phase = 0b010010;
				   }
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 60; //110
				   //Phase = 0b001001;
				   Phase = 0b100100;
				   }
			   else {
				   Electrical_angle = 0; //111
				   Phase = 0;//illegal state!
				   }
		   }

	   }
	//update commutation
	//NOTE: this is not representative of the final product.
	//NOTE: but, for open-loop control, instead of controlling 6 PWM waves,
	//NOTE: I've opted to generate 6 independent waveforms and simply enable/disable
	//NOTE: output for the relevant pins for instantaneous control.
	
	//NANDing Phase with GPAMUX1 will set all bits which are 1 in Phase to 0 in GPAMUX1
	//therefore, we need to invert Phase, cast it to an inverted Uint32 to match the size of GPAMUX1,
	// and finally NAND them together. This is all done in one line.
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all &= (0xFFFF0000 | ((Uint32)(~Phase))); //disable first, for safety
	// next, we need to OR GPAMUX1 with Phase, to set all the bits which actually are 1, to 1.
	//TODO: examine whether or not we need more dead time
	GpioCtrlRegs.GPAMUX1.all |= (Uint32)Phase; //then enable
	EDIS;
}

void InitECapRegs()
{
   HallsensorA = 0;
   HallsensorB = 0;
   HallsensorC = 0;
   Phase = 0;
   Electrical_angle = 0;
///////////////////////////////////////////////
/////// GPIO for commutation
///////////////////////////////////////////////
   EALLOW;
   //Setup GPIO(0-5) Outputs:
   GpioCtrlRegs.GPAPUD.all &= 0xFFFFFFC0;
   GpioDataRegs.GPACLEAR.all |= 0x0000003F;
   GpioCtrlRegs.GPADIR.all |= 0x0000003F;
   EDIS;   
   
   
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

   //capture initial state
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;	         //  GPIO24 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;            //  GPIO24 as input
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;	         //  GPIO25 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;	         //  GPIO26 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;
   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO24;  // read the signal from GPIO24
   HallsensorB = GpioDataRegs.GPADAT.bit.GPIO25;  // read the signal from GPIO25
   HallsensorC = GpioDataRegs.GPADAT.bit.GPIO26;  // read th signal from GPIO26
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;
   EDIS;
}


__interrupt void ecap1_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;	         //  GPIO24 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;            //  GPIO24 as input

   HallsensorA = GpioDataRegs.GPADAT.bit.GPIO24;  // read th signal from GPIO24
   //or with: 0b|0000|0000|0001|1111 = 0x001F clears all relevant interrupt conditions
   ECap1Regs.ECCLR.all |= 0x001F;
   /*
   ECap1Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap1Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap1Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap1Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap1Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global
   */
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1; // Back to capture mode   GPIO24 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap2_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;	         //  GPIO25 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;          //   GPIO25 as input
   HallsensorB = GpioDataRegs.GPADAT.bit.GPIO25;  // read the signal from GPIO25
   //or with: 0b|0000|0000|0001|1111 = 0x001F clears all relevant interrupt conditions
   ECap2Regs.ECCLR.all |= 0x001F;
   //ECap2Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   //ECap2Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   //ECap2Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   //ECap2Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   //ECap2Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global

   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1; // Back to capture mode   GPIO25 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap3_isr(void)
{
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;	         //  GPIO26 as I/O
   GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;           //  GPIO26 as input
   HallsensorC = GpioDataRegs.GPADAT.bit.GPIO26;  // read the signal from GPIO26
   //or with: 0b|0000|0000|0001|1111 = 0x001F clears all relevant interrupt conditions
   ECap3Regs.ECCLR.all |= 0x001F;
   /*
   ECap3Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
   ECap3Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
   ECap3Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
   ECap3Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
   ECap3Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global
   */
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1; // Back to capture mode   GPIO26 as ecap
   EDIS;
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
