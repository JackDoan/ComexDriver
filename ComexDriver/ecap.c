//! eCAP1/2/3 is configured to capture the time between rising
//! and falling edge of the Hall sensor signal A & B & C
//!
//! \b External \b Connections \n
//! - eCAP1 is on GPIO24
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

int gogo;
int HallsensorA;
int HallsensorB;
int HallsensorC;
int HallState;
Uint32 Phase;
int Electrical_angle;
int readHallStateFlag;
int direction; //0 = forward, 1 = reverse
//int phaseHistory[36];
int i;

//How to interpret int Phase
// 0x(CoilC)(CoilB)(CoilA)
// 4 = On High
// 1 = On Low
// 0 = Off


void updateHallState() {
	readHallStateFlag = 0;
	   /*if(HallsensorA == 0)
	   {
		   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 0; //000
				   Phase = 0x000; //illegal state!
				   }
			   else {//HallsensorC == 1
				   Electrical_angle = 240; //001
				  Phase =  0x140;
		   		  }
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 120; //010
				   Phase = 0x401;
				   }
			   else{
				   Electrical_angle = 180; //011
				   Phase = 0x041;
				   }
		   }
	   }
	   else //HallsensorA == 1
	   {
	   	   if (HallsensorB == 0)
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 0; //100
				   Phase = 0x014;
				   }
			   else {//HallsensorC == 1
				   Electrical_angle = 300; //101
				   Phase = 0x104;
				   }
		   }
		   else //HallsensorB == 1
		   {
			   if (HallsensorC == 0) {
				   Electrical_angle = 60; //110
				   Phase = 0x410;
				   }
			   else {
				   Electrical_angle = 0; //111
				   Phase = 0x000;
				   }
		   }

	   }*/
	switch (HallState) {
		case 0b000:
			Phase = 0;
			break;
		case 0b001:
			Phase = 0x140;
			break;
		case 0b010:
			Phase = 0x401;
			break;
		case 0b011:
			Phase = 0x041;
			break;
		case 0b100:
			Phase = 0x014;
			break;
		case 0b101:
			Phase = 0x104;
			break;
		case 0b110:
			Phase = 0x410;
			break;
		case 0b111:
			Phase = 0;
			break;
	}
	//update commutation
	//NOTE: this is not representative of the final product.
	//NOTE: but, for open-loop control, instead of controlling 6 PWM waves,
	//NOTE: I've opted to generate 6 independent waveforms and simply enable/disable
	//NOTE: output for the relevant pins for instantaneous control.
	
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all &= 0xFFFFF000;

	//keep a buffer of phase transitions
	/*phaseHistory[i] = Phase;
	i++;
	if (i == 36) {
		i = 0;
	}*/
	//end buffer

	if (gogo){
		GpioCtrlRegs.GPAMUX1.all |= Phase; //then enable
	}
	EDIS;
}



__interrupt void ecap1_isr(void)
{
	if(ECap1Regs.ECFLG.bit.CEVT1 || ECap1Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
	   {
		 HallState |= 0b100;
		 ECap1Regs.ECCLR.all |= 0b01011;
	     //HallsensorA = 1;
	    // ECap1Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
	     //ECap1Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
	     //aGoHigh++;
	   }

	if(ECap1Regs.ECFLG.bit.CEVT2 || ECap1Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
	   {
		  HallState &= 0b011;
	      //HallsensorA = 0;
	      ECap1Regs.ECCLR.all |= 0b10101;
		  //ECap1Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
		  //ECap1Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
		 // aGoLow++;
	   }
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   //ECap1Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap2_isr(void)
{
	if(ECap2Regs.ECFLG.bit.CEVT1 || ECap2Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
	   {
		 HallState |= 0b010;
		 ECap2Regs.ECCLR.all |= 0b01011;
	     //HallsensorB = 1;
	     //ECap2Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
	     //ECap2Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
	     //bGoHigh++;
	   }

	if(ECap2Regs.ECFLG.bit.CEVT2 || ECap2Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
	   {
		  HallState &= 0b101;
	      //HallsensorB = 0;
		  //ECap2Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
		  //ECap2Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
		 // bGoLow++;
		 ECap2Regs.ECCLR.all |= 0b10101;
	   }
   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   //ECap2Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


__interrupt void ecap3_isr(void)
{
	if(ECap3Regs.ECFLG.bit.CEVT1 || ECap3Regs.ECFLG.bit.CEVT3)   //  When detect rising edge of the signal Hall sensor state = 1
	   {
		 HallState |= 0b001;
	     //HallsensorC = 1;
	     //ECap3Regs.ECCLR.bit.CEVT1 = 1;      //clears the CEVT1 flag condition
	     //ECap3Regs.ECCLR.bit.CEVT3 = 1;      //clears the CEVT3 flag condition
	     ECap3Regs.ECCLR.all |= 0b01011;
	     //cGoHigh++;
	   }

	if(ECap3Regs.ECFLG.bit.CEVT2 || ECap3Regs.ECFLG.bit.CEVT4) // when detect falling edge of the signal, Hall sensor state = 0
	   {
		  HallState &= 0b110;
	      //HallsensorC = 0;
		  //ECap3Regs.ECCLR.bit.CEVT2 = 1;      //clears the CEVT2 flag condition
		  //ECap3Regs.ECCLR.bit.CEVT4 = 1;      //clears the CEVT4 flag condition
		  ECap3Regs.ECCLR.all |= 0b10101;
		  //cGoLow++;
	   }

   // Acknowledge this interrupt to receive more interrupts from group 4
   readHallStateFlag = 1;
   //ECap3Regs.ECCLR.bit.INT = 1;        //clears the INT flag and enable further interrupts to be generated ---global
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}



void InitECapRegs()
{
   HallsensorA = 0;
   HallsensorB = 0;
   HallsensorC = 0;
   HallState = 0;
   Phase = 0;
   Electrical_angle = 0;
   direction = 0;
   int forwardPhaseTable[] = {0x014, 0x410, 0x401, 0x041, 0x140, 0x104};
   //reversePhaseTable[] = {0x104, 0x140, 0x041, 0x401, 0x410, 0x014};
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
   //HallsensorA = GpioDataRegs.GPADAT.bit.GPIO24;  // read the signal from GPIO24
   //HallsensorB = GpioDataRegs.GPADAT.bit.GPIO25;  // read the signal from GPIO25
   //HallsensorC = GpioDataRegs.GPADAT.bit.GPIO26;  // read th signal from GPIO26
   HallState = (GpioDataRegs.GPADAT.bit.GPIO24 << 2)|(GpioDataRegs.GPADAT.bit.GPIO25 << 1)|(GpioDataRegs.GPADAT.bit.GPIO26 << 0);
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;
   updateHallState();
   //forwardPhaseTable[] = {0x014, 0x410, 0x401, 0x041, 0x140, 0x104};
   i = 0;
   switch (Phase) {
   case 0x014:
   	   i = 0 - (2*direction);
	   break;
   case 0x410:
	   i = 1 - (2*direction);
	   break;
   case 0x401:
	   i = 2 - (2*direction);
	   break;
   case 0x041:
	   i = 3 - (2*direction);
	   break;
   case 0x140:
	   i = 4 - (2*direction);
	   break;
   case 0x104:
	   i = 5 - (2*direction);
	   break;
   }
   if (i < 0)
       i = 5;
   else if (i > 5)
       i = 0;

   Phase = forwardPhaseTable[i];
   EDIS;
}
