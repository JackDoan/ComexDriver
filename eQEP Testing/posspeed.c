// TITLE:   Pos/speed measurement using EQEP peripheral
//
// Speed calculation steps performed by POSSPEED_Calc() at  SYSCLKOUT =  80 MHz are described below:
//
//    theta_mech = QPOSCNT/mech_Scaler = QPOSCNT/25600, where 25600 is the number of
//                 counts in 1 revolution.(25600/4 = 6400 line/rev. quadrature encoder)
//
//    theta_elec = (# pole pairs) * theta_mech = 2*QPOSCNT/25600 for this example
//
//    SpeedRpm_fr = [(x2-x1)/25600]/T                                             - Equation 1
//    Note (x2-x1) = difference in number of QPOSCNT counts. Dividing (x2-x1) by
//    25600 gives position relative to Index in one revolution.
// If base RPM  = 3200 rpm:   3200 rpm = [(x2-x1)/25600]/(1/3200) min                    - Equation 2
//                         max (x2-x1) = 25600 counts, or 1 revolution in 1.875 ms
//
// If both sides of Equation 2 are divided by 3200 rpm, then:
//                            1 = [(x2-x1)/25600] rev./[(1/3200) min * 3200rpm]
//                          Because (x2-x1) must be <25600 (max) for QPOSCNT increment,
//                          (x2-x1)/25600 < 1 for CW rotation
//                          And because (x2-x1) must be >-25600 for QPOSCNT decrement,
//                          (x2-x1)/25600>-1  for CCW rotation
//                          speed_fr = [(x2-x1)/25600]/[(1/3200) min * 3200rpm]
//                                   = (x2-x1)/25600                              - Equation 3
//
// To convert speed_fr to RPM, multiply Equation 3 by 3200 rpm
//                           SpeedRpm_fr = 3200rpm *(x2-x1)/25600                 - Final Equation
//
// 2. **min rpm ** = selected at 10 rpm based on CCPS prescaler options available (128 is greatest)
//
// 3. **SpeedRpm_pr**
//    SpeedRpm_pr = X/(t2-t1)                                                    - Equation 4
//    where X = QCAPCTL [UPPS]/25600 rev. (position relative to Index in 1 revolution)
// If max/base speed = 3200 rpm:  3200 = (32/25600)/[(t2-t1)/(80MHz/64)]
//    where 32 = QCAPCTL [UPPS] (Unit timeout - once every 32 edges)
//          32/25600 = (position as a fraction of 1 revolution)
//          t2-t1/(80MHz/64),  t2-t1= # of QCAPCLK cycles, and
//                        1 QCAPCLK cycle = 1/(80MHz/64)
//                                        = QCPRDLAT
//
//              So: 3200 rpm = [32(80MHz/64)*60s/min]/[25600(t2-t1)]
//                   t2-t1 = [32(80MHz/64)*60 s/min]/(25600*3200rpm)           - Equation 5
//                         = 100 CAPCLK cycles = maximum (t2-t1) = SpeedScaler
//
// Divide both sides by (t2-t1), and:
//                   1 = 50/(t2-t1) = [32(80MHz/64)*60 s/min]/(25600*3200rpm)]/(t2-t1)
//                   Because (t2-t1) must be < 100 for QPOSCNT increment:
//                   100/(t2-t1) < 1 for CW rotation
//                   And because (t2-t1) must be >-100 for QPOSCNT decrement:
//                   100/(t2-t1)> -1 for CCW rotation
//
//                   speed_pr = 100/(t2-t1)
//                      or [32(80MHz/64)*60 s/min]/(25600*3200rpm)]/(t2-t1)  - Equation 6
//
// To convert speed_pr to RPM:
// Multiply Equation 6 by 6000rpm:
//                  SpeedRpm_fr  = 6000rpm * [32(80MHz/64)*60 s/min]/[4000*6000rpm*(t2-t1)]
//                                          = [32(80MHz/64)*60 s/min]/[4000*(t2-t1)]
//                                        or [(32/4000)rev * 60 s/min]/[(t2-t1)(QCPRDLAT)]- Final Equation

#include <posspeed.h>   // Example specific Include file
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

void  POSSPEED_Init(void)
{
    EQep1Regs.QUPRD=800000;         // Unit Timer for 100Hz at 80 MHz SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;   // position counter is not affected by JTAG-suspend
    EQep1Regs.QEPCTL.bit.PCRM=01;		// PCRM=01 mode - QPOSCNT reset on max count (25600 counts)
  //EQep1Regs.QEPCTL.bit.PCRM=00;       // PCRM=00 mode - QPOSCNT reset on index event
    EQep1Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM=1;        // Latch on unit time out
  //EQep1Regs.QPOSMAX=0xffffffff;
    EQep1Regs.QPOSMAX=25600;     	    // max value
    EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable

    EQep1Regs.QCAPCTL.bit.UPPS=5;       // unit position event fires every 32 QCLK counts
    EQep1Regs.QCAPCTL.bit.CCPS=6;       // 1/64 for CAP clock (delta-T value = unit timer / 64) also (sysclkout / 64)
    EQep1Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable
}

void POSSPEED_Calc(POSSPEED *p)
{
     float Tmp1,newp,timeChange;

//**** Position calculation - mechanical and electrical motor angle  ****//
     p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;    // Motor direction: 0=CCW/reverse, 1=CW/forward

     //EQep1Regs.QPOSCNT is a counter from 1 to 25600 (0deg to 360deg).
     p->theta_raw = (unsigned int)EQep1Regs.QPOSCNT + p->cal_angle;        // raw theta = current pos. + cal_angle (simply an offset)

     // The following lines calculate p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
     //p->theta_mech = QPOSCNT/mech_Scaler = QPOSCNT/25600, where 25600 is the number of counts in 1 revolution.(25600/4 = 6400 line/rev. quadrature encoder)
     p->theta_mech = (float)p->theta_raw / (float)p->mech_scaler; // 0deg = 0, 360deg = 1
     p->theta_elec = (float)p->pole_pairs*p->theta_mech;

     // Check an index occurrence
     if (EQep1Regs.QFLG.bit.IEL == 1)
          {
             p->index_sync_flag = 0x00F0;
             EQep1Regs.QCLR.bit.IEL=1;                   // Clear interrupt flag
          }

//**** High Speed Calcultation using QEP Position counter ****//
// Check unit Time out-event for speed calculation:
// Unit Timer is configured for 100Hz in INIT function

    if(EQep1Regs.QFLG.bit.UTO==1)                    // If unit timeout (one 100Hz period)
    {
        // The following lines calculate position = (x2-x1)/25600 (position in 1 revolution)
		// The position-counter value is latched into QPOSLAT on unit time out event.
        newp = (float)EQep1Regs.QPOSLAT / (float)p->mech_scaler;

        if (p->DirectionQep==0)                     // POSCNT is counting down (reverse)
        {
            if (newp>p->oldpos)
                Tmp1 = -(1 - newp + p->oldpos);    // x2-x1 should be negative
            else
            	Tmp1 = newp - p->oldpos;
        }
        else if (p->DirectionQep==1)                // POSCNT is counting up (forwards)
        {
            if (newp<p->oldpos)
            	Tmp1 = 1 + newp - p->oldpos;
            else
            	Tmp1 = newp - p->oldpos;                     // x2-x1 should be positive
        }

        if (Tmp1>1)
            p->Speed_fr = 1;
        else if (Tmp1<-1)
            p->Speed_fr = -1;
        else
            p->Speed_fr = Tmp1;

        // Update the electrical angle
        p->oldpos = newp;

        // Change motor speed from pu value to rpm value
        p->SpeedRpm_fr = (int32)(p->BaseRpm*p->Speed_fr);
        EQep1Regs.QCLR.bit.UTO=1;                   // Clear interrupt flag
    }

//**** Low-speed computation using QEP capture counter ****//
    if(EQep1Regs.QEPSTS.bit.UPEVNT==1)                 // Unit position event
    {
        if(EQep1Regs.QEPSTS.bit.COEF==0)               // No Capture overflow
            timeChange=(float)EQep1Regs.QCPRDLAT;   // timeChange = t2-t1
        else                                           // Capture overflow, saturate the result
            timeChange=(float)0xFFFF;

        p->Speed_pr = (float)p->SpeedScaler / (float)timeChange;    // p->Speed_pr = p->SpeedScaler/timeChange
        //Tmp1 = p->Speed_pr;

        if (p->Speed_pr>1)
            p->Speed_pr = 1;
       // else
       //     p->Speed_pr = Tmp1;

        // Convert p->Speed_pr to RPM
        if (p->DirectionQep==0)                                 // Reverse direction = negative
            p->SpeedRpm_pr = -(p->BaseRpm * p->Speed_pr);
        else                                                    // Forward direction = positive
            p->SpeedRpm_pr = (p->BaseRpm * p->Speed_pr);

        EQep1Regs.QEPSTS.all=0x88;                  // Clear Unit position event flag
                                                    // Clear overflow error flag
    }
}
