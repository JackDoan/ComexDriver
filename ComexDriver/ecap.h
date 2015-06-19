/*
 * ecap.h
 *
 *  Created on: Jun 4, 2015
 *      Author: jad140230
 */

#ifndef ECAP_H_
#define ECAP_H_

__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
void InitECapRegs(void);
void updateHallState();

extern int HallsensorA;
extern int HallsensorB;
extern int HallsensorC;
extern Uint32 Phase;
extern int Electrical_angle;
extern int readHallStateFlag;
extern int CoilA;
extern int CoilB;
extern int CoilC;
extern int gogo;
extern int aGoHigh;
extern int aGoLow;
extern int bGoHigh;
extern int bGoLow;
extern int cGoHigh;
extern int cGoLow;
extern int phaseHistory[36];
extern int i;


#endif /* ECAP_H_ */
