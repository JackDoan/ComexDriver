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

extern int HallsensorA;
extern int HallsensorB;
extern int HallsensorC;
extern int Electrical_angle;
extern int readHallStateFlag;



#endif /* ECAP_H_ */
