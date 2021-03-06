/*
 * libSPI.h
 *
 *  Created on: Jun 12, 2015
 *      Author: Jack
 */

#ifndef LIBSPI_H_
#define LIBSPI_H_

typedef struct  {

	int StatusRegister1;
	int StatusRegister2;
	int ControlRegister1;
	int ControlRegister2;
	int OverCurrentMode; //location of OC bits
	int PwmMode;
	int OverCurrentAdjust;
	int tempWarning;
	int fault;
	int deviceID;


}drv8301_bits_t;



struct drv8301_t {

	int readMode;
	int writeMode;
	int dataMask;
	int pwmMode; //default is 6. change to 1 for 3.
	int resetStatus; //change to 1 to enable reset
	int peakCurrent; //max current the drv8301 itself it allowed to draw. 0=1.7A, 1=0.7, 2=0.25
	int OverCurrentMode; //0: enable current limits 1: latch shutdown 2: report on OC 3: disable protection
	int SndCmd;
    int RcvCmd;
    int timeOut;
    int OverCurrentAdjust;
    int tempWarning;
    int fault;
    int gain;
	int OverTempWarning;
	int OverTempShutdown;
	int deviceID;

    //BIT LOCATIONS

    drv8301_bits_t bits;


};

extern volatile struct drv8301_t drv8301;

void DRV8301_enable();
void DRV8301_readData();
void DRV8301_setupSpi();
void setupDrv8301();
void setupSpiA();



#endif /* LIBSPI_H_ */
