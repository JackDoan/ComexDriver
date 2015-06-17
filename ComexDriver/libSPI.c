/*
 * spi.c
 *
 *  Created on: Jun 10, 2015
 *      Author: jad140230
 */

#include <math.h>
#include <drv8301.h>
#include "DSP28x_Project.h"
#include "libSPI.h"

#pragma DATA_SECTION(drv8301,"drv8301File");
struct drv8301_t;
volatile struct drv8301_t drv8301;
//typedef struct drv8301_t drv8301_t;


void setupDrv8301() {

	drv8301.bits.StatusRegister1 = 0 << 11;
	drv8301.bits.StatusRegister2 = 1 << 11;
	drv8301.bits.ControlRegister1 = 2 << 11;
	drv8301.bits.ControlRegister2 = 3 << 11;
	drv8301.bits.OverCurrentMode = 1 << 15;
	drv8301.bits.PwmMode = 3 << 4; //check this
	drv8301.bits.OverCurrentAdjust = 31 << 6;
	drv8301.bits.tempWarning = 3 << 0;
	drv8301.bits.fault = 1 << 10;

	drv8301.OverCurrentAdjust = 21 << 6;
	drv8301.timeOut = 0;
	drv8301.SndCmd = 0;
	drv8301.RcvCmd = 0;
	drv8301.OverTempShutdown = 0;
	drv8301.OverTempWarning = 0;
	drv8301.OverCurrentMode = 0;
	drv8301.peakCurrent = 2 << 0;
	drv8301.resetStatus = 0 << 2;
	drv8301.pwmMode = 0 << 3;
	drv8301.readMode = 1;
	drv8301.writeMode = 0;
	drv8301.dataMask = 0x07FF;
	drv8301.tempWarning = 0;
	drv8301.gain = 0; //10VpV


}

void setupSpiA()
{
  //need to add code to set up GPIO in here
  SpiaRegs.SPICCR.all &= (~(1 << 7)); //reset the SPI module settings
  SpiaRegs.SPICTL.all |= (1 << 2); //master mode (set to zero for slave)
  SpiaRegs.SPICCR.all &= (~(1 << 6)); //clear clock polarity bits
  SpiaRegs.SPICCR.all |= (0 << 6);  //set clock polarity bits - tx data is output on the rising edge & rx data is latched on the falling edge. Set to 1 to reverse.
  SpiaRegs.SPICTL.all |= (1 << 1); //set the spi 'talk' bits
  SpiaRegs.SPIFFTX.all |= (1 << 14); //enable TX FIFO
  SpiaRegs.SPIFFTX.all |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFCT.all = 0x0010; //tx delay set to 16 clock cycles. why?
  SpiaRegs.SPIBRR = 0x000d; //set clock rate
  SpiaRegs.SPICCR.all &= (~(15 << 0)); // reset the char-length register
  SpiaRegs.SPICCR.all |= (15 << 0); //sets char-length to 16 bits. Set to (n bits)-1 to change
  SpiaRegs.SPIPRI.all &= (~(3 << 4));
  SpiaRegs.SPIPRI.all |= (2 << 4); //setup SPI to halt transfers during debug suspend
  SpiaRegs.SPICCR.all |= (1 << 7); //take SPI out of reset
  return;
}

unsigned int DRV8301_readSpi(const int regName)
{
  const unsigned int data = 0;
  volatile unsigned int readWord;
  static volatile unsigned int WaitTimeOut = 0;
  volatile int RxFifoCnt = 0;

  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX.all |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFTX.all |= (1 << 14); //enable TX FIFO

  // write the command
  SpiaRegs.SPITXBUF = (unsigned int)(drv8301.writeMode | regName | (data & drv8301.dataMask));
  // dummy write to return the reply from the 8301
  SpiaRegs.SPITXBUF = 0x0000;

  // wait for two words to populate the RX fifo, or a wait timeout will occur
  while((RxFifoCnt < (2 << 8)) && (WaitTimeOut < 0xffff))
  {
    RxFifoCnt = ((SpiaRegs.SPIFFRX.all) & (31 << 8)); //AND the data with the location of the FIFO register.

      if(++WaitTimeOut > 0xfffe)
      {
          //obj->RxTimeOut = true;
		  //do a thing
      }
  }

  // Read two words, the dummy word and the data
  readWord = SpiaRegs.SPIRXEMU;
  readWord = SpiaRegs.SPIRXEMU;

  return(readWord & drv8301.dataMask);
}

void DRV8301_writeSpi(const int regName, const unsigned int data)
{
  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX.all |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFTX.all |= (1 << 14); //enable TX FIFO

  // write the command
  SpiaRegs.SPITXBUF = (unsigned int)(drv8301.writeMode | regName | (data & drv8301.dataMask));

  return;
}




void DRV8301_writeData()
{
  unsigned int drvData;

  if(drv8301.SndCmd)
  {
    // Update Control Register 1
    drvData = drv8301.peakCurrent |  \
                 drv8301.resetStatus   |  \
                 drv8301.pwmMode   |  \
				 drv8301.OverCurrentMode      |  \
                 drv8301.OverCurrentAdjust;
    DRV8301_writeSpi(drv8301.bits.ControlRegister1,drvData);

    // Update Control Register 2
    drvData = drv8301.tempWarning     |  \
              drv8301.gain          |  \
                 Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                 Ctrl_Reg_2.OC_TOFF;
    DRV8301_writeSpi(drv8301.bits.ControlRegister2,drvData);

    drv8301.SndCmd = false;
  }

  return;
}


void DRV8301_readData()
{
  unsigned int drvDataNew;


  if(drv8301.RcvCmd)
  {
    // Update Status Register 1
    drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister1);
    drv8301.fault = (drvDataNew & drv8301.bits.fault);
    drv8301.OverTempShutdown = (drvDataNew & (1 << 7));
    drv8301.OverTempWarning = (drvDataNew & (1 << 6));

    // Update Status Register 2
    drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister2);
    drv8301.deviceID = (drvDataNew & (15 << 0));

    // Update Control Register 1
    drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister1);
    drv8301.peakCurrent = (DRV8301_PeakCurrent_e)(drvDataNew & (3 << 0));
    drv8301.resetStatus = (drvDataNew & (1 << 2));
    drv8301.pwmMode = (drvDataNew & drv8301.bits.PwmMode);
    drv8301.OverCurrentMode = (drvDataNew & drv8301.bits.OverCurrentMode);
    drv8301.OverCurrentAdjust = (drvDataNew & drv8301.bits.OverCurrentAdjust);

    // Update Control Register 2
    drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister2);
    drv8301.tempWarning = (drvDataNew & drv8301.bits.tempWarning);
    drv8301.gain = (drvDataNew & (3 << 2));
    drv8301.Ctrl_Reg_2.DC_CAL_CH1p2 = (drvDataNew & ((1 << 4) | (1 << 5)));
    drv8301.Ctrl_Reg_2.OC_TOFF = (drvDataNew & (1 << 6));

    drv8301.RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function


void DRV8301_setupSpi()
{
  unsigned int drvDataNew;
  unsigned int n;


  // Update Control Register 1
  drvDataNew = (drv8301.peakCurrent   | \
                drv8301.resetStatus        | \
                drv8301.pwmMode  | \
				drv8301.OverCurrentMode  | \
                drv8301.OverCurrentAdjust);
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,drvDataNew);

  // Update Control Register 2
  drvDataNew = (drv8301.tempWarning | drv8301.gain);
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,drvDataNew);


  drv8301.SndCmd = false;
  drv8301.RcvCmd = false;


  // Wait for the DRV8301 registers to update
  for(n=0;n<100;n++)
    asm(" NOP");


  // Update Status Register 1
  drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister1);
  drv8301.fault = (drvDataNew & drv8301.bits.fault);
  drv8301.OverTempShutdown = (drvDataNew & (1 << 7));
  drv8301.OverTempWarning = (drvDataNew & (1 << 6));

  // Update Status Register 2
  drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister2);
  drv8301.deviceID = (drvDataNew & (15 << 0));

  // Update Control Register 1
  drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister2);
  drv8301.peakCurrent = (drvDataNew & (3 << 0));
  drv8301.resetStatus = (drvDataNew & 1 << 2);
  drv8301.pwmMode = (drvDataNew & drv8301.bits.PwmMode);
  drv8301.OverCurrentMode = (drvDataNew & drv8301.bits.OverCurrentMode);
  drv8301.OverCurrentAdjust = (drvDataNew & drv8301.bits.OverCurrentAdjust);

  // Update Control Register 2
  drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister2);
  drv8301.tempWarning = (drvDataNew & drv8301.bits.tempWarning);
  drv8301.gain = (drvDataNew & (3 << 2));

  return;
}

void DRV8301_enable()
{
  static volatile unsigned int enableWaitTimeOut;
  unsigned int n = 0;

  // Enable the drv8301
  //GPIO_setHigh(obj->gpioHandle,obj->gpioNumber);

  enableWaitTimeOut = 0;

  // Make sure the Fault bit is not set during startup
  while(((DRV8301_readSpi(drv8301.bits.StatusRegister1) & (1 << 10)) != 0) && (enableWaitTimeOut < 1000))
  {
    if(++enableWaitTimeOut > 999)
    {
      drv8301.timeOut = true;
    }
  }

  // Wait for the DRV8301 registers to update
  for(n=0;n<0xffff;n++)
    asm(" NOP");

  return;
}



int DRV8301_isFault()
{
  int readWord;
  int status=false;


  // read the data
  readWord = DRV8301_readSpi(drv8301.bits.StatusRegister1);

  if(readWord & (1 << 10))
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isFault() function





void DRV8301_reset()
{
  unsigned int data;

  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // set the bits
  data |= (1 << 2);

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
}  // end of DRV8301_reset() function


