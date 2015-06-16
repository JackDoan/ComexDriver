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
    drv8301.fault = (int)(drvDataNew & (unsigned int)drv8301.bits.fault);
    drv8301.Stat_Reg_1.OTSD = (int)(drvDataNew & (1 << 7));
    drv8301.Stat_Reg_1.OTW = (int)(drvDataNew & (1 << 6));

    // Update Status Register 2
    drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister2);
    drv8301.deviceID = (drvDataNew & (15 << 0));
    /*akghin; asldjkf vlaksdj ;laheirtnpao;weiruo;aiwehfksjfgblajker;flakj kl;ajfh ;aksjf lkasjgljhglkgh akjfh l;sdfas;ldifj ;laskdjf;alskdjf;
     * alskdfj;laksjdf;laksjdf ;lTITTIESakjsdf; lajsd;vn;asleiru;akdfm ;lsdkjfva;owieru;awoiejf;alskdfjv;awoeiur;alkdjf;aslef
     * apwoeiruf;aslkdfjnavo;weiur;alksjdf;laksj;rvun;oweiuf;awoeiurpaosdfl kjhwriluyfa;skldfjn vao;ktyalksjda
     * paoseiuf v;alkj;flkja;eliruv;alksjfkajfh;alieu;ofiaeuvn;awoeiurDICKBUTTpaofglskjebr;aoseiuyf askjgfalkjerha
     * aseuil askdjfhlaweiurf;lkasdaf;lkjlisDANKBEERCANTMELTDANKSTEAKSuhfgjafjlja;dthglaksdyrfl;asj f;alksjf;as
     * a;sldfh ;alsidfyh ;lask dfjSUCKMYDICKas;dlfk ;alskdjf akjge;lriua;elj;lsdkfasuiyef;aoeif;laksjvlakuyerlaiuerlijalkgjailwuer;alsdifjla
     * alsdkfjhDANKMEMESlaeiura;lsdfj;asieyr;oaif;laskeur;aosief ;laskdfj a;woeSCRYSCRYSCRYiyf a;sldkfja;vlier;aoicu;lmx;lasieunv;ajf;lkasjf ;akurv;ali
     * a;lsdiruc;alksdj f;oaiue;vriau;sdlfjk;aeiura;lsdjkf
     * a;sidur;alsidur;ASSa;sodiur;alskdjf ;lasieurv;laksjdf;laisuner;laus;dlfua*/

    // Update Control Register 1
    drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister1);
    drv8301.peakCurrent = (DRV8301_PeakCurrent_e)(drvDataNew & (3 << 0));
    drv8301.resetStatus = (drvDataNew & (1 << 2));
    drv8301.pwmMode = (drvDataNew & drv8301.bits.PwmMode);
    drv8301.OverCurrentMode = (drvDataNew & drv8301.bits.OverCurrentMode);
    drv8301.OverCurrentAdjust = (drvDataNew & drv8301.bits.OverCurrentAdjust);

    // Update Control Register 2
    drvDataNew = DRV8301_readSpi(drv8301.bits.ControlRegister2);
    drv8301.tempWarning = (DRV8301_OcTwMode_e)(drvDataNew & drv8301.bits.tempWarning);
    drv8301.Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (unsigned int)DRV8301_CTRL2_GAIN_BITS);
    drv8301.Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (unsigned int)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    drv8301.Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (unsigned int)DRV8301_CTRL2_OC_TOFF_BITS);

    drv8301.RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function


void DRV8301_setupSpi(DRV_SPI_8301_Vars_t *Spi_8301_Vars)
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
  drvDataNew = (DRV8301_OcTwMode_Both        | \
                DRV8301_ShuntAmpGain_10VpV   | \
                DRV8301_DcCalMode_Ch1_Load   | \
                DRV8301_DcCalMode_Ch2_Load   | \
                DRV8301_OcOffTimeMode_Normal);
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,drvDataNew);


  drv8301.SndCmd = false;
  drv8301.RcvCmd = false;


  // Wait for the DRV8301 registers to update
  for(n=0;n<100;n++)
    asm(" NOP");


  // Update Status Register 1
  drvDataNew = DRV8301_readSpi(drv8301.bits.StatusRegister1);
  drv8301.fault = (drvDataNew & drv8301.bits.fault);
  drv8301.Stat_Reg_1.OTSD = (drvDataNew & (1 << 7));
  drv8301.Stat_Reg_1.OTW = (drvDataNew & (1 << 6));

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
  drv8301.Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (unsigned int)DRV8301_CTRL2_GAIN_BITS);
  drv8301.Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (unsigned int)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
  drv8301.Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (unsigned int)DRV8301_CTRL2_OC_TOFF_BITS);

  return;
}






////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////Unfixed things below this point
////////////////////////////////////////////////////////////////////////////////////////////////////


void DRV8301_enable()
{
  static volatile unsigned int enableWaitTimeOut;
  unsigned int n = 0;

  // Enable the drv8301
  //GPIO_setHigh(obj->gpioHandle,obj->gpioNumber);

  enableWaitTimeOut = 0;

  // Make sure the Fault bit is not set during startup
  while(((DRV8301_readSpi(drv8301.bits.StatusRegister1) & DRV8301_STATUS1_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
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

unsigned int DRV8301_getDcCalMode(const int ampNumber)
{
  unsigned int data;

  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  if(ampNumber == DRV8301_ShuntAmpNumber_1)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    }
  else if(ampNumber == DRV8301_ShuntAmpNumber_2)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

  return data;
} // end of DRV8301_getDcCalMode() function


DRV8301_FaultType_e DRV8301_getFaultType()
{
  DRV8301_Word_t      readWord;
  DRV8301_FaultType_e faultType = DRV8301_FaultType_NoFault;


  // read the data
  readWord = DRV8301_readSpi(drv8301.bits.StatusRegister1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      faultType = (DRV8301_FaultType_e)(readWord & DRV8301_FAULT_TYPE_MASK);

      if(faultType == DRV8301_FaultType_NoFault)
        {
          // read the data
          readWord = DRV8301_readSpi(drv8301.bits.StatusRegister2);

          if(readWord & DRV8301_STATUS2_GVDD_OV_BITS)
            {
              faultType = DRV8301_FaultType_GVDD_OV;
            }
        }
    }

  return(faultType);
} // end of DRV8301_getFaultType() function


unsigned int DRV8301_getId()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.StatusRegister2);

  // mask bits
  data &= DRV8301_STATUS2_ID_BITS;

  return(data);
} // end of DRV8301_getId() function


DRV8301_VdsLevel_e DRV8301_getOcLevel()
{
  unsigned int data;

  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  return((DRV8301_VdsLevel_e)data);
} // end of DRV8301_getOcLevel() function


unsigned int DRV8301_getOcMode()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~drv8301.bits.OverCurrentMode);

  return data;
} // end of DRV8301_getOcMode() function


DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  return((DRV8301_OcOffTimeMode_e)data);
} // end of DRV8301_getOcOffTimeMode() function


DRV8301_OcTwMode_e DRV8301_getOcTwMode()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  return((DRV8301_OcTwMode_e)data);
} // end of DRV8301_getOcTwMode() function


DRV8301_PeakCurrent_e DRV8301_getPeakCurrent()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  return((DRV8301_PeakCurrent_e)data);
} // end of DRV8301_getPeakCurrent() function


unsigned int DRV8301_getPwmMode()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~drv8301.bits.PwmMode);

  return data;
} // end of DRV8301_getPwmMode() function


DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain()
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  return((DRV8301_ShuntAmpGain_e)data);
} // end of DRV8301_getShuntAmpGain() function



int DRV8301_isFault()
{
  DRV8301_Word_t readWord;
  int status=false;


  // read the data
  readWord = DRV8301_readSpi(drv8301.bits.StatusRegister1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isFault() function


int DRV8301_isReset()
{
  DRV8301_Word_t readWord;
  int status=false;


  // read the data
  readWord = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  if(readWord & DRV8301_CTRL1_GATE_RESET_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isReset() function




void DRV8301_reset()
{
  unsigned int data;

  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // set the bits
  data |= DRV8301_CTRL1_GATE_RESET_BITS;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
}  // end of DRV8301_reset() function


void DRV8301_setDcCalMode(const DRV8301_ShuntAmpNumber_e ampNumber,const DRV8301_DcCalMode_e mode)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  if(ampNumber == DRV8301_ShuntAmpNumber_1)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    }
  else if(ampNumber == DRV8301_ShuntAmpNumber_2)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,data);

  return;
} // end of DRV8301_setDcCalMode() function


void DRV8301_setOcLevel(const DRV8301_VdsLevel_e VdsLevel)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // set the bits
  data |= VdsLevel;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
} // end of DRV8301_setOcLevel() function


void DRV8301_setOcMode(const unsigned int mode)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~drv8301.bits.OverCurrentMode);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
} // end of DRV8301_setOcMode() function


void DRV8301_setOcOffTimeMode(const DRV8301_OcOffTimeMode_e mode)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,data);

  return;
} // end of DRV8301_setOcOffTimeMode() function


void DRV8301_setOcTwMode(const DRV8301_OcTwMode_e mode)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,data);

  return;
} // end of DRV8301_setOcTwMode() function


void DRV8301_setPeakCurrent(const DRV8301_PeakCurrent_e peakCurrent)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  // set the bits
  data |= peakCurrent;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
} // end of DRV8301_setPeakCurrent() function


void DRV8301_setPwmMode(const unsigned int mode)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister1);

  // clear the bits
  data &= (~drv8301.bits.PwmMode);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister1,data);

  return;
} // end of DRV8301_setPwmMode() function


void DRV8301_setShuntAmpGain(const DRV8301_ShuntAmpGain_e gain)
{
  unsigned int data;


  // read data
  data = DRV8301_readSpi(drv8301.bits.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8301_writeSpi(drv8301.bits.ControlRegister2,data);

  return;
} // end of DRV8301_setShuntAmpGain() function




// end of file


