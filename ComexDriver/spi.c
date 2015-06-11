/*
 * spi.c
 *
 *  Created on: Jun 10, 2015
 *      Author: jad140230
 */

#include <math.h>
#include <drv8301.h>

struct drv8301 {
	const int StatusRegister1 = 0 << 11;
	const int StatusRegister2 = 1 << 11;
	const int ControlRegister1 = 2 << 11;
	const int ControlRegister2 = 3 << 11;
	const int readMode = (1 << 15);
	const int writeMode = 0;
	const int dataMask = (0x07FF);
	int pwmMode = (0 << 3); //default is 6. change to 1 for 3.
	int resetStatus = (0 << 2); //change to 1 to enable reset
	int peakCurrent = (2 << 0); //max current the drv8301 itself it allowed to draw. 0=1.7A, 1=0.7, 2=0.25
	int SndCmd = false;
    int drv8301.RcvCmd = false;
	



};



void setupSpiA()
{
  SpiaRegs.SPICCR &= (~(1 << 7)); //reset the SPI module settings
  SpiaRegs.SPICTL |= (1 << 2); //master mode (set to zero for slave)
  SpiaRegs.SPICCR &= (~(1 << 6)); //clear clock polarity bits
  SpiaRegs.SPICCR |= (0 << 6);  //set clock polarity bits - tx data is output on the rising edge & rx data is latched on the falling edge. Set to 1 to reverse.
  SpiaRegs.SPICTL |= (1 << 1); //set the spi 'talk' bits
  SpiaRegs.SPIFFTX |= (1 << 14); //enable TX FIFO
  SpiaRegs.SPIFFTX |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFCT = 0x0010; //tx delay set to 16 clock cycles. why?
  SpiaRegs.SPIBRR = 0x000d; //set clock rate
  SpiaRegs.SPICCR &= (~(15 << 0)); // reset the char-length register
  SpiaRegs.SPICCR |= (15 << 0); //sets char-length to 16 bits. Set to (n bits)-1 to change
  SpiaRegs.SPIPRI &= (~(3 << 4));
  SpiaRegs.SPIPRI |= emuSuspend; //TODO find these bits
  SpiaRegs.SPICCR |= (1 << 7); //take SPI out of reset
  return;
}

uint16_t DRV8301_readSpi(const int regName)
{
  const uint16_t data = 0;
  volatile uint16_t readWord;
  static volatile uint16_t WaitTimeOut = 0;
  volatile int RxFifoCnt = SPI_FifoStatus_Empty = 0;

  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFTX |= (1 << 14); //enable TX FIFO

  // write the command
  SpiaRegs.SPITXBUF = (uint16_t)(drv8301.writeMode | regName | (data & drv8301.dataMask));
  // dummy write to return the reply from the 8301
  SpiaRegs.SPITXBUF = 0x0000;

  // wait for two words to populate the RX fifo, or a wait timeout will occur
  while((RxFifoCnt < (2 << 8)) && (WaitTimeOut < 0xffff))
  {
    RxFifoCnt = ((SpiaRegs.SPIFFRX) & (31 << 8)); //AND the data with the location of the FIFO register.

      if(++WaitTimeOut > 0xfffe)
      {
          //obj->RxTimeOut = true;
		  //do a thing
      }
  }

  // Read two words, the dummy word and the data
  readWord = SpiaRegs.SPIEMU;
  readWord = SpiaRegs.SPIEMU;

  return(readWord & drv8301.dataMask);
}

void DRV8301_writeSpi(const int regName, const uint16_t data)
{
  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX |= (1 << 13); //reset TX FIFO
  SpiaRegs.SPIFFTX |= (1 << 14); //enable TX FIFO

  // write the command
  SpiaRegs.SPITXBUF = (uint16_t)(drv8301.writeMode | regName | (data & drv8301.dataMask));

  return;
}




void DRV8301_writeData()
{
  uint16_t drvData;

  if(drv8301.SndCmd)
  {
    // Update Control Register 1
    drvData = drv8301.peakCurrent |  \
                 drv8301.resetStatus   |  \
                 drv8301.pwmMode   |  \
                 Ctrl_Reg_1.OC_MODE      |  \
                 Ctrl_Reg_1.OC_ADJ_SET;
    DRV8301_writeSpi(drv8301.ControlRegister1,drvData);

    // Update Control Register 2
    drvData = Ctrl_Reg_2.OCTW_SET      |  \
                 Ctrl_Reg_2.GAIN          |  \
                 Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                 Ctrl_Reg_2.OC_TOFF;
    DRV8301_writeSpi(drv8301.ControlRegister2,drvData);

    drv8301.SndCmd = false;
  }

  return;
}


void DRV8301_readData()
{
  uint16_t drvDataNew;


  if(drv8301.RcvCmd)
  {
    // Update Status Register 1
    drvDataNew = DRV8301_readSpi(drv8301.StatusRegister1);
    drv8301.Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    drv8301.Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    drv8301.Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    drv8301.Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    drv8301.Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    drv8301.Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    drv8301.Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    drv8301.Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    drv8301.Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    drv8301.Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    drv8301.Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

    // Update Status Register 2
    drvDataNew = DRV8301_readSpi(drv8301.StatusRegister2);
    drv8301.Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    drv8301.Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

    // Update Control Register 1
    drvDataNew = DRV8301_readSpi(drv8301.ControlRegister1);
    drv8301.peakCurrent = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    drv8301.resetStatus = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    drv8301.pwmMode = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    drv8301.Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    drv8301.Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

    // Update Control Register 2
    drvRegName = drv8301.ControlRegister2;
    drvDataNew = DRV8301_readSpi(drvRegName);
    drv8301.Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    drv8301.Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    drv8301.Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    drv8301.Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

    drv8301.RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function


void DRV8301_setupSpi(DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  uint16_t drvDataNew;
  uint16_t n;


  // Update Control Register 1
  drvDataNew = (drv8301.peakCurrent   | \
                drv8301.resetStatus        | \
                drv8301.pwmMode  | \
                DRV8301_OcMode_CurrentLimit  | \
                DRV8301_VdsLevel_0p730_V);
  DRV8301_writeSpi(drv8301.ControlRegister1,drvDataNew);

  // Update Control Register 2
  drvDataNew = (DRV8301_OcTwMode_Both        | \
                DRV8301_ShuntAmpGain_10VpV   | \
                DRV8301_DcCalMode_Ch1_Load   | \
                DRV8301_DcCalMode_Ch2_Load   | \
                DRV8301_OcOffTimeMode_Normal);
  DRV8301_writeSpi(drv8301.ControlRegister2,drvDataNew);


  drv8301.SndCmd = false;
  drv8301.RcvCmd = false;


  // Wait for the DRV8301 registers to update
  for(n=0;n<100;n++)
    asm(" NOP");


  // Update Status Register 1
  drvDataNew = DRV8301_readSpi(DRV8301_RegName_Status_1);
  drv8301.Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
  drv8301.Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
  drv8301.Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
  drv8301.Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
  drv8301.Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
  drv8301.Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
  drv8301.Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
  drv8301.Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
  drv8301.Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
  drv8301.Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
  drv8301.Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

  // Update Status Register 2
  drvDataNew = DRV8301_readSpi(drv8301.StatusRegister2);
  drv8301.Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
  drv8301.Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

  // Update Control Register 1
  drvDataNew = DRV8301_readSpi(drv8301.ControlRegister2);
  drv8301.Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
  drv8301.Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
  drv8301.Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
  drv8301.Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
  drv8301.Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // Update Control Register 2
  drvDataNew = DRV8301_readSpi(drv8301.ControlRegister2);
  drv8301.Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
  drv8301.Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
  drv8301.Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
  drv8301.Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

  return;
}






////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////Unfixed things below this point
////////////////////////////////////////////////////////////////////////////////////////////////////


void DRV8301_enable(DRV8301_Handle handle)
{
  static volatile uint16_t enableWaitTimeOut;
  uint16_t n = 0;

  // Enable the drv8301
  GPIO_setHigh(obj->gpioHandle,obj->gpioNumber);

  enableWaitTimeOut = 0;

  // Make sure the Fault bit is not set during startup
  while(((DRV8301_readSpi(DRV8301_RegName_Status_1) & DRV8301_STATUS1_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
  {
    if(++enableWaitTimeOut > 999)
    {
      obj->enableTimeOut = true;
    }
  }

  // Wait for the DRV8301 registers to update
  for(n=0;n<0xffff;n++)
    asm(" NOP");

  return;
}

uint16_t DRV8301_getDcCalMode(const int ampNumber)
{
  uint16_t data;

  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

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


DRV8301_FaultType_e DRV8301_getFaultType(DRV8301_Handle handle)
{
  DRV8301_Word_t      readWord;
  DRV8301_FaultType_e faultType = DRV8301_FaultType_NoFault;


  // read the data
  readWord = DRV8301_readSpi(DRV8301_RegName_Status_1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      faultType = (DRV8301_FaultType_e)(readWord & DRV8301_FAULT_TYPE_MASK);

      if(faultType == DRV8301_FaultType_NoFault)
        {
          // read the data
          readWord = DRV8301_readSpi(drv8301.StatusRegister2);

          if(readWord & DRV8301_STATUS2_GVDD_OV_BITS)
            {
              faultType = DRV8301_FaultType_GVDD_OV;
            }
        }
    }

  return(faultType);
} // end of DRV8301_getFaultType() function


uint16_t DRV8301_getId(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.StatusRegister2);

  // mask bits
  data &= DRV8301_STATUS2_ID_BITS;

  return(data);
} // end of DRV8301_getId() function


DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  return((DRV8301_VdsLevel_e)data);
} // end of DRV8301_getOcLevel() function


DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  return((DRV8301_OcMode_e)data);
} // end of DRV8301_getOcMode() function


DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  return((DRV8301_OcOffTimeMode_e)data);
} // end of DRV8301_getOcOffTimeMode() function


DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  return((DRV8301_OcTwMode_e)data);
} // end of DRV8301_getOcTwMode() function


DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  return((DRV8301_PeakCurrent_e)data);
} // end of DRV8301_getPeakCurrent() function


DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  return((DRV8301_PwmMode_e)data);
} // end of DRV8301_getPwmMode() function


DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  return((DRV8301_ShuntAmpGain_e)data);
} // end of DRV8301_getShuntAmpGain() function


DRV8301_Handle DRV8301_init(void *pMemory,const size_t numBytes)
{
  DRV8301_Handle handle;


  if(numBytes < sizeof(DRV8301_Obj))
    return((DRV8301_Handle)NULL);


  // assign the handle
  handle = (DRV8301_Handle)pMemory;

  DRV8301_resetRxTimeout(handle);
  DRV8301_resetEnableTimeout(handle);


  return(handle);
} // end of DRV8301_init() function



bool DRV8301_isFault(DRV8301_Handle handle)
{
  DRV8301_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8301_readSpi(DRV8301_RegName_Status_1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isFault() function


bool DRV8301_isReset(DRV8301_Handle handle)
{
  DRV8301_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8301_readSpi(drv8301.ControlRegister1);

  if(readWord & DRV8301_CTRL1_GATE_RESET_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isReset() function




void DRV8301_reset()
{
  uint16_t data;

  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // set the bits
  data |= DRV8301_CTRL1_GATE_RESET_BITS;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister1,data);

  return;
}  // end of DRV8301_reset() function


void DRV8301_setDcCalMode(const DRV8301_ShuntAmpNumber_e ampNumber,const DRV8301_DcCalMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

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
  DRV8301_writeSpi(drv8301.ControlRegister2,data);

  return;
} // end of DRV8301_setDcCalMode() function


void DRV8301_setOcLevel(const DRV8301_VdsLevel_e VdsLevel)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // set the bits
  data |= VdsLevel;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister1,data);

  return;
} // end of DRV8301_setOcLevel() function


void DRV8301_setOcMode(const DRV8301_OcMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister1,data);

  return;
} // end of DRV8301_setOcMode() function


void DRV8301_setOcOffTimeMode(const DRV8301_OcOffTimeMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister2,data);

  return;
} // end of DRV8301_setOcOffTimeMode() function


void DRV8301_setOcTwMode(const DRV8301_OcTwMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister2,data);

  return;
} // end of DRV8301_setOcTwMode() function


void DRV8301_setPeakCurrent(const DRV8301_PeakCurrent_e peakCurrent)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  // set the bits
  data |= peakCurrent;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister1,data);

  return;
} // end of DRV8301_setPeakCurrent() function


void DRV8301_setPwmMode(const DRV8301_PwmMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister1,data);

  return;
} // end of DRV8301_setPwmMode() function


void DRV8301_setShuntAmpGain(const DRV8301_ShuntAmpGain_e gain)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(drv8301.ControlRegister2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8301_writeSpi(drv8301.ControlRegister2,data);

  return;
} // end of DRV8301_setShuntAmpGain() function




// end of file


