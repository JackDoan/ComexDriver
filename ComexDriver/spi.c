/*
 * spi.c
 *
 *  Created on: Jun 10, 2015
 *      Author: jad140230
 */

#include <math.h>
#include <drv8301.h>

struct drv8301 {
	const int ControlRegister1;
	const int ControlRegister2;



};



void setupSpiA()
{
  SpiaRegs.SPICCR &= (~SPI_SPICCR_RESET_BITS);
  SpiaRegs.SPICTL |= SPI_Mode_Master;
  //SPI_setClkPolarity(obj->spiAHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
  SpiaRegs.SPICCR &= (~SPI_SPICCR_CLKPOL_BITS);
  SpiaRegs.SPICCR |= SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge;
  //END
  SpiaRegs.SPICTL |= SPI_SPICTL_TALK_BITS;
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_ENA_BITS; //enable TX FIFO
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_RESET_BITS; //reset TX FIFO
  SpiaRegs.SPIFFCT = 0x0010; //tx delay set to 16 clock cycles. why?
  SpiaRegs.SPIBRR = 0x000d; //set clock rate
  //SPI_setCharLength(obj->spiAHandle,SPI_CharLength_16_Bits); BEGIN
  SpiaRegs.SPICCR &= (~SPI_SPICCR_CHAR_LENGTH_BITS);
  SpiaRegs.SPICCR |= SPI_CharLength_16_Bits;
  //END
  //SPI_setSuspend(obj->spiAHandle,SPI_TxSuspend_free);
  SpiaRegs.SPIPRI &= (~SPI_SPIPRI_SUSP_BITS);
  SpiaRegs.SPIPRI |= emuSuspend;
  //END
  SpiaRegs.SPICCR |= SPI_SPICCR_RESET_BITS;

  return;
}

uint16_t DRV8301_readSpi(const DRV8301_RegName_e regName)
{
  const uint16_t data = 0;
  volatile uint16_t readWord;
  static volatile uint16_t WaitTimeOut = 0;
  volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;

  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_RESET_BITS; //reset TX FIFO
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_ENA_BITS; //enable TX FIFO


  // write the command
  SpiaRegs.SPITXBUF = (uint16_t)(DRV8301_CtrlMode_Read | regName | (data & DRV8301_DATA_MASK));
  // dummy write to return the reply from the 8301
  SpiaRegs.SPITXBUF = 0x0000;

  // wait for two words to populate the RX fifo, or a wait timeout will occur
  while((RxFifoCnt < SPI_FifoStatus_2_Words) && (WaitTimeOut < 0xffff))
  {
    //RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
    RxFifoCnt = ((SpiaRegs.SPIFFRX) & SPI_SPIFFRX_FIFO_ST_BITS);

      if(++WaitTimeOut > 0xfffe)
      {
          obj->RxTimeOut = true;
      }
  }

  // Read two words, the dummy word and the data
  readWord = SpiaRegs.SPIEMU;
  readWord = SpiaRegs.SPIEMU;

  return(readWord & DRV8301_DATA_MASK);
}  // end of DRV8301_readSpi() function

void DRV8301_writeSpi(const DRV8301_RegName_e regName, const uint16_t data)
{
  // reset the Rx fifo pointer to zero
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_RESET_BITS; //reset TX FIFO
  SpiaRegs.SPIFFTX |= SPI_SPIFFTX_FIFO_ENA_BITS; //enable TX FIFO

  // write the command (time N)
  SpiaRegs.SPITXBUF = (uint16_t)(DRV8301_CtrlMode_Write | regName | (data & DRV8301_DATA_MASK));

  return;
}




void DRV8301_writeData()
{
  uint16_t drvData;

  if(SndCmd)
  {
    // Update Control Register 1
    drvData = Ctrl_Reg_1.DRV8301_CURRENT |  \
                 Ctrl_Reg_1.DRV8301_RESET   |  \
                 Ctrl_Reg_1.PWM_MODE     |  \
                 Ctrl_Reg_1.OC_MODE      |  \
                 Ctrl_Reg_1.OC_ADJ_SET;
    DRV8301_writeSpi(DRV8301_RegName_Control_1,drvDataNew);

    // Update Control Register 2
    drvData = Ctrl_Reg_2.OCTW_SET      |  \
                 Ctrl_Reg_2.GAIN          |  \
                 Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                 Ctrl_Reg_2.OC_TOFF;
    DRV8301_writeSpi(DRV8301_RegName_Control_2,drvDataNew);

    SndCmd = false;
  }

  return;
}


void DRV8301_readData(DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;


  if(Spi_8301_Vars->RcvCmd)
  {
    // Update Status Register 1
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(drvRegName);
    Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

    // Update Status Register 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(drvRegName);
    Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(drvRegName);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(drvRegName);
    Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

    Spi_8301_Vars->RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function


void DRV8301_setupSpi(DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  uint16_t drvDataNew;
  uint16_t n;


  // Update Control Register 1
  drvDataNew = (DRV8301_PeakCurrent_0p25_A   | \
                DRV8301_Reset_Normal         | \
                DRV8301_PwmMode_Six_Inputs   | \
                DRV8301_OcMode_CurrentLimit  | \
                DRV8301_VdsLevel_0p730_V);
  DRV8301_writeSpi(DRV8301_RegName_Control_1,drvDataNew);

  // Update Control Register 2
  drvDataNew = (DRV8301_OcTwMode_Both        | \
                DRV8301_ShuntAmpGain_10VpV   | \
                DRV8301_DcCalMode_Ch1_Load   | \
                DRV8301_DcCalMode_Ch2_Load   | \
                DRV8301_OcOffTimeMode_Normal);
  DRV8301_writeSpi(DRV8301_RegName_Control_2,drvDataNew);


  Spi_8301_Vars->SndCmd = false;
  Spi_8301_Vars->RcvCmd = false;


  // Wait for the DRV8301 registers to update
  for(n=0;n<100;n++)
    asm(" NOP");


  // Update Status Register 1
  drvDataNew = DRV8301_readSpi(DRV8301_RegName_Status_1);
  Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
  Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
  Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
  Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
  Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

  // Update Status Register 2
  drvDataNew = DRV8301_readSpi(DRV8301_RegName_Status_2);
  Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
  Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

  // Update Control Register 1
  drvDataNew = DRV8301_readSpi(DRV8301_RegName_Control_2);
  Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // Update Control Register 2
  drvDataNew = DRV8301_readSpi(DRV8301_RegName_Control_2);
  Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
  Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
  Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
  Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

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
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

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
          readWord = DRV8301_readSpi(DRV8301_RegName_Status_2);

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
  data = DRV8301_readSpi(DRV8301_RegName_Status_2);

  // mask bits
  data &= DRV8301_STATUS2_ID_BITS;

  return(data);
} // end of DRV8301_getId() function


DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  return((DRV8301_VdsLevel_e)data);
} // end of DRV8301_getOcLevel() function


DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  return((DRV8301_OcMode_e)data);
} // end of DRV8301_getOcMode() function


DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  return((DRV8301_OcOffTimeMode_e)data);
} // end of DRV8301_getOcOffTimeMode() function


DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  return((DRV8301_OcTwMode_e)data);
} // end of DRV8301_getOcTwMode() function


DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  return((DRV8301_PeakCurrent_e)data);
} // end of DRV8301_getPeakCurrent() function


DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  return((DRV8301_PwmMode_e)data);
} // end of DRV8301_getPwmMode() function


DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

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
  readWord = DRV8301_readSpi(DRV8301_RegName_Control_1);

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
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // set the bits
  data |= DRV8301_CTRL1_GATE_RESET_BITS;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_1,data);

  return;
}  // end of DRV8301_reset() function


void DRV8301_setDcCalMode(const DRV8301_ShuntAmpNumber_e ampNumber,const DRV8301_DcCalMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

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
  DRV8301_writeSpi(DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setDcCalMode() function


void DRV8301_setOcLevel(const DRV8301_VdsLevel_e VdsLevel)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // set the bits
  data |= VdsLevel;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setOcLevel() function


void DRV8301_setOcMode(const DRV8301_OcMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setOcMode() function


void DRV8301_setOcOffTimeMode(const DRV8301_OcOffTimeMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setOcOffTimeMode() function


void DRV8301_setOcTwMode(const DRV8301_OcTwMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setOcTwMode() function


void DRV8301_setPeakCurrent(const DRV8301_PeakCurrent_e peakCurrent)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  // set the bits
  data |= peakCurrent;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setPeakCurrent() function


void DRV8301_setPwmMode(const DRV8301_PwmMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setPwmMode() function


void DRV8301_setShuntAmpGain(const DRV8301_ShuntAmpGain_e gain)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8301_writeSpi(DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setShuntAmpGain() function




// end of file


