#ifndef _SPI_H_
#define _SPI_H_

//! \brief Defines the location of the SPILBK bits in the SPICCR register
//!
#define SPI_SPICCR_SPILBK_BITS             ( 1 << 4)


//! \brief Defines the location of the SPI INT Flag in the SPIST register
//!
#define SPI_SPIST_INTFLAG_BITS              ( 1 << 6)

//! \brief Defines the location of the TX BUF FULL FLAG in the SPIST register
//!
#define SPI_SPIST_TXBUF_BITS                (1 << 5)

//! \brief Defines the location of the SPI INT ENA bits in the SPICTL register
//!
#define SPI_SPICTL_INT_ENA_BITS             (1 << 0)

//! \brief Defines the location of the TALK bits in the SPICTL register
//!
#define SPI_SPICTL_TALK_BITS                (1 << 1)

//! \brief Defines the location of the MASTER/SLAVE bits in the SPICTL register
//!
#define SPI_SPICTL_MODE_BITS                (1 << 2)

//! \brief Defines the location of the CLOCK PHASE bits in the SPICTL register
//!
#define SPI_SPICTL_CLK_PHASE_BITS           (1 << 3)

//! \brief Defines the location of the OVERRUN INT ENA bits in the SPICTL register
//!
#define SPI_SPICTL_OVRRUN_INT_ENA_BITS      (1 << 4)


//! \brief Defines the location of the RXFFIL4-0 bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_IL_BITS           (31 << 0)

//! \brief Defines the location of the RXFFIENA bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_IENA_BITS         ( 1 << 5)

//! \brief Defines the location of the RXFFINT CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_INTCLR_BITS       ( 1 << 6)

//! \brief Defines the location of the RXFFINT CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_INT_BITS          ( 1 << 7)

//! \brief Defines the location of the RXFFST4-0 bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_ST_BITS      (31 << 8)

//! \brief Defines the location of the RXFIFO Reset bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_RESET_BITS   ( 1 << 13)

//! \brief Defines the location of the RXFFOVF CLR bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_OVFCLR_BITS  ( 1 << 14)

//! \brief Defines the location of the RXFFOVF bits in the SPIFFRX register
//!
#define SPI_SPIFFRX_FIFO_OVF_BITS     ( 1 << 15)


//! \brief Defines the location of the TXFFIL4-0 bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_IL_BITS           (31 << 0)

//! \brief Defines the location of the TXFFIENA bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_IENA_BITS         ( 1 << 5)

//! \brief Defines the location of the TXFFINT CLR bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_INTCLR_BITS       ( 1 << 6)

//! \brief Defines the location of the TXFFINT bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_INT_BITS          ( 1 << 7)

//! \brief Defines the location of the TXFFST4-0 bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_FIFO_ST_BITS      (31 << 8)


//! \brief Defines the location of the SPIRST bits in the SPIFFTX register
//!
#define SPI_SPIFFTX_CHAN_RESET_BITS   ( 1 << 15)


//! \brief Defines the location of the SUSP bits in the SPIPRI register
//!
#define SPI_SPIPRI_SUSP_BITS          (  3 << 4)

//! \brief Defines the location of the STE_INV bits in the SPIPRI register
//!
#define SPI_SPIPRI_STE_INV_BITS       (  1 << 1)

//! \brief Defines the location of the TRIWIRE bits in the SPIPRI register
//!
#define SPI_SPIPRI_TRIWIRE            (  1 << 0)


// **************************************************************************
// the typedefs



//! \brief Enumeration to define the serial peripheral interface (SPI) character lengths
//!
typedef enum
{
  SPI_CharLength_1_Bit=(0 << 0),        //!< Denotes a character length of 1 bit
  SPI_CharLength_2_Bits=(1 << 0),       //!< Denotes a character length of 2 bits
  SPI_CharLength_3_Bits=(2 << 0),       //!< Denotes a character length of 3 bits
  SPI_CharLength_4_Bits=(3 << 0),       //!< Denotes a character length of 4 bits
  SPI_CharLength_5_Bits=(4 << 0),       //!< Denotes a character length of 5 bits
  SPI_CharLength_6_Bits=(5 << 0),       //!< Denotes a character length of 6 bits
  SPI_CharLength_7_Bits=(6 << 0),       //!< Denotes a character length of 7 bits
  SPI_CharLength_8_Bits=(7 << 0),       //!< Denotes a character length of 8 bits
  SPI_CharLength_9_Bits=(8 << 0),       //!< Denotes a character length of 9 bits
  SPI_CharLength_10_Bits=(9 << 0),      //!< Denotes a character length of 10 bits
  SPI_CharLength_11_Bits=(10 << 0),     //!< Denotes a character length of 11 bits
  SPI_CharLength_12_Bits=(11 << 0),     //!< Denotes a character length of 12 bits
  SPI_CharLength_13_Bits=(12 << 0),     //!< Denotes a character length of 13 bits
  SPI_CharLength_14_Bits=(13 << 0),     //!< Denotes a character length of 14 bits
  SPI_CharLength_15_Bits=(14 << 0),     //!< Denotes a character length of 15 bits
  SPI_CharLength_16_Bits=(15 << 0)      //!< Denotes a character length of 16 bits
} SPI_CharLength_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) clock phase
//!
typedef enum
{
  SPI_ClkPhase_Normal=(0<<3),      //!< Denotes a normal clock scheme
  SPI_ClkPhase_Delayed=(1<<3)      //!< Denotes that the SPICLK signal is delayed by one half-cycle
} SPI_ClkPhase_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO level
//!
typedef enum
{
  SPI_FifoLevel_Empty=(0 << 0),      //!< Denotes the fifo is empty
  SPI_FifoLevel_1_Word=(1 << 0),     //!< Denotes the fifo contains 1 word
  SPI_FifoLevel_2_Words=(2 << 0),    //!< Denotes the fifo contains 2 words
  SPI_FifoLevel_3_Words=(3 << 0),    //!< Denotes the fifo contains 3 words
  SPI_FifoLevel_4_Words=(4 << 0)     //!< Denotes the fifo contains 4 words
} SPI_FifoLevel_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) FIFO status
//!
typedef enum
{
  SPI_FifoStatus_Empty=(0 << 8),      //!< Denotes the fifo is empty
  SPI_FifoStatus_1_Word=(1 << 8),     //!< Denotes the fifo contains 1 word
  SPI_FifoStatus_2_Words=(2 << 8),    //!< Denotes the fifo contains 2 words
  SPI_FifoStatus_3_Words=(3 << 8),    //!< Denotes the fifo contains 3 words
  SPI_FifoStatus_4_Words=(4 << 8)     //!< Denotes the fifo contains 4 words
}  SPI_FifoStatus_e;


//! \brief Enumeration to define the the serial peripheral interface (SPI) priority
//!
typedef enum
{
  SPI_Priority_Immediate=(0 << 4),      //!< Stops immediately after EMU halt
  SPI_Priority_FreeRun=(1 << 4),        //!< Doesn't stop after EMU halt
  SPI_Priority_AfterRxRxSeq=(2 << 4)    //!< Stops after EMU halt and next rx/rx sequence
} SPI_Priority_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Interrupt Flag Status
//!
typedef enum
{
  SPI_IntFlagStatus_InProgress=(0<<6),  //!< Denotes transmission or reception in progress
  SPI_IntFlagStatus_Completed=(1<<6)    //!< Denotes transmission or reception completed
} SPI_IntFlagStatus_e;


//! \brief Enumeration to define the the serial peripheral interface (SPI) STE pin status
//!
typedef enum
{
  SPI_SteInv_ActiveLow=(0 << 1),        //!< Denotes active low STE pin
  SPI_SteInv_ActiveHigh=(1 << 1)        //!< Denotes active high STE pin
} SPI_SteInv_e;


//! \brief Enumeration to define the tri-wire status
//!
typedef enum
{
  SPI_TriWire_NormalFourWire=(0 << 0),  //!< Denotes 4 wire SPI mode
  SPI_TriWire_ThreeWire=(1 << 0)        //!< Denotes 3 wire SPI mode
} SPI_TriWire_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Tx Buffer Status
//!
typedef enum
{
  SPI_TxBufferStatus_Empty=(0<<5),   //!< Denotes that the Tx buffer is empty
  SPI_TxBufferStatus_Full=(1<<5)     //!< Denotes that the Tx buffer is full
} SPI_TxBufferStatus_e;


//! \brief Enumeration to define the serial peripheral interface (SPI) Enumeration suspend bits
//!
typedef enum
{
  SPI_TxSuspend_00=(0x0<<4),      //!< Emulation Suspend option 1
  SPI_TxSuspend_10=(0x2<<4),      //!< Emulation Suspend option 2
  SPI_TxSuspend_free=(0x1<<4)     //!< Emulation Free run
} SPI_EmulationSuspend_e;



//! \brief Defines the serial peripheral interface (SPI) handle
//!
typedef struct _SPI_Obj_ *SPI_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Clears the Rx FIFO overflow flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearRxFifoOvf();


//! \brief     Clears the Rx FIFO interrupt flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearRxFifoInt();


//! \brief     Clears the Tx FIFO interrupt flag
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_clearTxFifoInt();


//! \brief     Disables the serial peripheral interface (SPI) interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableInt();


//! \brief     Disables the serial peripheral interface (SPI) loop back mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableLoopBack();


//! \brief     Disables the serial peripheral interface (SPI) over-run interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableOverRunInt();


//! \brief     Disables the serial peripheral interface (SPI) receive FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableRxFifoInt();


//! \brief     Disables the serial peripheral interface (SPI) master/slave transmit mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTx();


//! \brief     Disables the serial peripheral interface (SPI) transmit FIFO enhancements
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTxFifoEnh();


//! \brief     Disables the serial peripheral interface (SPI) transmit FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_disableTxFifoInt();


//! \brief     Enables the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enable();


//! \brief     Enables the serial peripheral interface (SPI) transmit and receive channels
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableChannels();


//! \brief     Enables the serial peripheral interface (SPI) interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableInt();


//! \brief     Enables the serial peripheral interface (SPI) loop back mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableLoopBack();


//! \brief     Enables the serial peripheral interface (SPI) over-run interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableOverRunInt();


//! \brief     Enables the serial peripheral interface (SPI) receive FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableRxFifo();


//! \brief     Enables the serial peripheral interface (SPI) receive FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableRxFifoInt();


//! \brief     Enables the serial peripheral interface (SPI) masater/slave transmit mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTx();


//! \brief     Re-enables the serial peripheral interface (SPI) transmit FIFO
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifo();


//! \brief     Enables the serial peripheral interface (SPI) transmit FIFO enhancements
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifoEnh();


//! \brief     Enables the serial peripheral interface (SPI) transmit FIFO interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_enableTxFifoInt();


//! \brief     Gets the serial peripheral interface (SPI) receive FIFO status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The receive FIFO status
extern SPI_FifoStatus_e SPI_getRxFifoStatus();


//! \brief     Gets the serial peripheral interface (SPI) transmit FIFO status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The transmit FIFO status
extern SPI_FifoStatus_e SPI_getTxFifoStatus();


//! \brief     Gets the serial peripheral interface (SPI) Interrupt Flag status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The Interrupt Flag status
extern SPI_IntFlagStatus_e SPI_getIntFlagStatus();


//! \brief     Gets the serial peripheral interface (SPI) Tx Buffer status
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \return    The Interrupt Flag status
extern SPI_TxBufferStatus_e SPI_getTxBufferStatus();


//! \brief     Resets the serial peripheral interface (SPI)
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_reset();


//! \brief     Resets the serial peripheral interface (SPI) transmit and receive channels
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
extern void SPI_resetChannels();


//! \brief     Sets the serial peripheral interface (SPI) receive FIFO level for generating an interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] fifoLevel  The FIFO level
extern void SPI_setRxFifoIntLevel(const SPI_FifoLevel_e fifoLevel);


//! \brief     Sets the priority of the SPI port vis-a-vis the EMU
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] priority   The priority of the SPI port vis-a-vis the EMU
extern void SPI_setPriority(const SPI_Priority_e priority);


//! \brief     Controls pin inversion of STE pin
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] steinv     Polarity of STE pin
extern void SPI_setSteInv(const SPI_SteInv_e steinv);


//! \brief     Sets the serial peripheral interface (SPI) emulation suspend bits
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] SPI_EmulationSuspend_e The emulation suspend enumeration
extern void SPI_setSuspend(const SPI_EmulationSuspend_e emuSuspend);


//! \brief     Sets SPI port operating mode
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] triwire    3 or 4 wire mode
extern void SPI_setTriWire(const SPI_TriWire_e triwire);


//! \brief     Sets the serial peripheral interface (SPI) transmit FIFO level for generating an interrupt
//! \param[in] spiHandle  The serial peripheral interface (SPI) object handle
//! \param[in] fifoLevel  The FIFO level
extern void SPI_setTxFifoIntLevel(const SPI_FifoLevel_e fifoLevel);



//@} // ingroup
#endif // end of _SPI_H_ definition



/*
 * drv8301.h
 *
 *  Created on: Jun 10, 2015
 *      Author: jad140230
 */

#ifndef DRV8301_H_
#define DRV8301_H_

// drivers
//#include "sw/drivers/spi/src/32b/f28x/f2802x/spi.h"
//#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h"


// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8301_ADDR_MASK               (0x7800)


//! \brief Defines the R/W mask
//!
#define DRV8301_RW_MASK                 (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8301_FAULT_TYPE_MASK         (0x07FF)


//! \brief Defines the location of the FETLC_OC (FET Low side, Phase C Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase C Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase B Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase B Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase A Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase A Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)

//! \brief Defines the location of the OTW (Over Temperature Warning) bits in the Status 1 register
//!
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)

//! \brief Defines the location of the OTSD (Over Temperature Shut Down) bits in the Status 1 register
//!
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)

//! \brief Defines the location of the PVDD_UV (Power supply Vdd, Under Voltage) bits in the Status 1 register
//!
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)

//! \brief Defines the location of the GVDD_UV (DRV8301 Vdd, Under Voltage) bits in the Status 1 register
//!
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)

//! \brief Defines the location of the FAULT bits in the Status 1 register
//!
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)


//! \brief Defines the location of the Device ID bits in the Status 2 register
//!
#define DRV8301_STATUS2_ID_BITS        (15 << 0)

//! \brief Defines the location of the GVDD_OV (DRV8301 Vdd, Over Voltage) bits in the Status 2 register
//!
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)


//! \brief Defines the location of the GATE_CURRENT bits in the Control 1 register
//!
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)

//! \brief Defines the location of the GATE_RESET bits in the Control 1 register
//!
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)


//! \brief Defines the location of the OC_ADJ bits in the Control 1 register
//!
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)


//! \brief Defines the location of the OCTW_SET bits in the Control 2 register
//!
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)

//! \brief Defines the location of the GAIN bits in the Control 2 register
//!
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)

//! \brief Defines the location of the DC_CAL_1 bits in the Control 2 register
//!
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)

//! \brief Defines the location of the DC_CAL_2 bits in the Control 2 register
//!
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)

//! \brief Defines the location of the OC_TOFF bits in the Control 2 register
//!
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)


//! \brief Enumeration for the DC calibration modes
//!
typedef enum
{
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),   //!< Shunt amplifier 1 connected to load via input pins
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),   //!< Shunt amplifier 2 connected to load via input pins
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
} DRV8301_DcCalMode_e;


//! \brief Enumeration for the fault types
//!
typedef enum
{
  DRV8301_FaultType_NoFault  = (0 << 0),  //!< No fault
  DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
  DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
  DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
  DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
  DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
  DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
  DRV8301_FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
  DRV8301_FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
  DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
  DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
  DRV8301_FaultType_GVDD_OV  = (1 << 10)  //!< DRV8301 Vdd Over Voltage fault
} DRV8301_FaultType_e;


//! \brief Enumeration for the Over Current Off Time modes
//!
typedef enum
{
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,   //!< normal CBC operation
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6    //!< off time control during OC
} DRV8301_OcOffTimeMode_e;


//! \brief Enumeration for the Over Current, Temperature Warning modes
//!
typedef enum
{
  DRV8301_OcTwMode_Both    = 0 << 0,   //!< report both OT and OC at /OCTW pin
  DRV8301_OcTwMode_OT_Only = 1 << 0,   //!< report only OT at /OCTW pin
  DRV8301_OcTwMode_OC_Only = 2 << 0    //!< report only OC at /OCTW pin
} DRV8301_OcTwMode_e;


//! \brief Enumeration for the drv8301 peak current levels
//!
typedef enum
{
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,   //!< drv8301 driver peak current 1.70A
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,   //!< drv8301 driver peak current 0.70A
  DRV8301_PeakCurrent_0p25_A  = 2 << 0    //!< drv8301 driver peak current 0.25A
} DRV8301_PeakCurrent_e;




//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum
{
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,   //!< 10 V per V
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,   //!< 20 V per V
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,   //!< 40 V per V
  DRV8301_ShuntAmpGain_80VpV = 3 << 2    //!< 80 V per V
} DRV8301_ShuntAmpGain_e;


//! \brief Enumeration for the shunt amplifier number
//!
typedef enum
{
  DRV8301_ShuntAmpNumber_1 = 1,      //!< Shunt amplifier number 1
  DRV8301_ShuntAmpNumber_2 = 2       //!< Shunt amplifier number 2
} DRV8301_ShuntAmpNumber_e;


//! \brief Enumeration for the Vds level for th over current adjustment
//!
typedef enum
{
  DRV8301_VdsLevel_0p060_V =  0 << 6,      //!< Vds = 0.060 V
  DRV8301_VdsLevel_0p068_V =  1 << 6,      //!< Vds = 0.068 V
  DRV8301_VdsLevel_0p076_V =  2 << 6,      //!< Vds = 0.076 V
  DRV8301_VdsLevel_0p086_V =  3 << 6,      //!< Vds = 0.086 V
  DRV8301_VdsLevel_0p097_V =  4 << 6,      //!< Vds = 0.097 V
  DRV8301_VdsLevel_0p109_V =  5 << 6,      //!< Vds = 0.109 V
  DRV8301_VdsLevel_0p123_V =  6 << 6,      //!< Vds = 0.123 V
  DRV8301_VdsLevel_0p138_V =  7 << 6,      //!< Vds = 0.138 V
  DRV8301_VdsLevel_0p155_V =  8 << 6,      //!< Vds = 0.155 V
  DRV8301_VdsLevel_0p175_V =  9 << 6,      //!< Vds = 0.175 V
  DRV8301_VdsLevel_0p197_V = 10 << 6,      //!< Vds = 0.197 V
  DRV8301_VdsLevel_0p222_V = 11 << 6,      //!< Vds = 0.222 V
  DRV8301_VdsLevel_0p250_V = 12 << 6,      //!< Vds = 0.250 V
  DRV8301_VdsLevel_0p282_V = 13 << 6,      //!< Vds = 0.282 V
  DRV8301_VdsLevel_0p317_V = 14 << 6,      //!< Vds = 0.317 V
  DRV8301_VdsLevel_0p358_V = 15 << 6,      //!< Vds = 0.358 V
  DRV8301_VdsLevel_0p403_V = 16 << 6,      //!< Vds = 0.403 V
  DRV8301_VdsLevel_0p454_V = 17 << 6,      //!< Vds = 0.454 V
  DRV8301_VdsLevel_0p511_V = 18 << 6,      //!< Vds = 0.511 V
  DRV8301_VdsLevel_0p576_V = 19 << 6,      //!< Vds = 0.576 V
  DRV8301_VdsLevel_0p648_V = 20 << 6,      //!< Vds = 0.648 V
  DRV8301_VdsLevel_0p730_V = 21 << 6,      //!< Vds = 0.730 V
  DRV8301_VdsLevel_0p822_V = 22 << 6,      //!< Vds = 0.822 V
  DRV8301_VdsLevel_0p926_V = 23 << 6,      //!< Vds = 0.926 V
  DRV8301_VdsLevel_1p043_V = 24 << 6,      //!< Vds = 1.403 V
  DRV8301_VdsLevel_1p175_V = 25 << 6,      //!< Vds = 1.175 V
  DRV8301_VdsLevel_1p324_V = 26 << 6,      //!< Vds = 1.324 V
  DRV8301_VdsLevel_1p491_V = 27 << 6,      //!< Vds = 1.491 V
  DRV8301_VdsLevel_1p679_V = 28 << 6,      //!< Vds = 1.679 V
  DRV8301_VdsLevel_1p892_V = 29 << 6,      //!< Vds = 1.892 V
  DRV8301_VdsLevel_2p131_V = 30 << 6,      //!< Vds = 2.131 V
  DRV8301_VdsLevel_2p400_V = 31 << 6       //!< Vds = 2.400 V
} DRV8301_VdsLevel_e;


typedef enum
{
  DRV8301_GETID=0
} Drv8301SpiOutputDataSelect_e;


typedef struct _DRV_SPI_8301_Stat1_t_
{
  int                  FAULT;
  int                  GVDD_UV;
  int                  PVDD_UV;
  int                  OTSD;
  int                  OTW;
  int                  FETHA_OC;
  int                  FETLA_OC;
  int                  FETHB_OC;
  int                  FETLB_OC;
  int                  FETHC_OC;
  int                  FETLC_OC;
}DRV_SPI_8301_Stat1_t_;


typedef struct _DRV_SPI_8301_Stat2_t_
{
  int                  GVDD_OV;
  unsigned int              DeviceID;
}DRV_SPI_8301_Stat2_t_;


typedef struct _DRV_SPI_8301_CTRL1_t_
{
  DRV8301_PeakCurrent_e    DRV8301_CURRENT;
  unsigned int          DRV8301_RESET;
  unsigned int        PWM_MODE;
}DRV_SPI_8301_CTRL1_t_;


typedef struct _DRV_SPI_8301_CTRL2_t_
{
  DRV8301_OcTwMode_e       OCTW_SET;
  DRV8301_ShuntAmpGain_e   GAIN;
  DRV8301_DcCalMode_e      DC_CAL_CH1p2;
  DRV8301_OcOffTimeMode_e  OC_TOFF;
}DRV_SPI_8301_CTRL2_t_;


typedef struct _DRV_SPI_8301_Vars_t_
{
  DRV_SPI_8301_Stat1_t_     Stat_Reg_1;
  DRV_SPI_8301_Stat2_t_     Stat_Reg_2;
  DRV_SPI_8301_CTRL1_t_     Ctrl_Reg_1;
  DRV_SPI_8301_CTRL2_t_     Ctrl_Reg_2;
  int                  SndCmd;
  int                  RcvCmd;
}DRV_SPI_8301_Vars_t;


//! \brief Defines the DRV8301 object
//!
typedef struct _DRV8301_Obj_
{
                 //!< the gpio number that is connected to the drv8301 enable pin
  int             RxTimeOut;                  //!< the timeout flag for the RX fifo
  int             enableTimeOut;              //!< the timeout flag for drv8301 enable
} DRV8301_Obj;


//! \brief Defines the DRV8301 handle
//!
typedef struct _DRV8301_Obj_ *DRV8301_Handle;


//! \brief Defines the DRV8301 Word type
//!
typedef  unsigned int    DRV8301_Word_t;




//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8301 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \return    The DC calibration mode
extern unsigned int DRV8301_getDcCalMode(const int ampNumber);

//! \brief     Enables the DRV8301
//! \param[in] handle     The DRV8301 handle
extern void DRV8301_enable();


//! \brief     Gets the fault type
//! \param[in] handle     The DRV8301 handle
//! \return    The fault type
extern DRV8301_FaultType_e DRV8301_getFaultType();


//! \brief     Gets the device ID
//! \param[in] handle     The DRV8301 handle
//! \return    The device ID
extern unsigned int DRV8301_getId();


//! \brief     Gets the over current level
//! \param[in] handle     The DRV8301 handle
//! \return    The over current level, V
extern DRV8301_VdsLevel_e DRV8301_getOcLevel();


//! \brief     Gets the over current mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current mode
extern unsigned int DRV8301_getOcMode();


//! \brief     Gets the over current off time mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current off time mode
extern DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode();


//! \brief     Gets the over current, temperature warning mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current, temperature warning mode
extern DRV8301_OcTwMode_e DRV8301_getOcTwMode();


//! \brief     Gets the peak current value
//! \param[in] handle     The DRV8301 handle
//! \return    The peak current value
extern DRV8301_PeakCurrent_e DRV8301_getPeakCurrent();


//! \brief     Gets the PWM mode
//! \param[in] handle     The DRV8301 handle
//! \return    The PWM mode
extern unsigned int DRV8301_getPwmMode();


//! \brief     Gets the shunt amplifier gain value
//! \param[in] handle     The DRV8301 handle
//! \return    The shunt amplifier gain value
extern DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain();


//! \brief     Gets the status register 1 value
//! \param[in] handle     The DRV8301 handle
//! \return    The status register1 value
extern unsigned int DRV8301_getStatusRegister1();


//! \brief     Gets the status register 2 value
//! \param[in] handle     The DRV8301 handle
//! \return    The status register2 value
extern unsigned int DRV8301_getStatusRegister2();



//! \brief     Determines if DRV8301 fault has occurred
//! \param[in] handle     The DRV8301 handle
//! \return    A intean value denoting if a fault has occurred (true) or not (false)
extern int DRV8301_isFault();


//! \brief     Determines if DRV8301 is in reset
//! \param[in] handle     The DRV8301 handle
//! \return    A intean value denoting if the DRV8301 is in reset (true) or not (false)
extern int DRV8301_isReset();


//! \brief     Reads data from the DRV8301 register
//! \param[in] handle   The DRV8301 handle
//! \param[in] regName  The register name
//! \return    The data value
extern unsigned int DRV8301_readSpi(const int regName);


//! \brief     Resets the DRV8301
//! \param[in] handle   The DRV8301 handle
extern void DRV8301_reset();


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8301 handle
static inline void DRV8301_resetEnableTimeout()
{


  //obj->enableTimeOut = false;

  return;
}


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8301 handle
static inline void DRV8301_resetRxTimeout()
{

  //obj->RxTimeOut = false;

  return;
}


//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8301 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
extern void DRV8301_setDcCalMode(
                                 const DRV8301_ShuntAmpNumber_e ampNumber,
                                 const DRV8301_DcCalMode_e mode);



//! \brief     Sets the over current level in terms of Vds
//! \param[in] handle    The DRV8301 handle
//! \param[in] VdsLevel  The over current level, V
extern void DRV8301_setOcLevel(const DRV8301_VdsLevel_e VdsLevel);


//! \brief     Sets the over current mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The over current mode
extern void DRV8301_setOcMode(const unsigned int mode);


//! \brief     Sets the over current off time mode
//! \param[in] handle   The DRV8301 handle
//! \param[in] mode     The over current off time mode
extern void DRV8301_setOcOffTimeMode(const DRV8301_OcOffTimeMode_e mode);


//! \brief     Sets the over current, temperature warning mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The over current, temperature warning mode
extern void DRV8301_setOcTwMode(const DRV8301_OcTwMode_e mode);


//! \brief     Sets the peak current value
//! \param[in] handle       The DRV8301 handle
//! \param[in] peakCurrent  The peak current value
extern void DRV8301_setPeakCurrent(const DRV8301_PeakCurrent_e peakCurrent);


//! \brief     Sets the PWM mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The PWM mode
extern void DRV8301_setPwmMode(const unsigned int mode);


//! \brief     Sets the shunt amplifier gain value
//! \param[in] handle  The DRV8301 handle
//! \param[in] gain    The shunt amplifier gain value
extern void DRV8301_setShuntAmpGain(const DRV8301_ShuntAmpGain_e gain);


//! \brief     Sets the SPI handle in the DRV8301
//! \param[in] handle     The DRV8301 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8301_setSpiHandle();


//! \brief     Writes data to the DRV8301 register
//! \param[in] handle   The DRV8301 handle
//! \param[in] regName  The register name
//! \param[in] data     The data value
extern void DRV8301_writeSpi(const int regName,const unsigned int data);


//! \brief     Interface to all 8301 SPI variables
//!
//! \details   Call this function periodically to be able to read the DRV8301 Status1, Status2,
//!            Control1, and Control2 registers and write the Control1 and Control2 registers.
//!            This function updates the members of the structure DRV_SPI_8301_Vars_t.
//!            <b>How to use in Setup</b>
//!            <b>Code</b>
//!            Add the structure declaration DRV_SPI_8301_Vars_t to your code
//!            Make sure the SPI and 8301 EN_Gate GPIO are setup for the 8301 by using HAL_init and HAL_setParams
//!            During code setup, call HAL_enableDrv and HAL_setupDrvSpi
//!            In background loop, call DRV8301_writeData and DRV8301_readData
//!            <b>How to use in Runtime</b>
//!            <b>Watch window</b>
//!            Add the structure, declared by DRV_SPI_8301_Vars_t above, to the watch window
//!            <b>Runtime</b>
//!            Pull down the menus from the DRV_SPI_8301_Vars_t strcuture to the desired setting
//!            Set SndCmd to send the settings to the DRV8301
//!            If a read of the DRV8301 registers is required, se RcvCmd
//!
//! \param[in] handle  The DRV8301 handle
//! \param[in] Spi_8301_Vars  The (DRV_SPI_8301_Vars_t) structure that contains all DRV8301 Status/Control register options
extern void DRV8301_writeData( );


//! \param[in] handle  The DRV8301 handle
//! \param[in] Spi_8301_Vars  The (DRV_SPI_8301_Vars_t) structure that contains all DRV8301 Status/Control register options
extern void DRV8301_readData( );


//! \brief     Initialize the interface to all 8301 SPI variables
//! \param[in] handle  The DRV8301 handle
extern void DRV8301_setupSpi( );


#endif /* DRV8301_H_ */
