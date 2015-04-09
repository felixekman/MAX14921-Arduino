/*
  MAX14921.h - Library for reading from a MAX14921.
  Created by Felix
*/

#include "max14921.h"
#include "Arduino.h"
#include <SPI.h>

long ticks = 0;

/**-----------------------------------------------------------------------------
 *
 * Default delay timeouts
 *
 *------------------------------------------------------------------------------
 */
#define   MD_AK35_LEVEL_SHIFTING_DELAY_MAX   50   // us
#define   MD_AK35_T_SETTLING_TIME_MAX        10   // us

/**-----------------------------------------------------------------------------
 *
 * SPI bit defines
 *
 *------------------------------------------------------------------------------
 */
#define   MD_AK35_SPI_SAMPLB_ENABLE     0x04
#define   MD_AK35_SPI_SAMPLB_DISABLE    0x00
#define   MD_AK35_SPI_DIAG_ENABLE       0x02
#define   MD_AK35_SPI_DIAG_DISABLE      0x00
#define   MD_AK35_SPI_LOPW_ENABLE       0x01
#define   MD_AK35_SPI_LOPW_DISABLE      0x00
#define   MD_AK35_SPI_TXSEL_DIRECT      0x08
#define   MD_AK35_SPI_TXSEL_BUFFERED    0x18

#define   MD_AK35_SPI_TXSEL_1           0x40
#define   MD_AK35_SPI_TXSEL_2           0x20
#define   MD_AK35_SPI_TXSEL_3           0x60

#define   MD_AK35_SPI_CELLSEL_00   0x80
#define   MD_AK35_SPI_CELLSEL_01   0xC0
#define   MD_AK35_SPI_CELLSEL_02   0xA0
#define   MD_AK35_SPI_CELLSEL_03   0xE0
#define   MD_AK35_SPI_CELLSEL_04   0x90
#define   MD_AK35_SPI_CELLSEL_05   0xD0
#define   MD_AK35_SPI_CELLSEL_06   0xB0
#define   MD_AK35_SPI_CELLSEL_07   0xF0
#define   MD_AK35_SPI_CELLSEL_08   0x88
#define   MD_AK35_SPI_CELLSEL_09   0xC8
#define   MD_AK35_SPI_CELLSEL_10   0xA8
#define   MD_AK35_SPI_CELLSEL_11   0xE8
#define   MD_AK35_SPI_CELLSEL_12   0x98
#define   MD_AK35_SPI_CELLSEL_13   0xD8
#define   MD_AK35_SPI_CELLSEL_14   0xB8
#define   MD_AK35_SPI_CELLSEL_15   0xF8

/**-----------------------------------------------------------------------------
 *
 * Lookup table for quick access to cell select bits needed for SPI command
 *
 *------------------------------------------------------------------------------
 */
uint8_t gMD_AK35_CellSelect_Table[ 16 ] =
{
  MD_AK35_SPI_CELLSEL_00,
  MD_AK35_SPI_CELLSEL_01,
  MD_AK35_SPI_CELLSEL_02,
  MD_AK35_SPI_CELLSEL_03,
  MD_AK35_SPI_CELLSEL_04,
  MD_AK35_SPI_CELLSEL_05,
  MD_AK35_SPI_CELLSEL_06,
  MD_AK35_SPI_CELLSEL_07,
  MD_AK35_SPI_CELLSEL_08,
  MD_AK35_SPI_CELLSEL_09,
  MD_AK35_SPI_CELLSEL_10,
  MD_AK35_SPI_CELLSEL_11,
  MD_AK35_SPI_CELLSEL_12,
  MD_AK35_SPI_CELLSEL_13,
  MD_AK35_SPI_CELLSEL_14,
  MD_AK35_SPI_CELLSEL_15
};

/**-----------------------------------------------------------------------------
 *
 * Current object parameters
 *
 *------------------------------------------------------------------------------
 */
typedef struct {
	uint8_t              tmrId;

  //MF_AFWRK_QUEUENODE_T     thdQNodes[ MD_AK35_THREAD_Q_SIZE ];    /**< Queue nodes  */
  //MF_AFWRK_QUEUE_T         thdQ;                                  /**< Thread queue */
  //MD_AK35_STATE_T          curState;

	uint8_t          acqCellNumber;
  int         acqCellSampleTmrValueMsec;
  int         acqCellRepeatTmrValueMsec;
  int         acqCellSettlingTimeUsec;
  bool        acqScanContinuous;
  bool        acqT1IsEnabled;
  bool        acqT2IsEnabled;
  bool        acqT3IsEnabled;
  int         acqT1SettlingTimeUsec;
  int         acqT2SettlingTimeUsec;
  bool        acqT3SettlingTimeUsec;
  //MI_CLI_REASON_T   acqInterface;

  uint8_t   spiBalanceC01_C08;
  uint8_t   spiBalanceC09_C16;
  uint8_t   spiSamplB;
  uint8_t   spiDiag;
  uint8_t   spiLoPw;

  long   calParErrTmrValueMsec;

  //MD_AK35_USERCFG_T   usrCfg;

} MD_AK35_INSTANCE_T;

/**-----------------------------------------------------------------------------
 *
 * Used to manage the device object.
 *
 *------------------------------------------------------------------------------
 */
MD_AK35_INSTANCE_T   MD_AK35_obj;

int readvalue;

max14921::max14921(int ADC_CS_pin, int CS_pin, int SAMPLPIN_pin)
{
	  pinMode(ADC_CS_pin, OUTPUT);
	  pinMode(CS_pin, OUTPUT);
	  pinMode(SAMPLPIN_pin, OUTPUT);

	  // disable device to start with
	  digitalWrite(ADC_CS_pin, HIGH);
	  digitalWrite(CS_pin, HIGH);
	  digitalWrite(SAMPLPIN_pin, HIGH);

	  SPI.setClockDivider( SPI_CLOCK_DIV8 ); // slow the SPI bus down
	  SPI.setBitOrder(MSBFIRST);
	  SPI.setDataMode(SPI_MODE0);    // SPI 0,0 as per MCP330x data sheet
	  SPI.begin();

	  _ADC_CS_pin = ADC_CS_pin;
	  _CS_pin = CS_pin;
	  _SAMPLPIN_pin = SAMPLPIN_pin;

	  //
	  // Setup initial parameters of the AL38 object
	  //
	  MD_AK35_obj.acqCellNumber             = 16;
	  MD_AK35_obj.acqCellSampleTmrValueMsec = 4;   // ms
	  MD_AK35_obj.acqCellRepeatTmrValueMsec = 6;   // ms
	  MD_AK35_obj.acqCellSettlingTimeUsec   = MD_AK35_T_SETTLING_TIME_MAX;   // us

	  MD_AK35_obj.acqScanContinuous = false;

	  MD_AK35_obj.acqT1IsEnabled        = false;
	  MD_AK35_obj.acqT2IsEnabled        = false;
	  MD_AK35_obj.acqT3IsEnabled        = false;
	  MD_AK35_obj.acqT1SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us
	  MD_AK35_obj.acqT2SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us
	  MD_AK35_obj.acqT3SettlingTimeUsec = MD_AK35_T_SETTLING_TIME_MAX;   // us

	//  MD_AK35_obj.acqInterface    = MI_CLI_REASON_EXECUTE_USBHID;

	  MD_AK35_obj.spiBalanceC01_C08 = 0x00;
	  MD_AK35_obj.spiBalanceC09_C16 = 0x00;
	  MD_AK35_obj.spiSamplB         = MD_AK35_SPI_SAMPLB_DISABLE;
	  MD_AK35_obj.spiDiag           = MD_AK35_SPI_DIAG_DISABLE;
	  MD_AK35_obj.spiLoPw           = MD_AK35_SPI_LOPW_DISABLE;

	  MD_AK35_obj.calParErrTmrValueMsec = 40;
}

// initialize the library with the numbers of the interface pins: (SPI on Arduino UNO as standard)
// set up the number of cells used: Choice between 3 and 16 cells total. This is used to calculate pack voltage
void max14921::SetCellNumber(uint8_t cellNum)
{
	MD_AK35_obj.acqCellNumber = cellNum;
}
// set up the battery chemestry used: LiPo is standard sets parimeters for vMax/vMin per cell.
//LiPo - vMin per cell = 3.600v vMax per cell = 4.200v

// Enable balancing of cells - Option - ON/OFF
void max14921::SetBalancing(bool bal)
{
}

// Enable open-wire detection - Option - ON/OFF
void max14921::SetOpenWireDetection(bool openwire)
{
}

// Set Sample time: Option in ms. Standard 4 ms
void max14921::SetSampleTime(int sampletime)
{
	MD_AK35_obj.acqCellSampleTmrValueMsec = sampletime;
}

// Set Settling Time in us. Standard 50 us.
void max14921::SetSettlingTime(int settlingtime)
{
	MD_AK35_obj.acqCellSettlingTimeUsec = settlingtime;
}

// Set Repeat Time in ms. Standard 10 ms.
void max14921::SetRepeatTime(int repeattime)
{
	MD_AK35_obj.acqCellRepeatTmrValueMsec = repeattime;
}


/**-----------------------------------------------------------------------------
 *
 * Read 2 bytes from device using dummy data
 *
 * @return
 * int : 2 bytes of SPI Rx Data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
int max14921::MAX11163_ReadData16( void )
{
  int adcData = 0;
  int spiData = 0;
  int hi = 0, lo = 0;

  digitalWrite(_ADC_CS_pin, LOW); // Enable device
  //digitalWrite (SELPIN, LOW); // Select adc

  // 100ns delay

  spiData = SPI.transfer(0x00);  // dummy read

  spiData = SPI.transfer(0x00);

  hi = spiData;

  spiData = SPI.transfer(0x00);
  lo = spiData;

  digitalWrite(_ADC_CS_pin, HIGH); // turn off device

  adcData= hi * 256 + lo;
  return( adcData );
}

/**-----------------------------------------------------------------------------
 *
 * Transfers 3 SPI data bytes and returns 3 bytes of read data
 *
 * @param byte1   in : SPI Data Tx Byte 1
 * @param byte2   in : SPI Data Tx Byte 2
 * @param byte3   in : SPI Data Tx Byte 3
 *
 * @return
 * MCS_U32_T : 24 bits of SPI Rx Data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
long max14921::MD_AK35_SpiTransfer24(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
  long ak35Data = 0;
  byte spiData  = 0;

  spiData    = SPI.transfer(byte1);
  ak35Data   = spiData;
  ak35Data <<= 8;

  spiData    = SPI.transfer(byte2);
  ak35Data  |= spiData;
  ak35Data <<= 8;

  spiData    = SPI.transfer(byte3);
  ak35Data  |= spiData;

  return( ak35Data );
}

/**-----------------------------------------------------------------------------
 *
 * Performs readings of cell data including -
 *   - Disable SAMPL with SAMPL settle timing
 *   - Send cell selection SPI command to AK35
 *   - Delay AOUT settle time
 *   - Read ADC with timing constraints
 *
 * @param cellNum            in : Number of cells to read (1-16)
 * @param pAdcCellVoltage   out : Pointer to array of cell ADC readings
 * @param pSpiRxData        out : Pointer to array of SPI Rx data for AK35
 *                                  cell selection requests
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void max14921::MD_AK35_Cmd_AcquireCell(uint8_t cellNum,
                              int *pAdcCellVoltage,
                              long *pSpiRxData )
{
  byte cellIndex = 0;


  //
  // Disable SAMPL and delay for data to be stored in AK35
  //
  digitalWrite (_SAMPLPIN_pin, LOW); // Sampl_Disable
  delayMicroseconds(MD_AK35_LEVEL_SHIFTING_DELAY_MAX);

  if ( cellNum != 0 )
  {
    //
    // Read x cell voltages
    //
    for ( cellIndex = (cellNum-1) ; cellIndex >= 0 ; cellIndex-- )
    {
    	uint8_t spiCmd=0;

      spiCmd  = gMD_AK35_CellSelect_Table[ cellIndex ];
      spiCmd |= ( MD_AK35_obj.spiSamplB | MD_AK35_obj.spiDiag | MD_AK35_obj.spiLoPw );

      //
      // Send command to AK35
      //
      digitalWrite (_SAMPLPIN_pin, LOW); // chip-select MAX14921

      pSpiRxData[ cellIndex ] = MD_AK35_SpiTransfer24( MD_AK35_obj.spiBalanceC01_C08,
                                                       MD_AK35_obj.spiBalanceC09_C16,
                                                       spiCmd );
      digitalWrite (_SAMPLPIN_pin, HIGH); // chip-select MAX14921

      //
      // HW bug fix for 1st silicon
      //   Need to write balance bits twice for C01 to work correctly!
      //
      digitalWrite (_CS_pin, LOW); // chip-select MAX14921
      pSpiRxData[ cellIndex ] = MD_AK35_SpiTransfer24( MD_AK35_obj.spiBalanceC01_C08,
                                                       MD_AK35_obj.spiBalanceC09_C16,
                                                       spiCmd );
      digitalWrite (_CS_pin, HIGH); // chip-select MAX14921


      //
      // Delay required for command to be processed and data valid on AOUT of AK35
      //
      //MD_AK35_Delay_uSec( MD_AK35_obj.acqCellSettlingTimeUsec );
      delayMicroseconds(MD_AK35_obj.acqCellSettlingTimeUsec);

      //
      // Read AOUT Data with ADC
      //
      pAdcCellVoltage[ cellIndex ] = MAX11163_ReadData16();
    }
  }
}

/**-----------------------------------------------------------------------------
 *
 * Scans thru requested cells and reads ADC data and returns data to PC
 *
 * @param calibrationData   in : TRUE  - Data read is Parasitic Error Cal data
 *                               FALSE - Data is normal cell voltage data
 *
 * @constraints
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void max14921::MD_AK35_ScanCell(bool calibrationData)
{
  int adcVal[ 16 ]    = { 0 };
  long spiRxData[ 16 ] = { 0 };

  //
  // If Cell Voltages requested, read and return that data
  //   0xFF = not requested
  //
  if ( MD_AK35_obj.acqCellNumber != 0 )
  {
    MD_AK35_Cmd_AcquireCell( MD_AK35_obj.acqCellNumber,
                             &adcVal[0],
                             &spiRxData[0] );

      //
      // Send Cell voltage data back along UART Pipe
      //
      long index          =   0;
      char      respStr[ 160 ] = { 0 };


      sprintf( respStr, "ak35Resp scan cell %02X %06X %04X",
                        MD_AK35_obj.acqCellNumber,
                        spiRxData[0],
                        adcVal[0] );

      for ( index = 1 ; index < MD_AK35_obj.acqCellNumber ; index++ )
      {
        sprintf( respStr, "%s %04X",
                          respStr,
                          adcVal[ index ] );
        Serial.println(respStr);
      }

      //MI_CLI_WriteTxt( respStr );
  }
}
