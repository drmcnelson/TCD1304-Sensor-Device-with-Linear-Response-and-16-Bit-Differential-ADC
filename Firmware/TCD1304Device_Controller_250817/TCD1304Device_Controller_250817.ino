/*
  Author:   Mitchell C. Nelson
  Date:     May 4, 2025
  Contact:  drmcnelsonlab@gmail.com

     Copyright 2025 Mitchell C. Nelson, PhD

     Patents pending.

  This work is a derivative of the following works:
  
    TCD1304_201223.ino   Mitchell C. Nelson,  Aug 10 through Dec 23, 2020
    LCCDController_Rev2_250106.ino  Mitchell C. Nelson through January 6, 2025

  Permissions:

  Permission is hereby granted for personal, non-commercial use of this code
  with devices published in the authors's github https://github.com/drmcnelson.

  All other rights and permissions reserved to the author.

  Provisos:

  1) Use entirely at your own risk.

  2) No guarantee nor representation is offered nor implied of suitability for any purpose whatsoever.

  3) Please cite appropriately in any work using this code.

 */

// ------------------------------------------------------------------
#define DRMCNELSONLAB
//#define YOURNAMEHERE

#define CONTROLLER_OCPIN 23

// The T4 R2 shares one SPI with two connetors, CS for the 20pin connector is pin 10, small connetor is from pin 9
#define CONTROLLER_CSPIN 10
#define CONTROLLER_CSPIN2 9

// ------------------------------------------------------------------
//#define DIAGNOSTICS_CPU
//#define DIAGNOSTICS_IDLER
//#define DIAGNOSTICS_SYNC
//#define DIAGNOSTICS_GATE
// Stream& dataPort = SerialUSB1;
// ------------------------------------------------------------------

#include "Arduino.h"

//#include "TeensyTimerTool.h"

#include "TCD1304Device.h"

#include "parselib.h"

#include <SPI.h>

// SPI interface to the ADC
SPISettings spi_settings( 30000000, MSBFIRST, SPI_MODE0);   // 30 MHz, reads 1.5usecs/word including CNVST

#include <limits.h>

#include <ADC.h>
#include <ADC_util.h>

#include <EEPROM.h>

extern float tempmonGetTemp(void);

// ADC setup - need this before tcd1304 if using the onboard adc
ADC *adc = new ADC();

// TCD1304 Setup
TCD1304Device tcd1304device = TCD1304Device();


// CPU Cycles per Usec
#define CYCLES_PER_USEC (F_CPU / 1000000)

/* If you have permission for commercial use or an alternative design,
   your company name and product name might go in the following.
*/
#define thisMANUFACTURER_NAME {'D','R','M','C','N','E','L','S','O','N','L','A','B' }
#define thisMANUFACTURER_NAME_LEN 13

#define thisPRODUCT_NAME {'T','C','D','1','3','0','4','S','P','I'}
#define thisPRODUCT_NAME_LEN 10

#define thisPRODUCT_SERIAL_NUMBER { 'S','N','0','0','0','0','0','0','0','0','0','1' }
#define thisPRODUCT_SERIAL_NUMBER_LEN 12

extern "C"
{
  struct usb_string_descriptor_struct_manufacturer
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisMANUFACTURER_NAME_LEN];
  };

  usb_string_descriptor_struct_manufacturer usb_string_manufacturer_name = {
    2 + thisMANUFACTURER_NAME_LEN * 2,
    3,
    thisMANUFACTURER_NAME
  };

  // -------------------------------------------------
  
  struct usb_string_descriptor_struct_product
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisPRODUCT_NAME_LEN];
  };

  usb_string_descriptor_struct_product usb_string_product_name = {
    2 + thisPRODUCT_NAME_LEN * 2,
    3,
    thisPRODUCT_NAME
  };

  // -------------------------------------------------
  
  struct usb_string_descriptor_struct_serial_number
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisPRODUCT_SERIAL_NUMBER_LEN];
  };

  usb_string_descriptor_struct_serial_number usb_string_serial_number =
    {
      2 + thisPRODUCT_SERIAL_NUMBER_LEN * 2, 
      3,
      thisPRODUCT_SERIAL_NUMBER
    };
}

/* ===================================================================================================
   External USB command and response buffers (define them here, we use them for async reporting)
*/
#define RCVLEN 256
char rcvbuffer[RCVLEN];
uint16_t nrcvbuf = 0;

#define SNDLEN 256
char sndbuffer[SNDLEN];

/* ===================================================================================================
   TCD1304 SPI Device
*/

const char versionstr[] = "TCD1304Device vers 0.3 " __DATE__;
const char authorstr[] =  "Patents Pending and (c) 2023, 2025 by Mitchell C. Nelson, Ph.D. ";

const char sensorstr[] = "TCD1304";

const int syncPin      = SYNC_PIN;        // Goes low for trigger standoff (delay) period, use this for desktop N2 laser
const int busyPin      = BUSY_PIN;        // Gate output pin, goes high during shutter
const int interruptPin = INTERRUPT_PIN;   // Trigger input pin
//const int sparePin     = SPARE_PIN;       // Spare pin for digital output

//const int fMPinMonitor = CLK_MONITOR_PIN; // Clock monitor
const int fMPin        = CLK_PIN;         // Mclk out 
const int ICGPin       = ICG_PIN;         // Integration clear gate
const int SHPin        = SH_PIN;          // Shift gate
//#define CLOCKREAD (CORE_PIN3_PINREG & CORE_PIN3_BITMASK)

const int CNVSTPin     = CNVST_PIN;
const int SDIPin       = 11;
const int SDOPin       = 12;
const int CLKPin       = 13;

const int analogPin    = A0;    // Analog input, Pin 14

//unsigned int intedgemode = RISING;  // Default trigger mode

void blink() {
}

void pulsePin( unsigned int pin, unsigned int usecs ) {
  digitalToggleFast(pin);
  delayMicroseconds(usecs);
  digitalToggleFast(pin);
}

/* =====================================================================================================
   Over current pin on the T4 Gen II controller
*/

#ifdef CONTROLLER_OCPIN
bool ocpinstate = false;
bool ocpinattached = false;

void ocpinRead() {
  ocpinstate = digitalRead(CONTROLLER_OCPIN);
  Serial.print( "#OCPIN state ");
  Serial.println( ocpinstate );
  if (!ocpinstate) {
    Serial.println( "#OverCurrent set" );
  }
  else {
    Serial.println( "#OverCurrent clear" );
  }
}

void ocpinISR() {
  ocpinstate = digitalRead(CONTROLLER_OCPIN);
  if (!ocpinstate) {
    Serial.println( "ERROR: OverCurrent set" );
  }
  else {
    Serial.println( "#OverCurrent clear" );
  }
}

void ocpinAttach() {
  if ( !ocpinattached ) {
    attachInterrupt(digitalPinToInterrupt(CONTROLLER_OCPIN), ocpinISR, CHANGE);
    ocpinattached = true;
    Serial.println( "#OCPIN attached");
  }
  else {
    Serial.println( "#OCPIN already attached");
  }
}

void ocpinDetach() {
  if ( ocpinattached ) {
    detachInterrupt(digitalPinToInterrupt(CONTROLLER_OCPIN));
    ocpinattached = false;
    Serial.println( "#OCPIN detached");
  }
  else {
    Serial.println( "#OCPIN already detached");
  }
}
#endif

/* --------------------------------------------------------
   Extension of the interrupt API, re-enable pin interrupt after dropping pending
 */
#define IMR_INDEX   5
#define ISR_INDEX   6

void resumePinInterrupts(uint8_t pin)
{
  if (pin >= CORE_NUM_DIGITAL) return;
  volatile uint32_t *gpio = portOutputRegister(pin);
  uint32_t mask = digitalPinToBitMask(pin);
  gpio[ISR_INDEX] = mask;  // clear pending
  gpio[IMR_INDEX] |= mask;  // re-enable
}

void disablePinInterrupts(uint8_t pin)
{
  if (pin >= CORE_NUM_DIGITAL) return;
  volatile uint32_t *gpio = portOutputRegister(pin);
  uint32_t mask = digitalPinToBitMask(pin);
  gpio[IMR_INDEX] &= ~mask;
}

/* ===================================================================================================
   Timer functions
*/
elapsedMicros elapsed_usecs;

elapsedMicros diagnostic_usecs;
bool diagnostics = false;

uint64_t cycles64( )
{
  static uint32_t oldCycles = ARM_DWT_CYCCNT;
  static uint32_t highDWORD = 0;

  uint32_t newCycles = ARM_DWT_CYCCNT;

  if (newCycles < oldCycles) {
    ++highDWORD;
  }
  oldCycles = newCycles;
  
  return (((uint64_t)highDWORD << 32) | newCycles);
}

/* ======================================================================================================
   CRC codes
*/

unsigned char crctable[256] = {
  0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
  0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
  0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
  0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
  0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
  0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
  0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
  0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
  0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
  0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
  0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
  0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
  0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
  0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
  0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
  0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};

inline uint8_t calculateCRC(uint16_t *bufferp)
{
  uint8_t *pc = ( unsigned char *)&bufferp[DATASTART];
  uint8_t crc = 0xff;
  size_t i;
  for (i = 0; i < NBYTES; i++) {
    crc ^= *pc++;
    crc = crctable[crc];
  }
  return crc;
}

inline uint32_t sumData(uint16_t *bufferp) {
  uint16_t *p16 = &bufferp[DATASTART];
  uint32_t u = 0;
  int n = 0;
  for (n = 0; n < NPIXELS; n++ ) {
    u += *p16++;
  }
  return u;
}

/* ==========================================================================
   Raw data buffer and send functions
*/
#define NBUFFERS 16
uint16_t buffer[NREADOUT] = { 0 };
uint16_t *bufferp = &buffer[0];

/* Data transfer format
 */
#define BINARY 0
#define ASCII 1
unsigned int dataformat = BINARY;
unsigned int counter = 0;

bool data_async = true;
bool crc_enable = false;
bool sum_enable = false;


void sendBuffer_Formatted( ) {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.print( "DATA " );
  Serial.println( NPIXELS );
  for ( int n = 0; n < NPIXELS; n++ ) {
    Serial.println( p16[n] );
  }
  Serial.println( "END DATA" );
}

void sendBuffer_Binary() {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.print( "BINARY16 " );
  Serial.println( NPIXELS );
  Serial.write( (byte *) p16, NBYTES );
  Serial.println( "END DATA" );
}

inline void sendDataCRC() {
  unsigned char crc;
  crc = calculateCRC(bufferp);
  Serial.print( "CRC 0x" );
  Serial.println( crc, HEX );
}

inline void sendDataSum() {
  uint32_t usum = 0;
  usum = sumData(bufferp);
  Serial.print( "SUM " );
  Serial.println( usum );
}

void sendData( ) {

  if (crc_enable) sendDataCRC();
  if (sum_enable) sendDataSum();
  
  if ( dataformat == BINARY ) {
    sendBuffer_Binary( );
  }
  else if ( dataformat == ASCII ) {
    sendBuffer_Formatted( );
  }
}

void sendDataReady() {
  Serial.println( "READY DATA" );
}

void sendTestData() {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.println( "TESTDATA" );
  // load test parttern
  for ( int n = 0; n < NPIXELS; n++ ) {
    p16[n] = n;
  }
  // send it
  sendData();
}

/* ==========================================================================
   For signal averaging
 */
uint32_t accumulator[NREADOUT] = { 0  };
uint accumulator_counter = 0;
bool do_accumulation = false;

void startAccumulator() {
  memset( accumulator, 0, NREADOUT * 4 );
  accumulator_counter = 0;
  do_accumulation = true;
}

void clearAccumulator() {
  memset( accumulator, 0, NREADOUT * 4 );
  accumulator_counter = 0;
}

void stopAccumulator() {
  do_accumulation = false;
}

void updateAccumulator() {
  if (accumulator_counter == 0) {
    for (int n =0; n<NREADOUT; n++) {
      accumulator[n] = bufferp[n];
    }
  }
  else
    for (int n =0; n<NREADOUT; n++) {
      accumulator[n] += bufferp[n];
    }
  accumulator_counter++;
}

void sendAccumulator_Binary( ) {
  uint32_t *p32 = &accumulator[DATASTART];
  Serial.print("ACCUMULATOR "); Serial.println(accumulator_counter);
  Serial.print( "BINARY32 " );
  Serial.println( NPIXELS );
  Serial.write( (byte *) p32, NBYTES32 );
  Serial.println( "END DATA" );
}

void sendAccumulator_Formatted( ) {
  uint32_t *p32 = &accumulator[DATASTART];
  Serial.print("ACCUMULATOR "); Serial.println(accumulator_counter);
  Serial.print( "DATA " );
  Serial.println( NPIXELS );
  for ( int n = 0; n < NPIXELS; n++ ) {
    Serial.println( p32[n] );
  }
  Serial.println( "END DATA" );
}

void sendAccumulator() {
  if ( dataformat == BINARY ) {
    sendAccumulator_Binary( );
  }
  else if ( dataformat == ASCII ) {
    sendAccumulator_Formatted( );
  }
}

/* =====================================================================================
   Built-in ADC readout
*/
inline int fastAnalogRead( uint8_t pin ) {
  adc->adc0->startReadFast(pin);
  while ( adc->adc0->isConverting() );
  return adc->adc0->readSingle();
}

unsigned int measureADCspeed( uint8_t pin, unsigned int knt ) {
  
  uint16_t *p16 = bufferp;
  
  unsigned int usecs;

  Serial.print( "test adc " );
  Serial.println( knt );

  elapsed_usecs = 0;

  for ( unsigned int m = 0; m < knt ; m++ ) {
    for ( int n = 0; n < NREADOUT; n++ ) {
      *p16++ = fastAnalogRead(pin);
    }
    p16 = bufferp;
  }

  usecs = elapsed_usecs;

  Serial.print( "elapsed usecs " );
  Serial.println( usecs );
  
  return usecs;
}

/* --------------------------------------------------------
   ADC single reads
   (note that the alpha0 board used the first two channels)
 */
#define NADC_CHANNELS 4
#define ADC_RESOLUTION (3.3/4096.)
uint16_t adc_data[NADC_CHANNELS] = { 0 };

// Set this to something more than 0 to turn on adc reporting.
unsigned int adc_averages = 0;

/* Send ADC readings
 */
void sendADCs( unsigned int averages ) {

  unsigned int i;
  float scalefactor = ADC_RESOLUTION/averages;
  float val;

  int n = averages;

  for( i = 0 ; i < NADC_CHANNELS; i++ ) {
    adc_data[i] = fastAnalogRead( i );
  }
  n--;
  
  while( n-- ) {
    for( i = 0 ; i < NADC_CHANNELS; i++ ) {
      adc_data[i] += fastAnalogRead( i );
    }
  }
  
  Serial.print( "ADCAVGS " );
  Serial.println( averages );
  
  Serial.print( "ADCDATA" );
  for( i = 0 ; i < NADC_CHANNELS; i++ ) {
    val = adc_data[i] * scalefactor;
    Serial.print( " " );
    Serial.print( val, 5 );
  }
  
  Serial.println( "" );
}

/* --------------------------------------------------------
   Send chip temperature
 */

// Set this to something more than 0 to turn on chip temperature reporting
unsigned int chipTemp_averages = 0;

void sendChipTemperature( unsigned int averages ){
  Serial.print( "CHIPTEMPERATURE " );
  Serial.println( tempmonGetTemp() );
}

/* ===========================================================================================
 * EEPROM support for saving identifier and coordinate mapping constants
 */
#define EEPROM_SIZE 1080

#define EEPROM_ID_ADDR 0
#define EEPROM_ID_LEN 64

#define EEPROM_COEFF_ADDR 64
#define EEPROM_NCOEFFS 4
#define EEPROM_COEFF_LEN ( EEPROM_NCOEFFS * sizeof( float) )

#define EEPROM_UNITS_ADDR (EEPROM_COEFF_ADDR + EEPROM_COEFF_LEN)
#define EEPROM_NUNITS 8
#define EEPROM_UNITS_LEN EEPROM_NUNITS

void eeread( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0  && address < EEPROM_SIZE ) {
    *p++ = EEPROM.read(address++);
  }
}

void eewrite( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE ) {
    EEPROM.write(address++, *p++);
  }
}

// -------------------------------------------------------------

void eereadUntil( unsigned int address, int nbytes, char *p ) {
  char b = 0xFF;
  while( nbytes-- > 0  && address < EEPROM_SIZE && b ) {
    b = EEPROM.read(address++);
    *p++ = b;
  }
}

void eewriteUntil( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE-1 && *p ) {
    EEPROM.write(address++, *p++);
  }
  EEPROM.write(address++, 0);
}

void eeErase( unsigned int address, int nbytes ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE-1 ) {
    EEPROM.write(address++, 0xFF );
  }
}

/* -------------------------------------------------------------
   Store/read Identifier
*/
void eraseIdentifier( ) {
  eeErase( EEPROM_ID_ADDR, EEPROM_ID_LEN );
}

void readIdentifier( char *p ) {
  eereadUntil( EEPROM_ID_ADDR, EEPROM_ID_LEN, p );
}

void storeIdentifier( char *p ) {
  eewriteUntil( EEPROM_ID_ADDR, EEPROM_ID_LEN, p );
}

void printIdentifier( ) {
  readIdentifier( sndbuffer );
  if (sndbuffer[0] != 0xff ) {
    Serial.print( "Identifier: " );
    Serial.println( sndbuffer );
  }
  else {
    Serial.println( "Warning:  identifier is not set" );
  }
}

/* -------------------------------------------------------------
   Store/read Units
*/
void eraseUnits( ) {
  eeErase( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN );
}

void readUnits( char *p ) {
  eereadUntil( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p );
}

void storeUnits( char *p ) {
  eewriteUntil( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p );
}

void printUnits( ) {
  readUnits( sndbuffer );
  if (sndbuffer[0] != 0xff ) {
    Serial.print( "units: " );
    Serial.println( sndbuffer );
  }
  else {
    Serial.println( "Warning:  units is not set" );
  }
}

/* -------------------------------------------------------------
   Store/read Wavelength Coeffs
*/

void eraseCoefficients( ) {
  eeErase( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN );
}

void readCoefficients( float *vals ) {
  eeread( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals );
}

void storeCoefficients( float *vals ) {
  eewrite( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals );
}

void printCoefficients( ) {

  float vals[EEPROM_NCOEFFS];

  readCoefficients( vals );
  
  Serial.print( "coefficients" );
  
  for( int n = 0; n < EEPROM_NCOEFFS; n++ ) {
    sprintf( sndbuffer, " %.8g", vals[n] );
    Serial.print( sndbuffer );
  }
  Serial.println( "" );
}

// ========================================================
// Timer-clocked flexpwm framesets (i.e. the short time-scale framesets)

void start_flexpwm_isr()
{
  tcd1304device.start_flexpwm();
}

IntervalTimer clocked_framesets_timer;
bool clocked_framesets_ready = false;
float clocked_framesets_usecs = 0;

void clocked_framesets_stop_callback()
{
  clocked_framesets_timer.end();
  tcd1304device.clear_framesets_completed_callback();
}

void clocked_framesets_setup(float secs, unsigned int nsets=0)
{
  // stop and disconnect the framesets complete callback when done
  tcd1304device.load_framesets_completed_callback(clocked_framesets_stop_callback,nsets);

  // setup the frameset clock interval in usecs
  clocked_framesets_usecs = secs * 1.E6;

  clocked_framesets_ready = true;
}

// We need this for triggered clocked framesets
void clocked_framesets_start_isr()
{
  clocked_framesets_timer.begin(start_flexpwm_isr, clocked_framesets_usecs);
  start_flexpwm_isr();
}


void clocked_framesets_start()
{
  if (!clocked_framesets_ready) {
    Serial.println("Error: clocked framesets start, but not setup");
  }
  else {
    tcd1304device.frame_counter = 0;
    tcd1304device.frameset_counter = 0;
    
    clocked_framesets_start_isr();
  }
}

// --------------------------------------------------------
// Timer-clocked frames

IntervalTimer clocked_frames_timer;
float clocked_frames_usecs = 0;
bool clocked_frames_ready = false;

void clocked_frames_stop_callback()
{
  clocked_frames_timer.end();
  tcd1304device.clear_frames_completed_callback();
}

void clocked_frames_setup(float secs, unsigned int nframes=0)
{
  // stop and disconnect the frames complete callback when done
  tcd1304device.load_frames_completed_callback(clocked_frames_stop_callback,nframes);

  // setup the time in usecs
  clocked_frames_usecs = secs * 1.E6;

  clocked_frames_ready = true;
}

// We need this for triggered or clocked starts
void clocked_frames_start_isr()
{
  // start the timer
  clocked_frames_timer.begin(start_flexpwm_isr,clocked_frames_usecs);
  start_flexpwm_isr();
}

void clocked_frames_start()
{
  if (!clocked_frames_ready) {
    Serial.println("Error clocked frames start, but not setup");
  }
  else {
    clocked_frames_start_isr();
  }
}

/* ======================================================================================================
   CCD sensor read callback
   ====================================================================================================== */

void send_header(const char *modestring)
{
  Serial.print("START ");
  Serial.print(modestring);

  if (tcd1304device.mode == FRAMESET) {
    Serial.println(" FRAMESET");
  }
  else if (tcd1304device.mode == SINGLE) {
    Serial.println(" SINGLE");
  }
  else {
    Serial.println("");
  }
  
  Serial.print( "FRAME COUNTS " );
  Serial.println( tcd1304device.frame_counts );

  Serial.print( "FRAMESET COUNTS " );
  Serial.println( tcd1304device.frameset_counts );

  if (!strncmp(modestring,"CLOCKED",7)) {
    Serial.print( "CLOCK PERIOD ");
    Serial.println(clocked_framesets_usecs);
  }
  
  if (tcd1304device.mode == FRAMESET) {
    Serial.print( "ICG PERIOD " );
    Serial.println( (int)(tcd1304device.icg.period_secs*1.E6) );
    
    Serial.print( "SH PERIOD " );
    Serial.println( (int)(tcd1304device.sh.period_secs*1.E6) );
  }

  if (tcd1304device.timer_trigger_mode == INTERVALTIMER) {
    Serial.print( "TIMER PERIOD " );
    Serial.println( (int)(tcd1304device.interval_timer_period_secs*1.E6) );
  }
  
}

void send_frame_header()
{
  if (tcd1304device.frame_counter<=1) {
    Serial.println("FRAMESET START ");
  }

  Serial.print( "FRAMESET COUNTER " );
  Serial.println( tcd1304device.frameset_counter );

  Serial.print( "FRAME COUNTER " );
  Serial.println( tcd1304device.frame_counter );

  Serial.print( "FRAME ELAPSED " );
  Serial.println( (int)(tcd1304device.frame_elapsed_secs()*1.E6) );

  Serial.print( "ICG ELAPSED " );
  Serial.println( (int)(tcd1304device.icg_elapsed_secs()*1.E6) );
    
  if (tcd1304device.mode == FRAMESET) {
    Serial.print( "ICG PERIOD " );
    Serial.println( (int)(tcd1304device.icg.period_secs*1.E6) );
    
    Serial.print( "SH PERIOD " );
    Serial.println( (int)(tcd1304device.sh.period_secs*1.E6) );
  }

  if (tcd1304device.timer_trigger_mode == INTERVALTIMER) {
    Serial.print( "TIMER ELAPSED " );
    Serial.println( (int)(tcd1304device.interval_timer_elapsed_secs()*1.E6) );
    Serial.print( "TIMER PERIOD " );
    Serial.println( (int)(tcd1304device.interval_timer_period_secs*1.E6) );
  }
    
  else if (tcd1304device.timer_trigger_mode == TRIGGER) {
    Serial.print( "TRIGGER ELAPSED " );
    Serial.println( (int)(tcd1304device.interrupt_elapsed_secs()*1.E6) );
  }

}

void send_callback()
{

#ifdef DEBUG
  unsigned int cyccnt0 = ARM_DWT_CYCCNT;
  unsigned int cyccnt1;
#endif
  
  if (!do_accumulation) {

    send_frame_header();

    sendData( );

    if (tcd1304device.frame_counter == tcd1304device.frame_counts) {
      Serial.println("FRAMESET END");
    }
    
#ifdef DEBUG
    cyccnt1 = ARM_DWT_CYCCNT - cyccnt0;
    Serial.print("time after send data ");
    Serial.println((float)cyccnt1*(1000000./F_CPU));
#endif

  }
  else {
    updateAccumulator();

#ifdef DEBUG
    cyccnt1 = ARM_DWT_CYCCNT - cyccnt0;
    Serial.print("time after updateAccumulator ");
    Serial.println((float)cyccnt1*(1000000./F_CPU));
#endif
    
    if (tcd1304device.frame_counter == tcd1304device.frame_counts) {

      send_frame_header();

      sendAccumulator( );

      Serial.println("FRAMESET END");
    }
    else {
      Serial.println("ACCUMULATING ");

      send_frame_header();

      sendAccumulator( );
    }
    
#ifdef DEBUG
    cyccnt1 = ARM_DWT_CYCCNT - cyccnt0;
    Serial.print("time for after send accumulator ");
    Serial.println((float)cyccnt1*(1000000./F_CPU));
#endif

  }  
}

/* ==========================================================================
   Hold off
*/
IntervalTimer holdoff_delay_Timer;
void (*holdoff_function_ptr)() = 0;
unsigned int holdoff_usecs = 0;
bool do_sync_holdoff = false;


void holdoff_delay_isr_() {
  holdoff_delay_Timer.end();
  if (holdoff_function_ptr) {
    (*holdoff_function_ptr)();
  }
}

inline void holdoff_isr( ) {
  if (do_sync_holdoff) {
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
    delayMicroseconds(1);
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
  }
  if ( holdoff_usecs < 10000 ) {
   delayMicroseconds(holdoff_usecs);
   (*holdoff_function_ptr)();
  }
  else {
    holdoff_delay_Timer.begin(holdoff_delay_isr_,holdoff_usecs);
  }
}

void set_holdoff_function( void (*funct)() ) {
  holdoff_function_ptr = funct;
}

void cancel_holdoff( ) {
  holdoff_delay_Timer.end();
}

/* ==========================================================================
   Parse for the trigger edge specification (rising,falling,change)
 */

/*
bool parseEdgeMode( char *pc ) {
  if (!strcmp(pc,"rising")) {
    intedgemode = RISING;
    return true;
  }
  if (!strcmp(pc,"falling")) {
    intedgemode = FALLING;
    return true;
  }
  if (!strcmp(pc,"change")) {
    intedgemode = CHANGE;
    return true;
  }
  return false;
}
*/

char *parsePin(char *pc, uint *pin)
{
  char *pc1 = pc;
  uint8_t upin = 0;


  Serial.print("parsePin ");
  Serial.println(pc);
  
  if ((pc1=startsWith(pc,"pin"))) {
    pc = pc1;

    if ((pc1=parseUint8(pc,&upin))) {
      *pin = upin;
      Serial.print(" recognized pin ");
      Serial.println(upin);
      return pc1;
    }    
    
  }

  if ((pc1=startsWith(pc,"sync"))) {
    *pin = syncPin;
    Serial.println(" recognized sync ");
    return pc1;
  }

  if ((pc1=startsWith(pc,"busy"))) {
    *pin = busyPin;
    Serial.println(" recognized busy ");
    return pc1;
  }

  if ((pc1=startsWith(pc,"interrupt"))) {
    *pin = interruptPin;
    Serial.println(" recognized interrupt ");
    return pc1;
  }

  return 0;
}

/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength(), these return a pointer to the
   next character in the string, or null if they fail

   code used to be here, now in parselib.c, .h
*/
  
/* ===================================================================
   Help text
 */
void help() {

  Serial.println("#Report device, version and configuration");
  Serial.println("#  version           - report software version");
  Serial.println("#  configuration     - report device configuration and data structure");
  Serial.println("#  pins              - report digital i/o functions and pin numbers");
  Serial.println("#  dump              - report all of the available settings and states");
  Serial.println("#");
  Serial.println("#Coefficients for pixel number to wavelength");
  Serial.println("#  store coefficients <a0> <a1> <a2> <a3> (need 4 numbers)");
  Serial.println("#  coefficients       - report");
  Serial.println("#");
  Serial.println("#  store units <string> (upto 6 characters, c.f. nm, um, mm)");
  Serial.println("#  units              - report");
  Serial.println("#");
  Serial.println("#Identifier string (63 bytes)");
  Serial.println("#  store identifier <identifier>");
  Serial.println("#  identifier         - list identifier string");
  Serial.println("#");
  Serial.println("#Data format");
  Serial.println("#  set ascii          - set data format to ascii");
  Serial.println("#  set binary         - set data format to binary");
  Serial.println("#  format             - report data form");
  Serial.println("#");
  Serial.println("#  set async          - data is sent asynchronously");
  Serial.println("#  set synchronous    - READY DATA is sent, host responds send data");
  Serial.println("#");
  Serial.println("#  enable|disable crc - precede each data transfer with crc");
  Serial.println("#  enable|disable sum - precede each data transfer with sum");
  Serial.println("#");
  Serial.println("#  crc | send crc     - send 8 bit crc calculated on the binary data");
  Serial.println("#  sum | send sum     - send 32 bit sum calculated on the binary data");
  Serial.println("#");
  Serial.println("#  send test          - send test data");
  Serial.println("#  send               - (re)send last data");
  Serial.println("#");
  Serial.println("#Microcontroller functions");
  Serial.println("#  temperature        - report microcontroller temperature");
  Serial.println("#  reboot             - reboots the entire board");
  Serial.println("#");
  Serial.println("#Read and average analog inputs");
  Serial.println("#  adcs <navgs>        - read analog inputs and report");
  Serial.println("#  set adcs <navgs>    - read ADCs at frame completion");
  Serial.println("#  set adcs off");
  Serial.println("#");
  Serial.println("#LCCD commands:");
  Serial.println("#  Optional configuration commands (before setup):");
  Serial.println("#     set sync on | off - enable or disable sync pin output");
  Serial.println("#");
  Serial.println("#  Low level TCD1304 setup (singles or framesets with exposure to 10msec)");
  Serial.println("#    setup tcd1304 frameset <clkbase (64|128)> <exposure> <pulsewidth> <frame interval> <nframes> [add] (in secs)");
  Serial.println("#    setup tcd1304 single <pulsewidth> <nframes> [add] (in secs)");
  Serial.println("#    start tcd1304");
  Serial.println("#    stop tcd1304");
  Serial.println("#");
  Serial.println("#  Low level timer - Clocks the TCD1304 start function (see setup tcd1304)");
  Serial.println("#    setup timer <period> [n] - defaults to number required by the tcd1304 setup");
  Serial.println("#    start timer");
  Serial.println("#    stop timer");
  Serial.println("#");
  Serial.println("#  System timer    - Clocks the tcd1304 start function (see setup tcd1304)");
  Serial.println("#    setup clock <period> [n] - defaults to number required by the tcd1304 setup");
  Serial.println("#    start clock");
  Serial.println("#    stop clock");
  Serial.println("#");
  Serial.println("#  Digital pin interrupt - Triggers the start function (see setup tcd1304)");
  Serial.println("#    setup trigger [ncounts]  - defaults to number required by the tcd1304 setup");
  Serial.println("#    start trigger");
  Serial.println("#    stop trigger");
  Serial.println("#");
  Serial.println("#    set trigger <pin <n>|rising|falling|change|pullup|nopullup> - any combination");
  Serial.println("#");
  Serial.println("#Flexpwm commands");
  Serial.println("#");
  Serial.println("#   flexpwm close - restore the pins to digital i/o, setup restores them to flexpwm");
  Serial.println("#   flexpwm dump  - dump all of the flexpwm registers");
  Serial.println("#   flexpwm stop  - stop all of the flexpwm submodules");
  Serial.println("#");
  Serial.println("#   set extra cnvst delay <n> - insert additional clock periods before reading the voltage");
  Serial.println("#");
  Serial.println("#   Submodule commands 0-3 or mask 0x1 to 0xF (1 bit each submodule)");
  Serial.println("#");
  Serial.println("#     flexpwm <submodule|mask> clear load ok  (cldok in mcu reference manual)");
  Serial.println("#     flexpwm <submodule|mask> set load ok    (ldok in mcu ref manual)");
  Serial.println("#");
  Serial.println("#     flexpwm <submodule|mask> clear run    - stops the clock");
  Serial.println("#     flexpwm <submodule|mask> set run      - starts the clock");
  Serial.println("#");
  Serial.println("#     flexpwm <submodule> set clock master  - clock runs independently");
  Serial.println("#     flexpwm <submodule> set clock slave   - clock syncs and starts with 0");
  Serial.println("#     flexpwm <submodule> set clock sync    - clock syncs with 0, starts independently");
  Serial.println("#");
  Serial.println("#     flexpwm <submodule> force  -  force counter to reload to init");
  Serial.println("#");
  Serial.println("#     flexpwm <submodule> set prescale <value>  - prescale, 0 to 7 for divider 1<<p");
  Serial.println("#     flexpwm <submodule> set init <value>      - set counter init register to value (16 bits)");
  Serial.println("#     flexpwm <submodule> set val<n> <value>    - set comparison register 0 to 5, to value");
  Serial.println("#");
  Serial.println("#Pin controls - <pin> = sync | busy trigger | pin n | n");
  Serial.println("#    set <pin> hi|low|input|pullup|output");
  Serial.println("#    pulse <pin>  [usecs]");
  Serial.println("#    toggle <pin>");
  Serial.println("#");  
  Serial.println("#Preconfigured pins");
  Serial.print("#  Trigger(input)" );
  Serial.print( interruptPin );
  Serial.print("  Busy " );
  Serial.print( busyPin );
  Serial.print("  Sync " );
  Serial.print( syncPin );
#ifdef CONTROLLER_OCPIN
  Serial.print("  ~OverCurrent " );
  Serial.print( CONTROLLER_OCPIN );
#endif
  Serial.println( "" );
}


/* ===================================================================
   The setup routine runs once when you press reset:
*/

void setup() {

  // Has overcurrent pint
#ifdef CONTROLLER_OCPIN
  pinMode(CONTROLLER_OCPIN, INPUT);
#endif

  tcd1304device.setup_digital_pins();

  // SPI setup
  SPI.begin();
  SPI.beginTransaction(spi_settings);

  // ------------------------------
  pinMode(analogPin, INPUT);
  //  pinMode(analogPin, INPUT_PULLUP);
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
  adc->adc0->setAveraging(1);                 // set number of averages
  adc->adc0->setResolution(12);               // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  adc->adc0->wait_for_cal();
  
  adc->adc0->singleMode();              // need this for the fast read

  // ------------------------------
  Serial.begin(9600);
  delay(100);

  // Patent pending and copyright notice displayed at startup
  Serial.println( versionstr );
  Serial.println( authorstr );
  //Serial.println( salesupportstr );

#ifdef DIAGNOSTICS_CPU 
  Serial.print("F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz."); 
  //Serial.print("F_BUS: "); Serial.print(F_BUS/1e6);  Serial.println(" MHz."); 
  Serial.print("ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
#endif

}

// the loop routine runs over and over again forever:
void loop() {

  uint16_t nlen = 0;
  char *pc, *pc1;
  char c;
  
  unsigned int utemp = 0;
  unsigned int utemp2 = 0;

  /* ---------------------------------------------------
     Read serial input until endof line or ctl character
   */
  while ( Serial.available() ) {

    c = Serial.read();

    if ( c ) {

      // break at ctl-character or semi-colon
      if ( iscntrl( c ) || c == ';' ) {
        nlen = nrcvbuf;
        rcvbuffer[nrcvbuf] = 0;
        nrcvbuf = 0;
        break;
      }

      // skip leading spaces
      else if ( nrcvbuf || !isspace(c) ) {
        rcvbuffer[nrcvbuf++] = c;        
      }

      if ( nrcvbuf >= RCVLEN ) {
        Serial.println( (char *)"Error: buffer overflow" );
        nrcvbuf = 0;
      }
    }
    
#ifdef DIAGNOSTICS_RCV
    Serial.print( '#' );
    Serial.println( rcvbuffer );
#endif
  }
  
  /* ====================================================================
   * Command processing
   */

  if ( nlen > 0 ) {
    
    //blink();

    for ( int n = 0; (n < RCVLEN) && rcvbuffer[n] ; n++ ) {
      rcvbuffer[n] = tolower( rcvbuffer[n] );
    }
    pc = rcvbuffer;

    Serial.println( pc );
     
    /* -------------------------------------------------------------
       Firmware version identification
    */
    if ( (pc = startsWith( rcvbuffer, "version" )) ) {
      Serial.println( versionstr );
      Serial.println( authorstr );
      //Serial.println( salesupportstr );
    }

    else if ( startsWith( rcvbuffer, "reboot") ) {
      _reboot_Teensyduino_();
    }

    else if ( (pc = startsWith( rcvbuffer, "set diagnostics" )) ) {
      diagnostics = true;
      Serial.println( "tcd1304 set diagnostics to true");
    }

    else if ( (pc = startsWith( rcvbuffer, "clear diagnostics" )) ) {
      diagnostics = false;
      Serial.println( "tcd1304 set diagnostics to false");
    }

    else if ( (pc = startsWith( rcvbuffer, "help" )) ) {
      help();
    }
    
    /* ====================================================================
       Clock commands
     */

    else if ((pc=startsWith(rcvbuffer,"setup clock"))) {
      
      float secs = 0;
      unsigned int ntemp = 1;

      // must specify time interval
      if (!(pc1=parseFlt(pc,&secs))) {
        Serial.print("Error: need clock secs [nsets]; ");
        Serial.println(pc);
      }

      // we are clocking flexpwm framesets
      else  if (tcd1304device.mode == FRAMESET) {

        ntemp = 1;
        parseUint(pc,&ntemp);

        serialPrintlnf("tcd1304: clocked framesets start %.4gsecs, nsets %d",secs,ntemp);
        clocked_framesets_setup(secs,ntemp);
      }

      // we are clocking single singles
      else if (tcd1304device.mode == SINGLE) {

        ntemp = 0;
        parseUint(pc,&ntemp);

        serialPrintlnf("tcd1304: clocked frames start %.4gsecs, nframes %d",secs,ntemp);
        clocked_frames_setup(secs,ntemp);
      }
        
      else {
        Serial.println("Error: clock command, but not setup yet");
      }

    }
    
    else if (((pc=startsWith(rcvbuffer, "start clock")))) {

      if (tcd1304device.mode == FRAMESET) {
	send_header("CLOCKED");

	clocked_framesets_start();
      }

      else if (tcd1304device.mode == SINGLE) {
	send_header("CLOCKED");

        clocked_frames_start();
      }
        
    }

    else if ((pc=startsWith(rcvbuffer, "stop clock"))) {
      
      clocked_framesets_timer.end();
      clocked_frames_timer.end();
        
      Serial.println("tcd1304: stopped clock");
    }

    /* ========================================================
     */
    else if ((pc=startsWith(rcvbuffer, "start trigger"))) {
      send_header("TRIGGERED");
      tcd1304device.start_interrupts();
    }
    
    else if ((pc=startsWith(rcvbuffer, "stop trigger"))) {
      tcd1304device.stop_interrupts();
    }
    
    else if ((pc=startsWith(rcvbuffer, "set trigger"))) {

      pc1 = pc;
      while( pc1 && *pc1 && (pc1=nextWord(pc1))) {

	if ((pc=startsWith(pc1,"pin"))) {
	  uint8_t pin = tcd1304device.interrupt_pin;
	  if (parseUint8(pc,&pin)) {
	    tcd1304device.interrupt_pin = pin;
	    pinMode(tcd1304device.interrupt_pin, INPUT);
	    tcd1304device.interrupt_pin_mode = INPUT;
	  }
	}

	else if ((pc=startsWith(pc1, "nopullup"))) {
	  pinMode(tcd1304device.interrupt_pin,INPUT);
	  tcd1304device.interrupt_pin_mode = INPUT;
	}

	else if ((pc=startsWith(pc1, "pullup"))) {
	  pinMode(tcd1304device.interrupt_pin,INPUT_PULLUP);
	  tcd1304device.interrupt_pin_mode = INPUT_PULLUP;
	}
    
	else if ((pc=startsWith(pc1, "rising"))) {
	  tcd1304device.interrupt_edge_mode = RISING;
	}

	else if ((pc=startsWith(pc1, "falling"))) {
	  tcd1304device.interrupt_edge_mode = FALLING;
	}
    
	else if ((pc=startsWith(pc1, "change"))) {
	  tcd1304device.interrupt_edge_mode = CHANGE;
	}
      }
    }

    else if ((pc=startsWith(rcvbuffer, "setup trigger"))) {

      unsigned int ncounts = 0;
      uint8_t pin = tcd1304device.interrupt_pin;
      uint8_t mode = tcd1304device.interrupt_edge_mode;
	
      parseUint(pc,&ncounts);

      serialPrintlnf("setup interrrupts pin %d mode %d counts %d", pin, mode, ncounts);

      tcd1304device.setup_interrupts(ncounts);	

    }
    
    /* ========================================================
     */
    else if ((pc=startsWith(rcvbuffer, "setup timer"))) {

      float period_secs;
      unsigned int nframes = 0;

      if ((pc=nextWord(pc)) &&
          (pc=parseFlt(pc,&period_secs))) {

        parseUint(pc,&nframes);
        
	serialPrintlnf("setup interval period %.4g frames %d", period_secs, nframes);

	tcd1304device.setup_interval_timer(period_secs,nframes,true);	
      }
      else {
        Serial.println("Error: need: setup timer <period> <offset> <n times>");        
      }

    }
    
    else if ((pc=startsWith(rcvbuffer, "start timer"))) {

      if (!tcd1304device.interval_timer_counts) {
        Serial.println("Error: TCD1304 device not configure, try timer");
      }
      else {
	send_header("TIMER");
        tcd1304device.interval_timer_start();
      }
    }
    
    else if ((pc=startsWith(rcvbuffer, "stop timer"))) {

      tcd1304device.interval_timer_stop();

      Serial.println("tcd1304: stopped timer");
    }

    /* ========================================================
     */
    else if ((pc1=startsWith(rcvbuffer, "clear accumulator"))) {
      clearAccumulator();
      Serial.println("tcd1304: cleared the accumulator");
    }

    else if ((pc1=startsWith(rcvbuffer, "set extra cnvst delay"))) {
      parseUint16(pc1,&tcd1304device.cnvst_extra_delay_counts);
      Serial.print("flexpwm extra cnvst delay counts ");
      Serial.println(tcd1304device.cnvst_extra_delay_counts);
    }
    
    else if ((pc1=startsWith(rcvbuffer, "setup tcd1304")) ||
             (pc1=startsWith(rcvbuffer, "setup"))) {
      
      unsigned int clk_base = 0;  // the library defaults to 64
      float exposure_secs;
      float frame_secs;
      float pulse_width_secs;
      unsigned int nframes;

      if ((pc=startsWith(pc1,"single"))) {

        if ((pc=nextWord(pc)) &&
            (pc=parseUint(pc,&clk_base)) &&
            (pc=parseFlt(pc,&pulse_width_secs)) &&
            (pc=parseUint(pc,&nframes))) {

          if ((pc = nextWord(pc)) && !strncmp(pc,"add",3)) {
            startAccumulator();
          }
          else {
            stopAccumulator();
          }

          serialPrintlnf("pulse %.6g frames %d", pulse_width_secs, nframes);

          if (tcd1304device.setup_flexpwm_singles(clk_base, pulse_width_secs, nframes, bufferp, NREADOUT, send_callback)) {
            Serial.println("FLEXPWM SHUTTER SETUP");
          }
          else {
            Serial.println("Error: setup flexpwm shutter");
          }
        }
        else {
          Serial.println("Error: need: setup tcd1304 single <clk_base> <pulse width> <n times> [add]");
        }
      }
      
      else if ((pc=startsWith(pc1,"frameset"))) {
	
        if ((pc=nextWord(pc)) &&
            (pc=parseUint(pc,&clk_base)) &&
            (pc=parseFlt(pc,&exposure_secs)) &&
            (pc=parseFlt(pc,&pulse_width_secs)) &&
            (pc=parseFlt(pc,&frame_secs)) &&
            (pc=parseUint(pc,&nframes))) {

          if ((pc = nextWord(pc)) && !strncmp(pc,"add",3)) {
            startAccumulator();
	    clk_base = 128;  // need extra time for the transfer and addition loop
          }
          else {
            stopAccumulator();
          }

          serialPrintlnf("exposure %.6g pulse %.6g frames %d", exposure_secs, pulse_width_secs, nframes);
	
          if(tcd1304device.setup_flexpwm(clk_base, exposure_secs, pulse_width_secs, frame_secs, nframes,
                                         bufferp, NREADOUT, send_callback)) {
            
            Serial.println("FLEXPWM FRAMESET SETUP");
          }
          else {
            Serial.println("Error: setup flexpwm frameset");
          }
        }
        else {
          Serial.println("Error: need: setup tcd1304 frameset <clk_base> <exposure> <pulse width> <frame interval> <n times> [add]");
        }  
      }
      else {
          Serial.println("Error: need: setup tcd1304 frameset|single ....");
      }

    }
    
    else if ((pc=startsWith(rcvbuffer, "start tcd1304"))) {
      if (tcd1304device.mode == NOTCONFIGURED) {
        Serial.println("Error: TCD1304 device not configure, try setup flexpwm or setup single");
      }
      else {
	send_header("FLEXPWM");
        tcd1304device.start_flexpwm();
      }
    }

    else if ((pc=startsWith(rcvbuffer, "stop tcd1394"))) {
      
      tcd1304device.stop_flexpwm();

      Serial.println("tcd1304: stopped flexpwm");
    }

    else if ((pc=startsWith(rcvbuffer, "stop"))) {
      
      tcd1304device.interval_timer_stop();
      tcd1304device.stop_flexpwm();
      clocked_framesets_timer.end();
      clocked_frames_timer.end();
    }
    
    /* ========================================================
     */
    else if ((pc=startsWith(rcvbuffer, "close"))) {

      tcd1304device.close();
      Serial.println("tcd1304 device closed, pins ready for digital i/o");
    }
    
    else if ((pc=startsWith(rcvbuffer, "set sync enable"))||
             (pc=startsWith(rcvbuffer, "set sync on"))) {
      tcd1304device.sync_enabled = true;;
      Serial.println("tcd1304 sync enabled");
    }

    else if ((pc=startsWith(rcvbuffer, "set sync not enable"))||
             (pc=startsWith(rcvbuffer, "set sync off"))) {
      tcd1304device.sync_enabled = false;;
      Serial.println("tcd1304 sync disenabled");
    }

    // --------------------------------------------------------------

    // for manual debugging work
    else if ((pc=startsWith(rcvbuffer, "flexpwm"))) {
      uint8_t submod = 0;
      uint8_t mask = 0;
      uint8_t u8;
      uint16_t val = 0;

      while (*pc && isspace(*pc)) pc++;
      if (!pc) {
        Serial.println("Error flexpwm needs a command or submodule number of hex mask 0x");        
      }
      else if(startsWith(pc,"dump")) {
        Serial.println("#======================================");
        Serial.println("#TCD1304 flexPWM");
        tcd1304device.register_dump();
        Serial.println("#======================================");
        Serial.println("#Timer flexPWM");
        tcd1304device.register_dump(tcd1304device.timerflexpwm,tcd1304device.timer.mask);
      }
      else if ((startsWith(rcvbuffer, "stop"))) {
        tcd1304device.stop_flexpwm();
      }
      else if ((startsWith(rcvbuffer, "close"))) {
        tcd1304device.close();
      }
      
      else if (!(pc1=parseUint8(pc,&u8))) {
	Serial.print("failed to parse ");
	Serial.println(pc);
      }
      else {
        Serial.print("flexpwm parsed "); Serial.print(pc); Serial.print(" got "); Serial.println(u8);
        if ((pc[0]=='0') && (pc[1]=='x' || pc[1]=='x')) {
          mask = u8;
          while((submod<4)&&(mask!=(1<<submod)))submod++;
          
        }
        else {
          submod = u8;
          mask = 1<<submod;            
        }

        pc = pc1;
	while (pc&&*pc&&isspace(*pc))pc++;

        // these can accept a mask for one or more submodules
        if (!mask || (mask>0xF)) {
          Serial.println("Error: need mask in range 0x1 to 0xF or submodule 1 to 3");
        }
        
        else if ((startsWith(pc,"clear load ok"))) {
          Serial.print("clear ldok "); Serial.println(mask,HEX);
          tcd1304device.clear_ldok((mask));
        }
        
        else if ((startsWith(pc,"set load ok"))) {
          Serial.print("set ldok "); Serial.println(mask,HEX);
          tcd1304device.set_ldok((mask));
        }
        else if ((startsWith(pc,"clear run"))) {
          Serial.print("clear run "); Serial.println(mask,HEX);
          tcd1304device.clear_run((mask));
        }

        else if ((startsWith(pc,"set run"))) {
          Serial.print("set run "); Serial.println(mask,HEX);
          tcd1304device.set_run(mask);
        }

        // after this the options all require a specific submodule
        else if (submod>3) {
          Serial.print("Error: need submodule 0 to 3 for ");
          Serial.println(pc);
        }
        
        else if ((startsWith(pc,"set clock master"))) {
          Serial.print("set clock master "); Serial.println(submod);
          tcd1304device.set_clock_master(submod);
        }
        else if ((startsWith(pc,"set clock slave"))) {
          Serial.print("set clock slave "); Serial.println(submod);
          tcd1304device.set_clock_slave(submod);
        }
        else if ((startsWith(pc,"set clock sync"))) {
          Serial.print("set clock sync "); Serial.println(submod);
          tcd1304device.set_clock_sync(submod);
        }
        
        else if ((startsWith(pc,"force"))) {
          Serial.print("set force "); Serial.println(submod);
          tcd1304device.force(submod);
        }

        else if ((pc1=startsWith(pc,"set init"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d init %d", submod, val);
            tcd1304device.set_init(submod,val);
          }
        }

        else if ((pc1=startsWith(pc,"set val0"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val0 %d", submod, val);
            tcd1304device.set_val0(submod,val);
          }
        }

        else if ((pc1=startsWith(pc,"set val1"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val1 %d", submod, val);
            tcd1304device.set_val1(submod,val);
          }
        }
        
        else if ((pc1=startsWith(pc,"set val2"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val2 %d", submod, val);
            tcd1304device.set_val2(submod,val);
          }
        }

        else if ((pc1=startsWith(pc,"set val3"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val3 %d", submod, val);
            tcd1304device.set_val3(submod,val);
          }
        }
        
        else if ((pc1=startsWith(pc,"set val4"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val4 %d", submod, val);
            tcd1304device.set_val4(submod,val);
          }
        }
        
        else if ((pc1=startsWith(pc,"set val5"))) {
          if (parseUint16(pc1,&val)) {
            serialPrintlnf("set submodule %d val5 %d", submod, val);
            tcd1304device.set_val5(submod,val);
          }
        }
        else {
          Serial.println("Error: need: flexpwm  <dump|stop|close>||<submod|mask> commmand (see help)");
        }
      }
    }
    
    /* -----------------------------------------------------------
       Sync pin
    */
    else if ( (pc = startsWith( rcvbuffer, "set sync off")) ||
	      (pc = startsWith( rcvbuffer, "clear sync")) ) {
      Serial.println( "setting sync off" );
      Serial.println( "not implemented yet" );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync exposure")) ) {
      Serial.println( "setting sync shutter" );
      Serial.println( "not implemented yet" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync shutter")) ) {
      Serial.println( "setting sync shutter" );
      Serial.println( "not implemented yet" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync start")) ) {
      Serial.println( "setting sync start" );
      Serial.println( "not implemented yet" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync holdoff")) ) {
      parseUint( pc, &utemp );
      Serial.print( "setting sync holdoff " );
      Serial.print( utemp );
      Serial.print( " usecs " );
      Serial.println( "not implemented yet" );
    }

    /* -----------------------------------------------------------
       Set any pin
     */
    else if ((pc=startsWith(rcvbuffer, "read"))&&(pc1=parsePin(pc,&utemp))) {

      utemp2 = digitalRead(utemp);
      serialPrintlnf( "Pin %d read %d", utemp, utemp2);
    }
    
    else if ((pc=startsWith(rcvbuffer,"set"))) {

      if (!(pc1=parsePin(pc,&utemp))) {
        Serial.println("Error: need: set <pin> <see help>, failed to parse pin");
      }

      else if ( (pc = startsWith(pc1, "hi" )) ) {
        digitalWriteFast(utemp, HIGH);
        serialPrintlnf( "Pin %d set HIGH", utemp);
      } 
    
      else if ( (pc = startsWith(pc1, "lo" )) ) {
        digitalWriteFast(utemp, LOW);
        serialPrintlnf( "Pin %d set LOW", utemp);
      }

      else if ( (pc = startsWith(pc1, "output" )) ) {
        pinMode(utemp, OUTPUT);
        serialPrintlnf( "Pin %d set OUTPUT", utemp);
      }
      
      else if ((pc=startsWith(pc1, "input"))) {
        pinMode(utemp, INPUT);
        serialPrintlnf( "Pin %d set INPUT", utemp);
      }      

      else if ((pc=startsWith(pc1, "nopullup"))) {
        pinMode(utemp, INPUT);
        serialPrintlnf( "Pin %d set INPUT", utemp);
      }      

      else if ( (pc = startsWith(pc1, "pullup" )) ) {
        pinMode(utemp, INPUT_PULLUP);
        serialPrintlnf( "Pin %d set PULLUP", utemp);
      }
      
      else {
        Serial.println("Error: need: set <pin> <hi|lo|output|input|nopullup|pullup>");
      }      
    }
    
    else if ((pc=startsWith( rcvbuffer, "pulse"))) {

      if ((pc1=parsePin(pc,&utemp))) {
      
        utemp2 = 1;
        parseUint( pc, &utemp2 );

        pulsePin( utemp, utemp2 );     
        serialPrintlnf( "pin %d pulsed %dusecs", utemp, utemp2);
      }
      else {
        Serial.println("Error: pulse <pin> [usecs] failed to parse pin");
      }
    }

    else if ((pc=startsWith( rcvbuffer, "toggle"))) {

      if ((pc1=parsePin(pc,&utemp))) {
        digitalToggleFast(utemp);
        serialPrintlnf( "pin %d toggled", utemp);
      }
      else {
        Serial.println("Error: toggle <pin> failed to parse pin");
      }
    }
    
    /* -----------------------------------------------------------
       Synchronous Data transmit,test and buffer managament
    */
    
    else if ( (pc = startsWith( rcvbuffer, "send crc" )) ||
              (pc = startsWith( rcvbuffer, "crc" )) ) {    
      sendDataCRC( );
      
    }
    
    else if ((pc = startsWith( rcvbuffer, "send sum" )) ||
             (pc = startsWith( rcvbuffer, "sum" ))) {    
      sendDataCRC( );
      
    }
    else if ((pc = startsWith( rcvbuffer, "send"))) {    
      sendData( );
    }
    
    
    /* =================================================================
       Check if active, stop required for further commands
    */
    else if (tcd1304device.busy) {
      Serial.println( "Error: device active, need stop before other commands" );
    }
    
    // this one needs stop first
    else if ( (pc = startsWith( rcvbuffer, "send test" )) ) {    
      sendTestData( );
    }

    /* =================================================================
       Over current pin
    */

#ifdef CONTROLLER_OCPIN
    else if ( (pc = startsWith( rcvbuffer, "attach ocpin" )) ) {
      ocpinAttach();      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "deattach ocpin" )) ) {
      ocpinDetach();      
    }

    else if ( (pc = startsWith( rcvbuffer, "ocpin" )) ) {
      ocpinRead();      
    }
#endif

    
    /* =================================================================
       The host program queries this to setup its buffers
    */    
    else if ( (pc = startsWith( rcvbuffer, "configuration" ))) {

      /*
      sprintf( sndbuffer, "PIXELS %u DARK %u VPERBIT %.8f",
	       NPIXELS, NDARK, 3.3/4096 );
      Serial.println( sndbuffer );
      */
      
      sprintf( sndbuffer, "PIXELS %u DARK %u BITS %u VFS %f",
	       NPIXELS, NDARK, NBITS, VFS );
      Serial.print( sndbuffer );

      Serial.print( " SENSOR " );
      Serial.println( sensorstr );
      
#ifdef DIAGNOSTICS_CPU
      Serial.print("F_CPU: ");
      Serial.print(F_CPU/1e6);
      Serial.println(" MHz."); 

      Serial.print("ADC_F_BUS: ");
      Serial.print(ADC_F_BUS/1e6);
      Serial.println(" MHz.");
#endif
      
    }

    else if ( (pc = startsWith( rcvbuffer, "pins" )) ) {

      Serial.println("#Pins");
      serialPrintf("# Trigger(input) %d Busy %d Sync %d",
		   tcd1304device.interrupt_pin,
		   BUSY_PIN,
		   SYNC_PIN);
      
      serialPrintf(" TCD1304 clock %d icg %d sh %d",
		   tcd1304device.clk.pinA,
		   tcd1304device.icg.pinA,
		   tcd1304device.sh.pinA);

      serialPrintf(" SPI clock %d sdi %d sdo %d cnvst %d", CLKPin, SDIPin, SDOPin, CNVSTPin );
     
#ifdef CONTROLER_R4_OCPIN
      Serial.print("  ~OverCurrent " );
      Serial.print( CONTROLLER_R4_OCPIN );
#endif
      Serial.println( "" );
      
    }
    
    /* -----------------------------------------------------------
       User store/print coefficients
    */
    else if ( (pc = startsWith( rcvbuffer, "store coefficients" )) || (pc = startsWith( rcvbuffer, "set coefficients" )) ) {
      float vals[EEPROM_NCOEFFS] = {0};
      if (parseFlts( pc, vals, EEPROM_NCOEFFS)) {
	storeCoefficients(vals);
      }      
      printCoefficients( );      
    }

    else if ( (pc = startsWith( rcvbuffer, "coefficients" )) ) {
      printCoefficients( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase coefficients" )) ) {
      eraseCoefficients();
    }
    
    /* -----------------------------------------------------------
       User store/print identifier
    */

    else if ( (pc = startsWith( rcvbuffer, "store identifier" )) || (pc = startsWith( rcvbuffer, "set identifier" )) ) {
      while( *pc && isspace(*pc) ) pc++;
      storeIdentifier( pc );      
      printIdentifier( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "identifier" )) ) {
      printIdentifier( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase identifier" )) ) {
      eraseIdentifier();
    }
    
    /* -----------------------------------------------------------
       User store/print units
    */

    else if ( (pc = startsWith( rcvbuffer, "store units" )) || (pc = startsWith( rcvbuffer, "set units" )) ) {
      while( *pc && isspace(*pc) ) pc++;
      storeUnits( pc );      
      printUnits( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "units" )) ) {
      printUnits( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase units" )) ) {
      eraseUnits();
    }
    
    /* -----------------------------------------------------------
       Data async data
     */
    else if ( (pc = startsWith( rcvbuffer, "set async" )) ) {
      data_async = true;
      Serial.println( "transfer asynchronous" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set synch" )) ) {
      data_async = false;
      Serial.println( "transfer synchronous" );
    }

    else if ( (pc = startsWith( rcvbuffer, "enable crc")) ) {
      Serial.println( "enabled crc" );
      crc_enable = true;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "disable crc")) ) {
      Serial.println( "disabled crc" );
      crc_enable = false;
    }

    else if ( (pc = startsWith( rcvbuffer, "enable sum")) ) {
      Serial.println( "enabled sum" );
      sum_enable = true;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "disable sum")) ) {
      Serial.println( "disabled sum" );
      sum_enable = false;
    }
    /* -----------------------------------------------------------
       Data output format
     */
    
    else if ( (pc = startsWith( rcvbuffer, "set ascii" )) || (pc = startsWith( rcvbuffer, "set formatted" )) ) {
      dataformat = ASCII;
      Serial.println( "format ascii" );
    }

    else if ( (pc = startsWith( rcvbuffer, "set binary" )) ) {
      dataformat = BINARY;
      Serial.println( "format binary" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "format" )) ) {
      if ( dataformat == BINARY ) {
	Serial.println( "format binary" );
      }
      else if ( dataformat == ASCII ) {
	Serial.println( "format ascii" );
      }
      else {
	Serial.println( "format unknown" );
      }
    }
    
    /* -----------------------------------------------------------
       ADC reporting
    */
    else if ( (pc = startsWith( rcvbuffer, "set adcs off")) ) {
      Serial.println( "setting adc reporting off" );
      adc_averages = 0;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set adcs")) && (pc = parseUint( pc, &adc_averages )) ) {
      Serial.print( "setting adc reporting, averaging " );
      Serial.println( adc_averages );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "adcs")) || (pc = startsWith( rcvbuffer, "read analog inputs")) ) {
      utemp = 1;
      parseUint( pc, &utemp );
      if (utemp) {
	Serial.println("reading ADCS");
	sendADCs( utemp );
      }
    }

    /* ------------------------------------------------------------
       Temperature reporting
     */
    else if ( (pc = startsWith( rcvbuffer, "set temperature off")) ) {
      Serial.println( "setting chip temperature reporting off" );
      chipTemp_averages = 0;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set temperature on")) ) {
      Serial.println( "setting chip temperature reporting on" );
      chipTemp_averages = 1;
    }
    
    else if ( !chipTemp_averages && (pc = startsWith( rcvbuffer, "temperature" )) ) {
      Serial.print( "CHIPTEMPERATURE " );
      Serial.println( tempmonGetTemp() );
    }
    
    /* -----------------------------------------------------------
       Test the ADC, benchmark
    */
    else if ( (pc = startsWith( rcvbuffer, "test adc")) ) {
      unsigned int usecs;
     
      utemp = 1;
     
      parseUint( pc, &utemp );
     
      Serial.print( "command: test adc " );
      Serial.println( utemp );
       
      usecs = measureADCspeed( analogPin, utemp );

      Serial.print( "elapsed time " );
      Serial.println( usecs );
    }

    else {
      Serial.print("Error: unrecognized command //");
      Serial.print(rcvbuffer);
      Serial.println("//");
    }

   // Indicate that we processed this message
    nlen = 0;
    
    Serial.println( "DONE" );
  }
}
