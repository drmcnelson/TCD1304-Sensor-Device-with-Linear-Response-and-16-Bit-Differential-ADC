/**@file*/
/*
  Author:   Mitchell C. Nelson
  Date:     Dec 5, 2025
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

#include "TCD1304Device2.h"

#include "parselib2.h"

#include <SPI.h>

// SPI interface to the ADC
SPISettings spi_settings(30000000, MSBFIRST, SPI_MODE0);   // 30 MHz, reads 1.5usecs/word including CNVST

#include <limits.h>

#include <ADC.h>
#include <ADC_util.h>

#include <EEPROM.h>

extern float tempmonGetTemp(void);

// ADC setup - need this before tcd1304 if using the onboard adc
ADC *adc = new ADC( );

// TCD1304 Setup
TCD1304Device tcd1304device = TCD1304Device( );


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

const char versionstr[] = "TCD1304Device vers 0.4 " __DATE__;
const char authorstr[] =  "Patents Pending and (c) 2020, 2023, 2025 by Mitchell C. Nelson, Ph.D. ";
const char srcfilestr[] = __FILE__ ;

const char sensorstr[] = "TCD1304";

const int syncPin      = SYNC_PIN;        // Goes low for trigger standoff (delay) period, use this for desktop N2 laser
const int busyPin      = BUSY_PIN;        // Gate output pin, goes high during shutter
const int triggerPin   = TRIGGER_PIN;   // Trigger input pin
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

void blink( ) {
}

void pulsePin(unsigned int pin, unsigned int usecs) {
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

void ocpinRead( ) {
  ocpinstate = digitalRead(CONTROLLER_OCPIN);
  Serial.print("#OCPIN state ");
  Serial.println(ocpinstate);
  if (!ocpinstate) {
    Serial.println("#OverCurrent set");
  }
  else {
    Serial.println("#OverCurrent clear");
  }
}

void ocpinISR( ) {
  ocpinstate = digitalRead(CONTROLLER_OCPIN);
  if (!ocpinstate) {
    Serial.println("ERROR: OverCurrent set");
  }
  else {
    Serial.println("#OverCurrent clear");
  }
}

void ocpinAttach( ) {
  if (!ocpinattached) {
    attachInterrupt(digitalPinToInterrupt(CONTROLLER_OCPIN), ocpinISR, CHANGE);
    ocpinattached = true;
    Serial.println("#OCPIN attached");
  }
  else {
    Serial.println("#OCPIN already attached");
  }
}

void ocpinDetach( ) {
  if (ocpinattached) {
    detachInterrupt(digitalPinToInterrupt(CONTROLLER_OCPIN));
    ocpinattached = false;
    Serial.println("#OCPIN detached");
  }
  else {
    Serial.println("#OCPIN already detached");
  }
}
#endif

/* ===========================================================================================
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

/* ===========================================================================================
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

/* ===========================================================================================
   ADC single reads
   (note that the alpha0 board used the first two channels)
 */
#define NADC_CHANNELS 4
#define ADC_RESOLUTION (3.3/4096.)
uint16_t adc_data[NADC_CHANNELS] = { 0 };

// Set this to something more than 0 to turn on adc reporting.
unsigned int adc_averages = 0;

/* Fast analog input read
 */
inline int fastAnalogRead(uint8_t pin) {
  adc->adc0->startReadFast(pin);
  while (adc->adc0->isConverting( ));
  return adc->adc0->readSingle( );
}

/* Send ADC readings
 */
void sendADCs(unsigned int averages) {

  unsigned int i;
  float scalefactor = ADC_RESOLUTION/averages;
  float val;

  int n = averages;

  for(i = 0 ; i < NADC_CHANNELS; i++) {
    adc_data[i] = fastAnalogRead(i);
  }
  n--;
  
  while(n--) {
    for(i = 0 ; i < NADC_CHANNELS; i++) {
      adc_data[i] += fastAnalogRead(i);
    }
  }
  
  Serial.print("ADCAVGS ");
  Serial.println(averages);
  
  Serial.print("ADCDATA");
  for(i = 0 ; i < NADC_CHANNELS; i++) {
    val = adc_data[i] * scalefactor;
    Serial.print(" ");
    Serial.print(val, 5);
  }
  
  Serial.println("");
}

/* ===========================================================================================
   Send chip temperature
 */

// Set this to something more than 0 to turn on chip temperature reporting
unsigned int chipTemp_averages = 0;

void sendChipTemperature(unsigned int averages){
  Serial.print("CHIPTEMPERATURE ");
  Serial.println(tempmonGetTemp( ));
}

/* ===========================================================================================
 * EEPROM support for saving identifier and coordinate mapping constants
 */
#define EEPROM_SIZE 1080

#define EEPROM_ID_ADDR 0
#define EEPROM_ID_LEN 64

#define EEPROM_COEFF_ADDR 64
#define EEPROM_NCOEFFS 4
#define EEPROM_COEFF_LEN (EEPROM_NCOEFFS * sizeof(float))

#define EEPROM_UNITS_ADDR (EEPROM_COEFF_ADDR + EEPROM_COEFF_LEN)
#define EEPROM_NUNITS 8
#define EEPROM_UNITS_LEN EEPROM_NUNITS

void eeread(unsigned int address, int nbytes, char *p) {
  while(nbytes-- > 0  && address < EEPROM_SIZE) {
    *p++ = EEPROM.read(address++);
  }
}

void eewrite(unsigned int address, int nbytes, char *p) {
  while(nbytes-- > 0 && address < EEPROM_SIZE) {
    EEPROM.write(address++, *p++);
  }
}

// -------------------------------------------------------------

void eereadUntil(unsigned int address, int nbytes, char *p) {
  char b = 0xFF;
  while(nbytes-- > 0  && address < EEPROM_SIZE && b) {
    b = EEPROM.read(address++);
    *p++ = b;
  }
}

void eewriteUntil(unsigned int address, int nbytes, char *p) {
  while(nbytes-- > 0 && address < EEPROM_SIZE-1 && *p) {
    EEPROM.write(address++, *p++);
  }
  EEPROM.write(address++, 0);
}

void eeErase(unsigned int address, int nbytes) {
  while(nbytes-- > 0 && address < EEPROM_SIZE-1) {
    EEPROM.write(address++, 0xFF);
  }
}

/* -------------------------------------------------------------
   Store/read Identifier
*/
void eraseIdentifier( ) {
  eeErase(EEPROM_ID_ADDR, EEPROM_ID_LEN);
}

void readIdentifier(char *p) {
  eereadUntil(EEPROM_ID_ADDR, EEPROM_ID_LEN, p);
}

void storeIdentifier(char *p) {
  eewriteUntil(EEPROM_ID_ADDR, EEPROM_ID_LEN, p);
}

void printIdentifier( ) {
  readIdentifier(sndbuffer);
  if (sndbuffer[0] != 0xff) {
    Serial.print("Identifier: ");
    Serial.println(sndbuffer);
  }
  else {
    Serial.println("Warning:  identifier is not set");
  }
}

/* -------------------------------------------------------------
   Store/read Units
*/
void eraseUnits( ) {
  eeErase(EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN);
}

void readUnits(char *p) {
  eereadUntil(EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p);
}

void storeUnits(char *p) {
  eewriteUntil(EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p);
}

void printUnits( ) {
  readUnits(sndbuffer);
  if (sndbuffer[0] != 0xff) {
    Serial.print("units: ");
    Serial.println(sndbuffer);
  }
  else {
    Serial.println("units: none");
  }
}

/* -------------------------------------------------------------
   Store/read Wavelength Coeffs
*/

void eraseCoefficients( ) {
  eeErase(EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN);
}

void readCoefficients(float *vals) {
  eeread(EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals);
}

void storeCoefficients(float *vals) {
  eewrite(EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals);
}

void printCoefficients( ) {

  float vals[EEPROM_NCOEFFS];

  readCoefficients(vals);
  
  Serial.print("coefficients");
  
  for(int n = 0; n < EEPROM_NCOEFFS; n++) {
    sprintf(sndbuffer, " %.8g", vals[n]);
    Serial.print(sndbuffer);
  }
  Serial.println("");
}

// ========================================================
// Timer-clocked flexpwm framesets (i.e. the short time-scale framesets)

IntervalTimer clock_timer;

void (*clock_callback)( ) = nullptr;
unsigned int clock_counts = 0;
unsigned int clock_counter = 0;
float clock_usecs = 0;
bool clock_mode = false;

void clock_isr( )
{
  clock_callback( );
  clock_counter++;
  if (clock_counter >= clock_counts) {
    clock_timer.end( );
  }
}

void clock_stop( )
{
  clock_timer.end( );
  clock_mode = false;
}

void clock_start( )
{
  clock_mode = true;
  if (clock_callback && clock_usecs) {
    clock_timer.begin(clock_callback, clock_usecs);
    clock_isr( );
  }
  else {
    clock_stop( );
    Serial.println("Error: clock_start but clock is not set up");
  }
}

bool clock_setup(float secs, unsigned int ncounts=0)
{
  if (tcd1304device.mode==PULSE) {
    clock_callback = tcd1304device.pulse_start;
    clock_counts = ncounts ? ncounts+1 : tcd1304device.frame_counts+1;
    clock_usecs  = secs * 1.E6;
    tcd1304device.frame_counts = clock_counts - 1 ;
    return true;
  }
  else {
    Serial.println("Error: clock setup needs tcd1304 device pulse already setup");
  }
  return false;
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
  uint8_t *pc = (unsigned char *)&bufferp[DATASTART];
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
  for (n = 0; n < NPIXELS; n++) {
    u += *p16++;
  }
  return u;
}

/* ==========================================================================
   Data buffer and send functions
*/
#define NBUFFERS 32

TCD1304Device :: Frame_Header frame_header_ring[NBUFFERS] = {0};
TCD1304Device :: Frame_Header *framep = &frame_header_ring[0];

uint16_t buffer_ring[NBUFFERS][NREADOUT] = { 0 };
uint16_t *bufferp = buffer_ring[0];

unsigned int frame_index = 0;
unsigned int buffer_index = 0;

unsigned int frame_send_index = 0;

// -------------------------------------
/* Data transfer format
 */
#define BINARY 0
#define ASCII 1
unsigned int dataformat = BINARY;
unsigned int counter = 0;

bool data_async = true;
bool crc_enable = false;
bool sum_enable = false;

void sendBuffer_Formatted(uint16_t *bp) {
  uint16_t *p16 = &bp[DATASTART];
  Serial.print("DATA ");
  Serial.println(NPIXELS);
  for (int n = 0; n < NPIXELS; n++) {
    Serial.println(p16[n]);
  }
  Serial.send_now();
  Serial.println("END DATA");
}

void sendBuffer_Binary(uint16_t *bp) {
  uint16_t *p16 = &bp[DATASTART];
  Serial.print("BINARY16 ");
  Serial.println(NPIXELS);
  Serial.write((byte *) p16, NBYTES);
  Serial.send_now();
  Serial.println("END DATA");
}

inline void sendDataCRC(uint16_t *bp) {
  unsigned char crc;
  crc = calculateCRC(bp);
  Serial.print("CRC 0x");
  Serial.println(crc, HEX);
}

inline void sendDataSum(uint16_t *bp) {
  uint32_t usum = 0;
  usum = sumData(bp);
  Serial.print("SUM ");
  Serial.println(usum);
}

void sendData(uint16_t *bp) {

  Serial.send_now();
  
  if (crc_enable) sendDataCRC(bp);
  if (sum_enable) sendDataSum(bp);
  
  if (dataformat == BINARY) {
    sendBuffer_Binary(bp);
  }
  else if (dataformat == ASCII) {
    sendBuffer_Formatted(bp);
  }
  Serial.send_now();
}

void sendDataReady( ) {
  Serial.println("READY DATA");
}

void sendTestData(uint16_t *bp) {
  uint16_t *p16 = &bp[DATASTART];
  Serial.println("TESTDATA");
  // load test parttern
  for (int n = 0; n < NPIXELS; n++) {
    p16[n] = n;
  }
  // send it
  sendData(bp);
}

/* ======================================================================================================
   CCD sensor read callback
   ====================================================================================================== */

void send_header()
{
  Serial.print("START");

  if (tcd1304device.trigger_mode) {
    Serial.print(" TRIGGER");
  }
  
  if (tcd1304device.mode == FRAMESET) {
    Serial.print(" FRAMESET");
  }
  else if (tcd1304device.mode == PULSE) {
    Serial.print(" PULSE");
  }

  else if (tcd1304device.mode == TIMER) {
    Serial.print(" TIMER");
  }

  if (clock_mode) {
    Serial.print(" CLOCK");
  }

  Serial.println("");

  // ------------------------------------------------
  
  Serial.print("FRAME COUNTS ");
  Serial.println(tcd1304device.frame_counts);

  Serial.print("FRAMESET COUNTS ");
  Serial.println(tcd1304device.frameset_counts);

  if (clock_mode) {
    Serial.print("CLOCK PERIOD ");
    Serial.println((int)clock_usecs);
  }
  
  if (tcd1304device.mode == FRAMESET) {
    Serial.print("SH PERIOD ");
    Serial.println((int)(tcd1304device.sh.period_secs*1.E6));
  }  
  else if (tcd1304device.mode == TIMER) {
    Serial.print("TIMER PERIOD ");
    Serial.println((int)(tcd1304device.timer_interval_secs*1.E6));

    if (tcd1304device.timer_period_secs<tcd1304device.timer_interval_secs) {
      Serial.print("TIMER SUBPERIOD ");
      Serial.println((int)(tcd1304device.timer_period_secs*1.E6));
    }
  }
  
  if (tcd1304device.trigger_mode) {
    Serial.print("TRIGGER COUNTS ");
    Serial.println(tcd1304device.trigger_counts);
  }
}

// ==========================================================
// Send the frame, header and data

bool send_frame(TCD1304Device :: Frame_Header *p)
{

#ifdef DEBUG
  Serial.printf("#send_frame 0x%p->0x%p\n",p,p->buffer);
#endif

  if (p->frame_counter<1) {
    Serial.println("FRAMESET START ");
  }

  Serial.print("FRAMESET COUNTER ");
  Serial.println(p->frameset_counter);

  Serial.print("FRAME COUNTER ");
  Serial.println(p->frame_counter);

  Serial.print("FRAME ELAPSED ");
  Serial.println(p->frame_elapsed_secs,8);

  Serial.print("FRAME EXPOSURE ");
  Serial.println(p->frame_exposure_secs,8);

#ifdef DEBUG  
  Serial.print("FRAME_DIFFERENCE ");
  Serial.println(p->frame_difference_secs,8);
#endif
  
  if (p->mode==TIMER) {
    Serial.print("TIMER ELAPSED ");
    Serial.println(p->timer_elapsed_secs,8);
    Serial.print("TIMER DIFFERENCE ");
    Serial.println(p->timer_difference_secs,8);
  }
    
  if (p->trigger_mode) {
    Serial.print("TRIGGER ELAPSED ");
    Serial.println(p->trigger_elapsed_secs,8);
    Serial.print("TRIGGER DIFFERENCE ");
    Serial.println(p->trigger_difference_secs,8);
    Serial.print("TRIGGER COUNTER ");
    Serial.println(p->trigger_counter);
  }

  // send the raw offset as average of the dummy pixels
  Serial.print("RAWOFFSET ");
  Serial.println(p->avgdummy);

  // send the voltage offset based on the dummy pixels
  Serial.print("OFFSET ");
  Serial.println(p->offset,6);

  // Send the data buffer
  sendData(p->buffer);

  // These have to come after sending the daa
  if (p->frames_completed) {
    Serial.println("FRAMESET END");

    if (p->framesets_completed) {
      Serial.println("COMPLETE");
    }
  }

  // Clear the ready flag, this one is now available
  p->ready_for_send = false;
  
  return true;
}

// =====================================================
// call this at the top of loop

void send_frames()
{
  while(frame_header_ring[frame_send_index].ready_for_send) {
    send_frame(&frame_header_ring[frame_send_index]);
    frame_send_index = (frame_send_index+1) % NBUFFERS;
  }
  
}


// =====================================================
// here is the callback for read complete, load the frame header and advance the pointers

void frame_callback( )
{
  unsigned int next_frame_index;
  TCD1304Device :: Frame_Header *next_framep;

  // Ask the device to populate the current frame header, gets a pointer to the buffer
  tcd1304device.fill_frame_header(framep);

  // Here is the next frame
  next_frame_index = (frame_index+1) % NBUFFERS;
  next_framep = &frame_header_ring[next_frame_index];

  // Check that the next frame is available
  if (!next_framep->ready_for_send) {

    // Advance the frame index to this next frame
    frame_index = next_frame_index;
    framep = next_framep;

    // Advance the buffer index to this next buffer
    buffer_index = (buffer_index+1) % NBUFFERS;
    bufferp = buffer_ring[buffer_index];

    // Set the read pointer to this next buffer
    tcd1304device.update_read_buffer(bufferp);
  }

  // If counts not completed, then it is an overrun (will clear eventually, from loop)
  else if (!frame_header_ring[frame_index].framesets_completed) {
    Serial.println("Error: frame ring overrun");
    tcd1304device.error_flag = true;
  }
}

/* ==========================================================================
   Parse for the trigger edge specification (rising,falling,change)
 */

bool strPin(char *pc, uint8_t *pin, char **next)
{

#ifdef DEBUG
  Serial.print("strPin ");
  Serial.println(pc);
#endif

  if (strMatch(pc,"pin",&pc) && strUint8(pc,pin,next)) {
#ifdef DEBUG
    Serial.print(" recognized pin ");
    Serial.println(*pin);
#endif
    return true;
  }

  if (strMatch(pc,"sync",next)) {
    *pin = syncPin;
#ifdef DEBUG
    Serial.println(" recognized sync ");
#endif
    return true;
  }

  if (strMatch(pc,"busy",next)) {
    *pin = busyPin;
#ifdef DEBUG
    Serial.println(" recognized busy ");
#endif
    return true;
  }

  if (strMatch(pc,"trigger",next)) {
    *pin = triggerPin;
#ifdef DEBUG
    Serial.println(" recognized trigger ");
#endif
    return true;
  }

  return false;
}

void printTriggerSetup( )
{
  Serial.print("#trigger pin ");
  Serial.print(tcd1304device.trigger_pin);

  switch(tcd1304device.trigger_pin_mode)
    {
    case INPUT:
      Serial.print(" input");
      break;
    case INPUT_PULLUP:
      Serial.print(" pullup");
      break;
    case INPUT_PULLDOWN:
      Serial.print(" pulldown");
      break;
    }
  
  switch(tcd1304device.trigger_edge_mode)
    {
    case RISING:
      Serial.print(" rising");
      break;
    case FALLING:
      Serial.print(" falling");
      break;
    case CHANGE:
      Serial.print(" change");
      break;
    }

  if (tcd1304device.trigger_attached) {
    Serial.println(" isattached");
  }
  else {
    Serial.println("");
  }
}

bool strTrigger(char *pc, char **next)
{
  tcd1304device.stop_triggers( );
  Serial.println("#trigger interrupts stopped");

  uint8_t pin;

  while(pc && *pc) {

    if (strPin(pc,&pin,&pc)) {
      tcd1304device.trigger_pin = pin;
      pinMode(tcd1304device.trigger_pin, tcd1304device.trigger_pin_mode);
    }

    else if (strMatch(pc, "nopull",&pc)||strMatch(pc, "input",&pc)) {
      pinMode(tcd1304device.trigger_pin,INPUT);
      tcd1304device.trigger_pin_mode = INPUT;
    }

    else if (strMatch(pc, "pullup",&pc)) {
      pinMode(tcd1304device.trigger_pin,INPUT_PULLUP);
      tcd1304device.trigger_pin_mode = INPUT_PULLUP;
    }
    
    else if (strMatch(pc, "pulldown",&pc)) {
      pinMode(tcd1304device.trigger_pin,INPUT_PULLDOWN);
      tcd1304device.trigger_pin_mode = INPUT_PULLDOWN;
    }
    
    else if (strMatch(pc, "rising",&pc)) {
      tcd1304device.trigger_edge_mode = RISING;
    }

    else if (strMatch(pc, "falling",&pc)) {
      tcd1304device.trigger_edge_mode = FALLING;
    }
    
    else if (strMatch(pc, "change",&pc)) {
      tcd1304device.trigger_edge_mode = CHANGE;
    }
    
    else {
      return false;
    }

    while (pc && (*pc) && isspace(*pc)) pc++;
  }

  if (next) *next = pc;
  
  return true;
}

bool strSubModule(char *s, char **next=0)
{
  uint8_t mask = 0;
  uint8_t submod = 0;
  IMXRT_FLEXPWM_t *q = nullptr;
  
  TCD1304Device :: SubModule *p = nullptr;
  
  if (strMatch(s,"sh",&s)) {
    p = &tcd1304device.sh;
  }
  else if (strMatch(s,"clk",&s)) {
    p = &tcd1304device.clk;
  }
  else if (strMatch(s,"icg",&s)) {
    p = &tcd1304device.clk;
  }
  else if (strMatch(s,"cnvst",&s)) {
    p = &tcd1304device.cnvst;
  }
  else if (strMatch(s,"timer",&s)) {
    p = &tcd1304device.cnvst;
  }

  if (!p) {
    Serial.println("Error: submodule name not recognized");
    return false;
  }

  q      = p->flexpwm;
  mask   = p->mask;
  submod = p->submod;

  while (s && *s) {

    if (!(s = nextWord(s))) break;
    
    if (strMatch(s,"prescale",&s)) {
      strUint8lim(s,&p->prescale,&s,7);
      p->divider = (1<<p->prescale);
      p->newvals = true;
    }
    else if (strMatch(s,"period",&s)) {
      strUint16(s,&p->period_counts,&s);
      p->newvals = true;
    }    
    // ---------------------------------------------------
    else if (strMatch(s,"onb",&s)) {
      strUint16(s,&p->onB_counts,&s);
      p->newvals = true;
    }
    else if (strMatch(s,"offb",&s)) {
      strUint16(s,&p->offB_counts,&s);
      p->newvals = true;
    }
    // ---------------------------------------------------
    else if (strMatch(s,"ona",&s)||strMatch(s,"on",&s)) {
      strUint16(s,&p->onA_counts,&s);
      p->newvals = true;
    }
    else if (strMatch(s,"offa",&s)||strMatch(s,"off",&s)) {
      strUint16(s,&p->offA_counts,&s);
      p->newvals = true;
    }
    // ---------------------------------------------------
    else if (strMatch(s,"invertb",&s)) {
      p->invertB = true;
      p->newvals = true;
    }
    else if (strMatch(s,"noinvertb",&s)) {
      p->invertB = false;
      p->newvals = true;
    }
    // ---------------------------------------------------
    else if (strMatch(s,"inverta",&s)||strMatch(s,"invert",&s)) {
      p->invertA = true;
      p->newvals = true;
    }
    else if (strMatch(s,"noinverta",&s)||strMatch(s,"noinvert",&s)) {
      p->invertA = false;
      p->newvals = true;
    }    
    // ---------------------------------------------------
    else if (strMatch(s,"master",&s)) {
      p->ctrl2_mask = PWM_CTRL2_CLOCK_MASTER;
      p->newvals = true;
    }
    else if (strMatch(s,"slave",&s)) {
      p->ctrl2_mask = PWM_CTRL2_CLOCK_SLAVE;
      p->newvals = true;
    }
    else if (strMatch(s,"sync",&s)) {
      p->ctrl2_mask = PWM_CTRL2_CLOCK_SYNC;
      p->newvals = true;
    }
    else if (strMatch(s,"ctrl2",&s)) {
      strUint16(s,&p->ctrl2_mask,&s);
      p->newvals = true;
    }
    // ---------------------------------------------------
    else if (strMatch(s,"print",&s)||strMatch(s,"list",&s)) {
      tcd1304device.print_submodule(p);
    }

    else if (strMatch(s,"check",&s)) {
      tcd1304device.check_submodule(p);
    }
    
    // this loads the submodule settings into the submodule registers
    else if (strMatch(s,"load",&s)) {
      if (!tcd1304device.print_and_check_submodule(p)) {
        return false;
      }
      tcd1304device.load_submodule(p);
    }
    // ---------------------------------------------------
    else if (strMatch(s,"cldok",&s)||strMatch(s,"clear ldok",&s)) {
      strUint8lim(s,&mask,&s,0xF);
      tcd1304device.clear_ldok(mask,q);
    }
    
    else if (strMatch(s,"ldok",&s)||strMatch(s,"set dok",&s)) {
      strUint8lim(s,&mask,&s,0xF);
      tcd1304device.set_ldok(mask,q);
    }

    // ---------------------------------------------------
    else if (strMatch(s,"stop",&s)||strMatch(s,"clear run",&s)) {
      strUint8lim(s,&mask,&s,0xF);
      tcd1304device.clear_run(mask,q);
    }
    
    else if (strMatch(s,"run",&s)||strMatch(s,"set run",&s)) {
      strUint8lim(s,&mask,&s,0xF);

      if (p->newvals) {
        Serial.println("Warning: need load before run");
      }

      tcd1304device.set_run(mask,q);      
    }

    // ---------------------------------------------------
    else if (strMatch(s,"force",&s)) {
      strUint8lim(s,&submod,&s,3);
      tcd1304device.force(submod,q);
    }

    else {
      Serial.print("Error: submodule failed to parse ");
      Serial.println(s);
      return false;
    }

  }

  if (next) *next = s;
  
  return true;
}

/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength( ), these return a pointer to the
   next character in the string, or null if they fail

   code used to be here, now in parselib.c, .h
*/
  
/* ===================================================================
   Help text
 */
void help(const char *key) {

  char *s = 0;
  
  bool all = !key || !(*key) || !strncmp(key,"all",3);
  
  if (all || strMatch(key,"report",&s)) {
      Serial.println("#Report device, version and configuration");
      Serial.println("#  version           - report software version");
      Serial.println("#  configuration     - report device configuration and data structure");
      Serial.println("#  pins              - report digital i/o functions and pin numbers");
      //Serial.println("#  dump              - report all of the available settings and states");
      Serial.println("#  stop              - stop everything");
      Serial.println("#");
  }
  if (all || strMatch(key,"coefficients",&s)) {
    Serial.println("#Coefficients for pixel number to wavelength");
    Serial.println("#  store coefficients <a0> <a1> <a2> <a3> (need 4 numbers)");
    Serial.println("#  erase coefficients");
    Serial.println("#  coefficients       - report");
    Serial.println("#");
    Serial.println("#  store units <string> (upto 6 characters, c.f. nm, um, mm)");
    Serial.println("#  units              - report");
    Serial.println("#");
  }
  if (all || strMatch(key,"identifier",&s)) {
    Serial.println("#Identifier string (63 bytes)");
    Serial.println("#  store identifier <identifier>");
    Serial.println("#  identifier         - list identifier string");
    Serial.println("#");
  }
  if (all || strMatch(key,"data",&s)) {
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
  }
  if (all || strMatch(key,"microcontroller",&s)) {
    Serial.println("#Microcontroller functions");
    Serial.println("#  temperature        - report microcontroller temperature");
    Serial.println("#  reboot             - reboots the entire board");
    Serial.println("#");
  }
  if (all || strMatch(key,"adcs",&s)) {
    Serial.println("#ADCs Read and average analog inputs");
    Serial.println("#  adcs <navgs>        - read analog inputs and report");
    Serial.println("#  set adcs <navgs>    - read ADCs at frame completion");
    Serial.println("#  set adcs off");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s)) {
    Serial.println("#LCCD commands:");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s) || strMatch(key,"read",&s) || strMatch(key,"trigger",&s)) {
    Serial.println("#  1) High level read and trigger commands");
    Serial.println("#");
    Serial.println("#    read <exposure>  (secs)                  - read single frame");
    Serial.println("#    read <nframes> <exposure>                - read contiquous frames");
    Serial.println("#    read <nframes> <exposure>  <interval>    - read fast frame sequence");
    Serial.println("#");
    Serial.println("#    read ...   nostart                       - setup read without start");
    Serial.println("#    start read");
    Serial.println("#");
    Serial.println("#    wait read");
    Serial.println("#");
    Serial.println("#    trigger <exposure>  (secs)               - triggered read, single frame");
    Serial.println("#    trigger <nframes> <exposure>             - trigger read, contiquous frames");
    Serial.println("#    trigger <nframes> <exposure>  <interval> - trigger read, fast frame sequence");
    Serial.println("#");
    Serial.println("#    trigger ...   nostart                    - setup triggered read, without start");
    Serial.println("#    start trigger");
    Serial.println("#");
    Serial.println("#    wait trigger");
    Serial.println("#");
    Serial.println("#    gate frame <n>");
    Serial.println("#");
    Serial.println("#    configure trigger pin <n> | pullup | pulldown | nopull | rising | falling | change");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s) || strMatch(key,"pulse",&s)) {
    Serial.println("#  2) Single pulse setup (time syntax: 1E-6, 0.000001 or 1u)");
    Serial.println("#");
    Serial.println("#    setup pulse [<clk_period> <sh_width> <sh_offset> <icg_width> <icg>_offset> [nframes [nframesets]]] (in secs)");
    Serial.println("#");
    Serial.println("#          defaults to 0.5u 1u 0.6u 2.6u 0.5u 1 1");
    Serial.println("#");
    Serial.println("#          Generally this command will be followed by \"setup timer\" or \"setup trigger\"");
    Serial.println("#");
    Serial.println("#      start pulse - launches a single pulse with readout");
    Serial.println("#      stop pulse  - stops ongoing pulse");
    Serial.println("#");
    Serial.println("#      arm pulse   - re-arm for the next pulse, see set nframes and nframesets");
    Serial.println("#      init pulse  - re-arms and sets frame counter to 0");
    Serial.println("#");
    Serial.print("#      configure clearing pulses [n] - power-up default is "); Serial.println(SH_CLEARING_DEFAULT);
    Serial.println("#         (SH is pulsed before the start of the next exposure to clear residual charge)");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s)|| strMatch(key,"frameset",&s)) {
    Serial.println("#  3) Frameset setup - provides short exposure time (time syntax: 1E-6, 0.000001 or 1u)");
    Serial.println("#");
    Serial.println("#    setup frameset <exposure> <interval> <nframes> [<clk_period> <sh_width> <sh_offset> <icg_width> <icg>_offset> [nframes [nframesets]]] (in secs)");
    Serial.println("#");
    Serial.println("#          defaults to 100u 5E-3 10 0.5u 1u 0.6u 2.6u 0.5u");
    Serial.println("#");
    Serial.println("#          Generally this command will be followed by \"start frameset\" or \"setup trigger\"");
    Serial.println("#");
    Serial.println("#      start frameset - launches a single frameset with readout");
    Serial.println("#      stop frameset  - stops ongoing frameset");
    Serial.println("#");
    Serial.println("#      arm frameset   - re-arm for the next frameset, see set nframes and nframesets");
    Serial.println("#      init frameset  - re-arms and sets frame counter to 0");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s)|| strMatch(key,"timer",&s)) {
    Serial.println("#  4) Timer setup, clocks pulses, implemented in the flexpwm, rounded to multiple of 5,10 or 20 msecs depending on readout time");
    Serial.println("#");
    Serial.println("#    setup timer <exposure> [<nframes> [<offset>]] - nframes defaults to previous setup, offset to 0");
    Serial.println("#");
    Serial.println("#          Generally this command will be followed by \"start timer\" or \"setup trigger\"");
    Serial.println("#");
    Serial.println("#      start timer - launches the timer");
    Serial.println("#      stop timer  - stops the timer");
    Serial.println("#");
  }
  if (all || strMatch(key,"lccd",&s)|| strMatch(key,"trigger",&s)) {
    Serial.println("#  5) External trigger setup - triggers pulse, timer or frameset per setup (see section 1 above)");
    Serial.println("#");
    Serial.println("#    setup trigger [ncounts]");
    Serial.println("#");
    Serial.println("#          Generally this command will be followed by \"start trigger\"");
    Serial.println("#");
    Serial.println("#      start trigger - arms the interrupt");
    Serial.println("#      stop trigger  - disables the trigger");
    Serial.println("#");
    Serial.println("#    configure trigger [args ....] ");
    Serial.println("#");
    Serial.println("#      args:  pin <n>,  pullup | pulldown | nopull,  rising | falling | change");
    Serial.println("#");
  }
  if (all) {
    Serial.println("#  More commands");
    Serial.println("#");
    Serial.println("#       set sync on | off - enable or disable sync pin output");
    Serial.println("#");
    Serial.println("#       set frame counts nframes [nframesets]");
    Serial.println("#       set frame counts nframesets");
    Serial.println("#");
    Serial.println("#       print pulse counters");
    Serial.println("#");
    Serial.println("#    System timer    - Clocks the pulse start function using standard timer calls (see setup pulse)");
    Serial.println("#");
    Serial.println("#      setup clock <period> [n] - defaults to number required by the tcd1304 setup");
    Serial.println("#");
    Serial.println("#        start clock");
    Serial.println("#        stop clock");
    Serial.println("#");
    Serial.println("#    This is the obsoleted frameset setup:");
    Serial.println("#");
    Serial.println("#      setup old frameset <clkbase (64|128)> <exposure> <pulsewidth> <frame interval> <nframes> [add] (in secs)");
    Serial.println("#");
    Serial.println("#        start old tcd1304");
    Serial.println("#        stop old tcd1304");
    Serial.println("#");
    Serial.println("#  Low level TCD1304 submodule setup, refer to flexpwm section of the processor manual");
    Serial.println("#");
    Serial.println("#    setup submodule <name> [arg val ...] [commands ....]");
    Serial.println("#");
    Serial.println("#      names:   clk sh icg cnvst timer");
    Serial.println("#");
    Serial.println("#      args:    prescale <n>, period <n>, onA|B <n>, offA|B <n>, [no]invertA|B, ctrl2 <n> ");
    Serial.println("#               master, slave, sync, print, list, check, load");
    Serial.println("#");
    Serial.println("#  Low level TCD1304 FlexPWM Commands, refer to flexpwm section of the processor manual");
    Serial.println("#");
    Serial.println("#     All of the submodules:");
    Serial.println("#");
    Serial.println("#       flexpwm dump  - dump all of the flexpwm registers");
    Serial.println("#");
    Serial.println("#       flexpwm stop  - stop all of the flexpwm submodules");
    Serial.println("#");
    Serial.println("#     Single or group commands (submodule 0-3 or mask  0x1 - 0xF");
    Serial.println("#");
    Serial.println("#       flexpwm <submodule|mask> clear load ok  (cldok in mcu reference manual)");
    Serial.println("#       flexpwm <submodule|mask> set load ok    (ldok in mcu ref manual)");
    Serial.println("#");
    Serial.println("#       flexpwm <submodule|mask> clear run    - stops the clock");
    Serial.println("#       flexpwm <submodule|mask> set run      - starts the clock");
    Serial.println("#");
    Serial.println("#     Single submodule commands");
    Serial.println("#");
    Serial.println("#       flexpwm <submodule> set clock master  - clock runs independently");
    Serial.println("#       flexpwm <submodule> set clock slave   - clock syncs and starts with 0");
    Serial.println("#       flexpwm <submodule> set clock sync    - clock syncs with 0, starts independently");
    Serial.println("#");
    Serial.println("#       flexpwm <submodule> force  -  force counter to reload to init");
    Serial.println("#");
    Serial.println("#     Single submodule counters");
    Serial.println("#");
    Serial.println("#       flexpwm <submodule> set prescale <value>  - prescale, 0 to 7 for divider 1<<p");
    Serial.println("#       flexpwm <submodule> set init <value>      - set counter init register to value (16 bits)");
    Serial.println("#       flexpwm <submodule> set val<n> <value>    - set comparison register 0 to 5, to value");
    Serial.println("#");
    Serial.println("#  Processor reference manual:");
    Serial.println("#");
    Serial.println("#       i.MX RT1060 Processor Reference Manual");
    Serial.println("#       https://www.nxp.com/products/i.MX-RT1060");
    Serial.println("#");
    Serial.println("#       Chapter 55 Enhanced Flex Pulse Width Modulator (eFlexPWM) page 3091");
    Serial.println("#");
    Serial.println("#  Pin controls - <pin> = sync | busy trigger | <n>");
    Serial.println("#");
    Serial.println("#     read pin <pin>");
    Serial.println("#     set pin <pin> hi|low|input|pullup|pulldown|output");
    Serial.println("#     pulse pin  <pin>  [usecs]");
    Serial.println("#     toggle pin <pin>");
    Serial.println("#");  
    Serial.println("#Preconfigured pins");
    Serial.print("#  Trigger(input)"); Serial.print(triggerPin);
    Serial.print("  Busy "); Serial.print(busyPin); Serial.print("  Sync "); Serial.print(syncPin);
#ifdef CONTROLLER_OCPIN
    Serial.print("  ~OverCurrent "); Serial.print(CONTROLLER_OCPIN);
#endif
    Serial.println("");
    Serial.println("#");  
  }
  if (all || strMatch(key,"help",&s)) {
    Serial.println("#Help subcommands");
    Serial.println("#");
    Serial.println("#  help [all | report | coefficients | data | microcontroller | adcs");
    Serial.println("#           lccd | pulse | frameset | trigger | help]");
    Serial.println("#");
  }
}


/* ===================================================================
   The setup routine runs once when you press reset:
*/

void setup( ) {

  // Has overcurrent pint
#ifdef CONTROLLER_OCPIN
  pinMode(CONTROLLER_OCPIN, INPUT);
#endif

  tcd1304device.setup_digital_pins( );

  // SPI setup
  SPI.begin( );
  SPI.beginTransaction(spi_settings);

  // ------------------------------
  pinMode(analogPin, INPUT);
  //  pinMode(analogPin, INPUT_PULLUP);
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
  adc->adc0->setAveraging(1);                 // set number of averages
  adc->adc0->setResolution(12);               // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  adc->adc0->wait_for_cal( );
  
  adc->adc0->singleMode( );              // need this for the fast read

  // ------------------------------
  Serial.begin(9600);
  delay(100);

  // Patent pending and copyright notice displayed at startup
  Serial.println(versionstr);
  Serial.println(authorstr);
  Serial.println(basenamef(srcfilestr));

#ifdef DIAGNOSTICS_CPU 
  Serial.print("F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz."); 
  //Serial.print("F_BUS: "); Serial.print(F_BUS/1e6);  Serial.println(" MHz."); 
  Serial.print("ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
#endif

}

// the loop routine runs over and over again forever:
void loop( ) {

  uint16_t nlen = 0;
  char *pc;
  //char **pc1, *pc2;
  char c;

  uint8_t u8tmp = 0;
  unsigned int utemp = 0;
  unsigned int utemp2 = 0;

  /* ---------------------------------------------------
     Read serial input until endof line or ctl character
   */
  while (Serial.available( )) {

    c = Serial.read( );

    if (c) {

      // break at ctl-character or semi-colon
      if (iscntrl(c) || c == ';') {
        nlen = nrcvbuf;
        rcvbuffer[nrcvbuf] = 0;
        nrcvbuf = 0;
        break;
      }

      // skip leading spaces
      else if (nrcvbuf || !isspace(c)) {
        rcvbuffer[nrcvbuf++] = c;        
      }

      if (nrcvbuf >= RCVLEN) {
        Serial.println((char *)"Error: buffer overflow");
        nrcvbuf = 0;
      }
    }
    
#ifdef DIAGNOSTICS_RCV
    Serial.print('#');
    Serial.println(rcvbuffer);
#endif
  }

  /* ====================================================================
     Frame Ring buffer processing
  */
  send_frames();
  
  /* ====================================================================
   * Command processing
   */

  if (nlen > 0) {
    
    //blink( );

    for (int n = 0; (n < RCVLEN) && rcvbuffer[n] ; n++) {
      rcvbuffer[n] = tolower(rcvbuffer[n]);
    }
    pc = rcvbuffer;

    Serial.println(pc);
     
    /* -------------------------------------------------------------
       Firmware version identification
    */
    if (strMatch(rcvbuffer, "version", &pc)) {
      Serial.println(versionstr);
      Serial.println(authorstr);
      Serial.println(basenamef(srcfilestr));
    }

    else if (strMatch(rcvbuffer, "configuration", &pc)) {

      sprintf(sndbuffer, "PIXELS %u DARK %u BITS %u VFS %f",
	       NPIXELS, NDARK, NBITS, VFS);
      Serial.print(sndbuffer);

      Serial.print(" SENSOR ");
      Serial.println(sensorstr);
      
#ifdef DIAGNOSTICS_CPU
      Serial.print("F_CPU: ");
      Serial.print(F_CPU/1e6);
      Serial.println(" MHz."); 

      Serial.print("ADC_F_BUS: ");
      Serial.print(ADC_F_BUS/1e6);
      Serial.println(" MHz.");
#endif
      
    }
    else if (strMatch(rcvbuffer, "reboot", &pc)) {
      _reboot_Teensyduino_( );
    }

    else if (strMatch(rcvbuffer, "set diagnostics", &pc)) {
      diagnostics = true;
      Serial.println("tcd1304 set diagnostics to true");
    }

    else if (strMatch(rcvbuffer, "clear diagnostics", &pc)) {
      diagnostics = false;
      Serial.println("tcd1304 set diagnostics to false");
    }

    else if (strMatch(rcvbuffer, "help", &pc)) {
      if (pc && *pc) {
	while (isspace(*pc)) pc++;
      }
      Serial.printf("help command parameter %s\n", pc);
      help(pc);
    }

    else if ((strMatch(rcvbuffer, "stop", &pc) && !nextWord(pc)) ||
              strMatch(rcvbuffer, "stop all", &pc)) {
      tcd1304device.stop_all( );
    }
    
    /* ====================================================================
       High level commands
     */
    
    else if (((strMatch(rcvbuffer,"read", &pc) && (pc=nextWord(pc)) && isdigit(*pc)))
              || strMatch(rcvbuffer,"read frames", &pc)
              || strMatch(rcvbuffer,"read frame", &pc)) {
      
      uint nframes = 1;
      float exposure = 0.;
      float frameinterval = 0;
      bool start = true;

      char *pc1 = pc;

      if (strUint(pc,&nframes, &pc1)
          && strFlt(pc1,&exposure, &pc1)
          && strFlt(pc1,&frameinterval, &pc)) {

        if (strMatch(pc,"nostart",&pc)) {
          start = false;
        }

        tcd1304device.read(nframes, exposure, frameinterval, bufferp,
                           frame_callback, 0, 0, send_header, start);
        
      }
          
      else if (strUint(pc,&nframes,&pc1)
               && strFlt(pc1,&exposure,&pc)) {

        if (strMatch(pc,"nostart",&pc)) {
          start = false;
        }
	
        tcd1304device.read(nframes,exposure,bufferp,
                           frame_callback, 0, 0, send_header, start);
      }
           
      else if (strFlt(pc,&exposure, &pc)) {

        if (strMatch(pc,"nostart",&pc)) {
          start = false;
        }
        
        tcd1304device.read(nframes,exposure,bufferp,
                           frame_callback, 0, 0, send_header, start);
      }

      else {
        Serial.println("Error: read parameters not recognized, see help read");
      }
           
    }
      
    else if (strMatch(rcvbuffer,"start read", &pc)) {
       tcd1304device.start_read( );
    }
    
    else if (strMatch(rcvbuffer,"wait read", &pc)) {
      float timeout = 0.;
      float timestep = 0.01;
      bool verbose = false;

      if (strFlt(pc,&timeout, &pc)) {
        strFlt(pc,&timestep, &pc);
      }

      if (strMatch(pc, "verbose", &pc)) {
        verbose = true;
      }
      
      if(tcd1304device.wait_read(timeout,timestep,verbose)) {
        Serial.println("wait read success");
      }
      else {
        Serial.println("wait read not success");
      }
    }

    // ---------------------------------------------------------------------------------

    else if (((strMatch(rcvbuffer,"trigger", &pc) && (pc=nextWord(pc)) && isdigit(*pc)))
              || strMatch(rcvbuffer,"trigger read", &pc)) {
      
      uint ntriggers = 1;
      uint nframes = 1;
      float exposure = 0.;
      float frameinterval = 0;

      bool start = true;

      char *pc1 = pc;

      if (strUint(pc,&ntriggers, &pc1)
          && strUint(pc1,&nframes, &pc1)
          && strFlt(pc1,&exposure, &pc1)
          && strFlt(pc1,&frameinterval, &pc)) {

        if (strMatch(pc,"nostart", &pc)) {
          start = false;
        }

        Serial.printf("triggered read with m %d n %d exposure %.6f interval %.6f start %d\n",
                      ntriggers, nframes, exposure, frameinterval, start);
        
        tcd1304device.triggered_read(ntriggers, nframes, exposure, frameinterval, bufferp,
                                     frame_callback, 0, 0, send_header, start);
      }
          
      else if (strUint(pc,&ntriggers, &pc1)
               && strUint(pc1,&nframes, &pc1)
               && strFlt(pc1,&exposure, &pc)) {

        if (strMatch(pc,"nostart", &pc)) {
          start = false;
        }
        
        Serial.printf("triggered read with m %d n %d exposure %.6f start %d\n",
                      ntriggers, nframes, exposure, start);
        
        tcd1304device.triggered_read(ntriggers, nframes, exposure, bufferp,
                                     frame_callback, 0, 0, send_header, start);
      }
      
      else if (strUint(pc,&nframes, &pc1)
               && strFlt(pc1,&exposure, &pc1)
               && strFlt(pc1,&frameinterval, &pc)) {

        if (strMatch(pc,"nostart", &pc)) {
          start = false;
        }
        
        Serial.printf("triggered read with n %d exposure %.6f interval %.6f start %d\n",
                      nframes, exposure, frameinterval, start);
        
        tcd1304device.triggered_read(1, nframes, exposure, frameinterval, bufferp,
                                     frame_callback, 0, 0, send_header, start);
      }
      
      else if (strUint(pc,&nframes, &pc1)
               && strFlt(pc1,&exposure, &pc)) {

        if (strMatch(pc,"nostart",&pc)) {
          start = false;
        }
        
        Serial.printf("triggered read with n %d exposure %.6f start %d\n",
                      nframes, exposure, start);
        
        tcd1304device.triggered_read(1, nframes, exposure, bufferp,
                                     frame_callback, 0, 0, send_header, start);
      }

      else {
        Serial.println("Error: trigger parameters not recognized, see help trigger.");
      }
      
    }
    
    else if (strMatch(rcvbuffer,"gate frame", &pc)) {
      uint nframes = 1;
      
      strUint(pc,&nframes,&pc);
      
      Serial.println("Error: not yet implemented");

    }
    
    else if (strMatch(rcvbuffer,"wait trigger", &pc)) {
      float timeout = 0.;
      float timestep = 0.01;
      bool verbose = false;

      if (strFlt(pc,&timeout, &pc)) {
        strFlt(pc,&timestep, &pc);
      }

      if (strMatch(pc, "verbose",&pc)) {
        verbose = true;
      }
      
      if(tcd1304device.wait_triggered_read(timeout,timestep,verbose)) {
        Serial.println("wait read success");
      }
      else {
        Serial.println("wait read not success");
      }
    }
    
    /* ====================================================================
       Clock commands
     */

    else if (strMatch(rcvbuffer,"setup clock", &pc)) {
      
      float secs = 0;
      unsigned int ntemp = 0;

      // must specify time interval
      if (strFlt(pc,&secs, &pc)) {
        strUint(pc,&ntemp,&pc);
      
        clock_setup(secs,ntemp);
      }
      else {
        Serial.print("Error: need clock secs [nsets]; ");
        Serial.println(pc);
      }

    }
    
    else if (strMatch(rcvbuffer, "start clock",&pc)) {
      //send_header("CLOCKED");
      send_header();
      clock_start( );
    }

    else if (strMatch(rcvbuffer, "stop clock", &pc)) {
      clock_stop( );
      Serial.println("tcd1304: stopped clock");
    }

    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "setup tcd1304 timer", &pc) ||
             strMatch(rcvbuffer, "setup timer", &pc)) {

      float exposure = 0.1;
      float exposure_offset = 0.0;
      unsigned int nframes = 0;

      if (tcd1304device.mode==NOTCONFIGURED) {
        Serial.println("Error: not configured, use \"setup pulse\" first");
      }
      else if (strFlt(pc,&exposure, &pc)) {
        if (strUint(pc,&nframes, &pc)) {
	  strFlt(pc,&exposure_offset,&pc);
	}
	tcd1304device.setup_timer(exposure,exposure_offset,nframes);
      }
      else {
        Serial.println("Error: not able to parse setup timer <exposure> [N [offset]] (secs)");
      }

    }
    
    else if (strMatch(rcvbuffer, "start tcd1304 timer", &pc)||
             strMatch(rcvbuffer, "start timer", &pc)) {
      //send_header("TIMER");
      send_header();
      tcd1304device.timer_start( );
    }

    else if (strMatch(rcvbuffer, "stop tcd1304 timer", &pc)||
             strMatch(rcvbuffer, "stop timer", &pc)) {
      tcd1304device.timer_stop( );
    }
    
    /* ========================================================
       carefull about this, should not do it while trigger is started
     */
    else if (strMatch(rcvbuffer, "configure trigger", &pc)) {

      if (*pc) {
        if (tcd1304device.trigger_attached) {
          Serial.println("Error: set trigger but already attached, stop trigger first");
        }

        // Here is the parsing for the trigger setup
        else if (!strTrigger(pc,&pc)) {
          Serial.println("Error: not able to parse trigger configuration commands");
        }
      }

      printTriggerSetup( );

    }

    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "setup trigger",&pc)) {

      unsigned int ncounts = 0;

      // optional number of triggers
      if (tcd1304device.mode==NOTCONFIGURED) {
        Serial.println("Error: not configured, use setup pulse, frameset or timer first");
      }
      
      else if (!(pc=nextWord(pc)) || strUint(pc,&ncounts,&pc)) {
        tcd1304device.setup_triggers(ncounts);
      }
      else {
        Serial.println("Error: not able to parse setup trigger counts");
      }

    }
    
    else if (strMatch(rcvbuffer, "start trigger", &pc)) {
      //send_header("TRIGGERED");
      send_header();
      tcd1304device.start_triggers( );
    }
    
    else if (strMatch(rcvbuffer, "stop trigger", &pc)) {
      tcd1304device.stop_triggers( );
    }
    
    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "set extra cnvst delay", &pc)) {
      strUint16(pc,&tcd1304device.cnvst_extra_delay_counts,&pc);
      Serial.print("flexpwm extra cnvst delay counts ");
      Serial.println(tcd1304device.cnvst_extra_delay_counts);
    }
    
    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "set frame counts", &pc)) {
      if (strUint(pc,&tcd1304device.frame_counts, &pc)) {
        strUint(pc,&tcd1304device.frameset_counts,&pc);
      }
    }
      
    else if (strMatch(rcvbuffer, "set frameset counts", &pc)) {
      strUint(pc,&tcd1304device.frameset_counts,&pc);
    }
    
    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "print counters", &pc)) {
      tcd1304device.print_counters( );
    }

    /* ========================================================
     */
    else if (strMatch(rcvbuffer, "configure clearing pulses", &pc)) {

      strUint(pc,&tcd1304device.sh_clearing_counts,&pc);

      Serial.print("clearing pulses ");
      Serial.println(tcd1304device.sh_clearing_counts);
    }
    
    else if (strMatch(rcvbuffer, "setup tcd1304 pulse", &pc) ||
             strMatch(rcvbuffer, "setup pulse", &pc)) {

      float clk_secs = 0.5E-6;
      float sh_secs = 1.0E-6;
      float sh_offset_secs = 0.6E-6;
      float icg_secs = 2.6E-6;
      float icg_offset_secs = 0.5E-6;

      // if we have any parameters, we have to have all of them
      if (!(pc=nextWord(pc)) ||
          (
           strFlt(pc,&clk_secs, &pc) &&
           strFlt(pc,&sh_secs, &pc) &&
           strFlt(pc,&sh_offset_secs, &pc) &&
           strFlt(pc,&icg_secs, &pc) &&
           strFlt(pc,&icg_offset_secs, &pc)
          )
         )
        {
          // clock specified as frequency
          if (clk_secs >= 0.8E6) clk_secs = 1./clk_secs;
        
          serialPrintlnf("clk %.6gs sh %.6gs +%.6gs icg %.6gs +%.6gs",
                         clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs);
          
          if (tcd1304device.setup_pulse(clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs,
                                        bufferp, NREADOUT, frame_callback)) {
	    
            // provide override for default single frame
            if (strUint(pc,&tcd1304device.frame_counts, &pc)) {
              strUint(pc,&tcd1304device.frameset_counts,&pc);
            }
            
          }
          
        }
      else {
        Serial.println("Error: pulse parameters not recognized, see help pulse.");
      }
    }
      
    else if (strMatch(rcvbuffer, "init pulse", &pc)) {
      tcd1304device.pulse_init_frames( );
    }
      
    else if (strMatch(rcvbuffer, "arm pulse", &pc)) {
      tcd1304device.pulse_arm( );
    }
    
    else if (strMatch(rcvbuffer, "start pulse", &pc)) {
      tcd1304device.pulse_start( );
    }

    else if (strMatch(rcvbuffer, "stop pulse", &pc)) {
      tcd1304device.flexpwm_stop( );
    }
    
    /* ========================================================
     */
      // These codes need debugging in the library, intervals dont look right on the scope
      
    else if (strMatch(rcvbuffer, "setup tcd1304 frameset", &pc) ||
             strMatch(rcvbuffer, "setup frameset", &pc)) {

      float clk_secs = 0.5E-6;
      float sh_secs = 1.0E-6;
      float sh_offset_secs = 0.6E-6;
      float icg_secs = 2.6E-6;
      float icg_offset_secs = 0.5E-6;

      /*
      float clk_secs = 0.640E-6;
      float sh_secs = 1.28E-6;
      float sh_offset_secs = 0.64E-6;
      float icg_secs = 2.56E-6;
      float icg_offset_secs = 0.15E-6;
      */
      
      float exposure_secs = 100.E-6;
      float frame_interval_secs = 5.E-3;
      unsigned int nframes = 10;

      // if we have any parameters, we have to have all of them
      if ((!(pc=nextWord(pc)) || (strFlt(pc,&exposure_secs, &pc) &&
                                  strFlt(pc,&frame_interval_secs, &pc) &&
                                  strUint(pc,&nframes, &pc))
          ) &&
          (!(pc=nextWord(pc)) || (strFlt(pc,&clk_secs, &pc) &&
                                  strFlt(pc,&sh_secs, &pc) &&
                                  strFlt(pc,&sh_offset_secs, &pc) &&
                                  strFlt(pc,&icg_secs, &pc) &&
                                  strFlt(pc,&icg_offset_secs, &pc))
          )
         )
        {
          // clock specified as frequency
          if (clk_secs >= 0.8E6) clk_secs = 1./clk_secs;
        
          serialPrintlnf("clk %.6gs sh %.6gs +%.6gs icg %.6gs +%.6gs %.6gs %.6gs %d",
                         clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs,
                         exposure_secs, frame_interval_secs, nframes);
          
          if (tcd1304device.setup_frameset(clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs,
                                           exposure_secs, frame_interval_secs, nframes,
                                           bufferp, NREADOUT, frame_callback)) {

            // provide override for default single frameset
            strUint(pc,&tcd1304device.frameset_counts,&pc);
          }
          
        }
      else {
        Serial.println("Error: frameset parameters not recognized, see help frameset.");
      }
    }
      
    else if (strMatch(rcvbuffer, "start frameset", &pc)) {
      tcd1304device.frameset_start( );
    }

    else if (strMatch(rcvbuffer, "stop frameset", &pc)) {
      tcd1304device.flexpwm_stop( );
    }
 
    /* ========================================================
       access the pins from the flexpwm as digital i/o pins
     */
    else if (strMatch(rcvbuffer, "close", &pc)) {

      tcd1304device.close( );
      Serial.println("tcd1304 device closed, pins ready for digital i/o");
    }
    
    else if (strMatch(rcvbuffer, "set sync enable", &pc)||
             strMatch(rcvbuffer, "set sync on", &pc)) {
      tcd1304device.sync_enabled = true;;
      Serial.println("tcd1304 sync enabled");
    }

    else if (strMatch(rcvbuffer, "set sync not enable", &pc)||
             strMatch(rcvbuffer, "set sync off", &pc)) {
      tcd1304device.sync_enabled = false;;
      Serial.println("tcd1304 sync disenabled");
    }

    // --------------------------------------------------------------
    // intermediate level access
    else if (strMatch(rcvbuffer, "setup tcd1304 submodule", &pc) ||
             strMatch(rcvbuffer, "setup submodule", &pc)) {

      strSubModule(pc);
      
    }
    // --------------------------------------------------------------

    // for manual debugging work
    else if (strMatch(rcvbuffer, "flexpwm", &pc)) {
      uint8_t submod = 0;
      uint8_t mask = 0;
      uint8_t u8;
      uint16_t val = 0;

      while (*pc && isspace(*pc)) pc++;
      
      if (!pc) {
        Serial.println("Error flexpwm needs a command or submodule number of hex mask 0x");        
      }
      else if(strMatch(pc,"dump", &pc)) {
        Serial.println("#======================================");
        Serial.println("#TCD1304 flexPWM");
        tcd1304device.register_dump( );
        Serial.println("#======================================");
        Serial.println("#Timer flexPWM");
        tcd1304device.register_dump(tcd1304device.timerflexpwm,tcd1304device.timer.mask);
      }
      
      else if (strMatch(rcvbuffer, "stop", &pc)) {
        tcd1304device.stop_all( );
      }
      
      else if (strMatch(rcvbuffer, "close", &pc)) {
        tcd1304device.close( );
      }
      
      else if (strUint8(pc,&u8, &pc)) {
        Serial.print("flexpwm parsed got "); Serial.println(u8);
        if ((pc[0]=='0') && (pc[1]=='x' || pc[1]=='x')) {
          mask = u8;
          while((submod<4)&&(mask!=(1<<submod)))submod++;  
        }
        else {
          submod = u8;
          mask = 1<<submod;            
        }

	while (pc&&*pc&&isspace(*pc))pc++;

        // these can accept a mask for one or more submodules
        if (!mask || (mask>0xF)) {
          Serial.println("Error: need mask in range 0x1 to 0xF or submodule 1 to 3");
        }
        
        else if (strMatch(pc,"clear load ok", &pc)) {
          Serial.print("clear ldok "); Serial.println(mask,HEX);
          tcd1304device.clear_ldok((mask));
        }
        
        else if (strMatch(pc,"set load ok", &pc)) {
          Serial.print("set ldok "); Serial.println(mask,HEX);
          tcd1304device.set_ldok((mask));
        }
        else if (strMatch(pc,"clear run", &pc)) {
          Serial.print("clear run "); Serial.println(mask,HEX);
          tcd1304device.clear_run((mask));
        }

        else if (strMatch(pc,"set run", &pc)) {
          Serial.print("set run "); Serial.println(mask,HEX);
          tcd1304device.set_run(mask);
        }

        // after this the options all require a specific submodule
        else if (submod>3) {
          Serial.print("Error: need submodule 0 to 3 for ");
          Serial.println(pc);
        }
        
        else if (strMatch(pc,"set clock master", &pc)) {
          Serial.print("set clock master "); Serial.println(submod);
          tcd1304device.set_clock_master(submod);
        }
        else if (strMatch(pc,"set clock slave", &pc)) {
          Serial.print("set clock slave "); Serial.println(submod);
          tcd1304device.set_clock_slave(submod);
        }
        else if (strMatch(pc,"set clock sync", &pc)) {
          Serial.print("set clock sync "); Serial.println(submod);
          tcd1304device.set_clock_sync(submod);
        }
        
        else if (strMatch(pc,"force", &pc)) {
          Serial.print("set force "); Serial.println(submod);
          tcd1304device.force(submod);
        }

        else if (strMatch(pc,"set init", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d init %d", submod, val);
            tcd1304device.set_init(submod,val);
          }
        }

        else if (strMatch(pc,"set val0", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val0 %d", submod, val);
            tcd1304device.set_val0(submod,val);
          }
        }

        else if (strMatch(pc,"set val1", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val1 %d", submod, val);
            tcd1304device.set_val1(submod,val);
          }
        }
        
        else if (strMatch(pc,"set val2", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val2 %d", submod, val);
            tcd1304device.set_val2(submod,val);
          }
        }

        else if (strMatch(pc,"set val3", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val3 %d", submod, val);
            tcd1304device.set_val3(submod,val);
          }
        }
        
        else if (strMatch(pc,"set val4", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val4 %d", submod, val);
            tcd1304device.set_val4(submod,val);
          }
        }
        
        else if (strMatch(pc,"set val5", &pc)) {
          if (strUint16(pc,&val, &pc)) {
            serialPrintlnf("set submodule %d val5 %d", submod, val);
            tcd1304device.set_val5(submod,val);
          }
        }
        else {
          Serial.println("Error: need: flexpwm  <dump|stop|close>||<submod|mask> commmand (see help)");
        }
      }
      else {        
        Serial.print("Error: failed to parse ");
        Serial.println(pc);  
      }
    }
    
    /* -----------------------------------------------------------
       Sync pin
    */
    else if (strMatch(rcvbuffer, "set sync off", &pc) ||
	      strMatch(rcvbuffer, "clear sync", &pc)) {
      Serial.println("setting sync off");
      Serial.println("not implemented yet");      
    }
    
    else if (strMatch(rcvbuffer, "set sync exposure", &pc)) {
      Serial.println("setting sync shutter");
      Serial.println("not implemented yet");
    }
    
    else if (strMatch(rcvbuffer, "set sync shutter", &pc)) {
      Serial.println("setting sync shutter");
      Serial.println("not implemented yet");
    }
    
    else if (strMatch(rcvbuffer, "set sync start", &pc)) {
      Serial.println("setting sync start");
      Serial.println("not implemented yet");
    }
    
    else if (strMatch(rcvbuffer, "set sync holdoff", &pc)) {
      strUint(pc, &utemp, &pc);
      Serial.print("setting sync holdoff ");
      Serial.print(utemp);
      Serial.print(" usecs ");
      Serial.println("not implemented here - see timer commands");
    }

    /* -----------------------------------------------------------
       Set any pin
     */
    else if (strMatch(rcvbuffer, "read",&pc) && strPin(pc,&u8tmp,&pc)) {
      utemp = digitalRead(utemp);
      serialPrintlnf("Pin %d read %d", u8tmp, utemp);
    }   
    
    else if (strMatch(rcvbuffer, "set",&pc)&&strPin(pc,&u8tmp,&pc)) {

      if (strMatch(pc, "hi", &pc)) {
        digitalWriteFast(u8tmp, HIGH);
        serialPrintlnf("Pin %d set HIGH", u8tmp);
      } 
    
      else if (strMatch(pc, "lo", &pc)) {
        digitalWriteFast(u8tmp, LOW);
        serialPrintlnf("Pin %d set LOW", utemp);
      }

      else if (strMatch(pc, "output", &pc)) {
        pinMode(u8tmp, OUTPUT);
        serialPrintlnf("Pin %d set OUTPUT", utemp);
      }
      
      else if (strMatch(pc, "input", &pc)) {
        pinMode(u8tmp, INPUT);
        serialPrintlnf("Pin %d set INPUT", utemp);
      }      

      else if (strMatch(pc, "nopull", &pc)) {
        pinMode(u8tmp, INPUT);
        serialPrintlnf("Pin %d set INPUT", utemp);
      }      

      else if (strMatch(pc, "pullup", &pc)) {
        pinMode(u8tmp, INPUT_PULLUP);
        serialPrintlnf("Pin %d set PULLUP", utemp);
      }
      
      else if (strMatch(pc, "pulldown", &pc)) {
        pinMode(u8tmp, INPUT_PULLDOWN);
        serialPrintlnf("Pin %d set PULLDOWN", utemp);
      }
      
      else {
        Serial.println("Error: need: set <pin> <hi|lo|output|input|nopullup|pullup>");
      }      
    }
    
    else if (strMatch(rcvbuffer, "pulse pin", &pc)) {

      if (strPin(pc,&u8tmp, &pc)) {
      
        utemp2 = 1;
        strUint(pc, &utemp2, &pc);

        pulsePin(u8tmp, utemp2);     
        serialPrintlnf("pin %d pulsed %dusecs", u8tmp, utemp2);
      }
      else {
        Serial.println("Error: pulse <pin> [usecs] failed to parse pin");
      }
    }

    else if (strMatch(rcvbuffer, "toggle pin", &pc)) {

      if (strPin(pc,&u8tmp, &pc)) {
        digitalToggleFast(utemp);
        serialPrintlnf("pin %d toggled", utemp);
      }
      else {
        Serial.println("Error: toggle <pin> failed to parse pin");
      }
    }
    
    // the sync and busy pins have their own bookkeeping
    else if (strMatch(rcvbuffer, "toggle busy", &pc)) {
      tcd1304device.toggle_busypin( );      
    }
    else if (strMatch(rcvbuffer, "clear busy", &pc)) {
      tcd1304device.clear_busypin( );      
    }
    
    else if (strMatch(rcvbuffer, "toggle sync", &pc)) {
      tcd1304device.toggle_syncpin( );      
    }
    else if (strMatch(rcvbuffer, "clear sync", &pc)) {
      tcd1304device.clear_syncpin( );      
    }
    
    /* -----------------------------------------------------------
       Synchronous Data transmit,test and buffer managament
    */
    
    else if (strMatch(rcvbuffer, "send crc", &pc) ||
              strMatch(rcvbuffer, "crc", &pc)) {    
      sendDataCRC(bufferp);
      
    }
    
    else if (strMatch(rcvbuffer, "send sum", &pc) ||
             strMatch(rcvbuffer, "sum", &pc)) {    
      sendDataCRC(bufferp);
      
    }
    else if (strMatch(rcvbuffer, "send", &pc)) {    
      sendData(bufferp);
    }
        
    else if (strMatch(rcvbuffer, "send test", &pc)) {    
      sendTestData(bufferp);
    }

    /* =================================================================
       Over current pin
    */

#ifdef CONTROLLER_OCPIN
    else if (strMatch(rcvbuffer, "attach ocpin", &pc)) {
      ocpinAttach( );      
    }
    
    else if (strMatch(rcvbuffer, "deattach ocpin", &pc)) {
      ocpinDetach( );      
    }

    else if (strMatch(rcvbuffer, "ocpin", &pc)) {
      ocpinRead( );      
    }
#endif

    
    /* =================================================================
       Pins
    */    

    else if (strMatch(rcvbuffer, "pins", &pc)) {

      Serial.println("#Pins");
      serialPrintf("# Trigger(input) %d",
		   tcd1304device.trigger_pin);

      
      serialPrintf(" Busy %d Sync %d",
		   BUSY_PIN,
		   SYNC_PIN);
      
      serialPrintf(" TCD1304 clock %d icg %d sh %d",
		   tcd1304device.clk.pinA,
		   tcd1304device.icg.pinA,
		   tcd1304device.sh.pinA);

      serialPrintf(" SPI clock %d sdi %d sdo %d cnvst %d", CLKPin, SDIPin, SDOPin, CNVSTPin);
     
#ifdef CONTROLER_R4_OCPIN
      Serial.print("  ~OverCurrent ");
      Serial.print(CONTROLLER_R4_OCPIN);
#endif
      Serial.println("");
      
    }
    
    /* -----------------------------------------------------------
       User store/print coefficients
    */
    else if (strMatch(rcvbuffer, "store coefficients", &pc) || strMatch(rcvbuffer, "set coefficients", &pc)) {
      float vals[EEPROM_NCOEFFS] = {0};
      if (strFlts(pc, vals, EEPROM_NCOEFFS, &pc)) {
	storeCoefficients(vals);
      }      
      printCoefficients( );      
    }

    else if (strMatch(rcvbuffer, "coefficients", &pc)) {
      printCoefficients( );      
    }
    
    else if (strMatch(rcvbuffer, "erase coefficients", &pc)) {
      eraseCoefficients( );
    }
    
    /* -----------------------------------------------------------
       User store/print identifier
    */

    else if (strMatch(rcvbuffer, "store identifier", &pc) || strMatch(rcvbuffer, "set identifier", &pc)) {
      while(*pc && isspace(*pc)) pc++;
      storeIdentifier(pc);      
      printIdentifier( );      
    }
    
    else if (strMatch(rcvbuffer, "identifier", &pc)) {
      printIdentifier( );      
    }
    
    else if (strMatch(rcvbuffer, "erase identifier", &pc)) {
      eraseIdentifier( );
    }
    
    /* -----------------------------------------------------------
       User store/print units
    */

    else if (strMatch(rcvbuffer, "store units", &pc) || strMatch(rcvbuffer, "set units", &pc)) {
      while(*pc && isspace(*pc)) pc++;
      storeUnits(pc);      
      printUnits( );      
    }
    
    else if (strMatch(rcvbuffer, "units", &pc)) {
      printUnits( );      
    }
    
    else if (strMatch(rcvbuffer, "erase units", &pc)) {
      eraseUnits( );
    }
    
    /* -----------------------------------------------------------
       Data async data
     */
    else if (strMatch(rcvbuffer, "set async", &pc)) {
      data_async = true;
      Serial.println("transfer asynchronous");
    }
    
    else if (strMatch(rcvbuffer, "set synch", &pc)) {
      data_async = false;
      Serial.println("transfer synchronous");
    }

    else if (strMatch(rcvbuffer, "enable crc", &pc)) {
      Serial.println("enabled crc");
      crc_enable = true;
    }
    
    else if (strMatch(rcvbuffer, "disable crc", &pc)) {
      Serial.println("disabled crc");
      crc_enable = false;
    }

    else if (strMatch(rcvbuffer, "enable sum", &pc)) {
      Serial.println("enabled sum");
      sum_enable = true;
    }
    
    else if (strMatch(rcvbuffer, "disable sum", &pc)) {
      Serial.println("disabled sum");
      sum_enable = false;
    }
    /* -----------------------------------------------------------
       Data output format
     */
    
    else if (strMatch(rcvbuffer, "set ascii", &pc) || strMatch(rcvbuffer, "set formatted", &pc)) {
      dataformat = ASCII;
      Serial.println("format ascii");
    }

    else if (strMatch(rcvbuffer, "set binary", &pc)) {
      dataformat = BINARY;
      Serial.println("format binary");
    }
    
    else if (strMatch(rcvbuffer, "format", &pc)) {
      if (dataformat == BINARY) {
	Serial.println("format binary");
      }
      else if (dataformat == ASCII) {
	Serial.println("format ascii");
      }
      else {
	Serial.println("format unknown");
      }
    }
    
    /* -----------------------------------------------------------
       ADC reporting
    */
    else if (strMatch(rcvbuffer, "set adcs off", &pc)) {
      Serial.println("setting adc reporting off");
      adc_averages = 0;
    }
    
    else if (strMatch(rcvbuffer, "set adcs", &pc) && strUint(pc, &adc_averages, &pc)) {
      Serial.print("setting adc reporting, averaging ");
      Serial.println(adc_averages);
    }
    
    else if (strMatch(rcvbuffer, "adcs", &pc) || strMatch(rcvbuffer, "read analog inputs", &pc)) {
      utemp = 1;
      strUint(pc, &utemp, &pc);
      if (utemp) {
	Serial.println("reading ADCS");
	sendADCs(utemp);
      }
    }

    /* ------------------------------------------------------------
       Temperature reporting
     */
    else if (strMatch(rcvbuffer, "set temperature off", &pc)) {
      Serial.println("setting chip temperature reporting off");
      chipTemp_averages = 0;
    }
    
    else if (strMatch(rcvbuffer, "set temperature on", &pc)) {
      Serial.println("setting chip temperature reporting on");
      chipTemp_averages = 1;
    }
    
    else if (!chipTemp_averages && strMatch(rcvbuffer, "temperature", &pc)) {
      Serial.print("CHIPTEMPERATURE ");
      Serial.println(tempmonGetTemp( ));
    }
    
    /* -----------------------------------------------------------
       Command not recognized
    */
    else {
      Serial.print("Error: unrecognized command //");
      Serial.print(rcvbuffer);
      Serial.println("//");
    }

   // Indicate that we processed this message
    nlen = 0;
    
    Serial.println("DONE");
  }
}
