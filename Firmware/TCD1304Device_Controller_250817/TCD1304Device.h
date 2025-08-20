/* tcd1304 flexpwm library

   Author Mitchell C Nelson, PhD
   Copyright 2025

   Free for non-commercial use.

   No warranty and no representation of suitability for any purpose whatsoever

*/

#ifndef TCD1304DEVICE_H
#define TCD1304DEVICE_H

#include "Arduino.h"
#include <SPI.h>

#include "imxrt.h"

#define ALLINONEBOARD

#ifdef  ALLINONEBOARD
#include <ADC.h>
#include <ADC_util.h>
extern ADC *adc;
#define ANALOGPIN A0
#endif

//#define DEBUG

// we'll comment this out for the new boards
#define SH_STOP_IN_READ

// debug prints in setup
//#define DEBUG

#define ROUNDUP(a,b) (ceil((float)a/b)*b)
#define ROUNDTO(a,b) ((a/b)*b)
#define ROUNDTOMOD(a,b,c) (((a/b)*b)%c)

// Clock and force configurations
#define PWM_CTRL2_CLOCK_MASTER FLEXPWM_SMCTRL2_FRCEN
#define PWM_CTRL2_CLOCK_SLAVE (FLEXPWM_SMCTRL2_CLK_SEL(0x2) | (FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_FORCE_SEL(0X3)))
#define PWM_CTRL2_CLOCK_SYNC (FLEXPWM_SMCTRL2_CLK_SEL(0x2) | FLEXPWM_SMCTRL2_FRCEN)

// Flexpwm compare interrupts
#define CMPF_MASKA_ON (1<<2)
#define CMPF_MASKA_OFF (1<<3)
#define CMPF_MASKA_ON_OFF (CMPF_MASKA_ON|CMPF_MASKA_OFF)

#define CMPF_MASKB_ON (1<<4)
#define CMPF_MASKB_OFF (1<<5)
#define CMPF_MASKB_ON_OFF (CMPF_MASKB_ON|CMPF_MASKB_OFF)

//#define CLK_DEFAULT 48
#define CLK_DEFAULT 64
//#define CLK_DEFAULT 96
//#define CLK_DEFAULT 128

#define CLK_SUBMODULE 0
#define CLK_MASK 0x1
#define CLK_CHANNEL 1
#define CLK_PIN 4
#define CLK_MUXVAL 1
#define CLK_CTRL2_MASK PWM_CTRL2_CLOCK_MASTER

#define ICG_SUBMODULE 1
#define ICG_MASK 0x2
#define ICG_CHANNEL 1
#define ICG_PIN 5
#define ICG_MUXVAL 1
#define ICG_CTRL2_MASK PWM_CTRL2_CLOCK_SLAVE
#define ICG_IRQ IRQ_FLEXPWM2_1
#define ICG_CMPF_MASK (CMPF_MASKA_OFF|CMPF_MASKB_ON_OFF)

#define SH_SUBMODULE 2
#define SH_MASK 0x4
#define SH_CHANNEL 1
#define SH_PIN 6
#define SH_MUXVAL 2
#define SH_CTRL2_MASK PWM_CTRL2_CLOCK_SLAVE
#define SH_IRQ IRQ_FLEXPWM2_2
#define SH_CMPF_MASK CMPF_MASKA_ON_OFF

#define CNVST_SUBMODULE 3
#define CNVST_MASK 0x8
#define CNVST_CHANNEL 1
#define CNVST_CTRL2_MASK PWM_CTRL2_CLOCK_SYNC
#define CNVST_IRQ IRQ_FLEXPWM2_3
#define CNVST_CMPF_MASK CMPF_MASKA_OFF

// this one is for PWM4
#define TIMER_SUBMODULE 2
#define TIMER_MASK 0x4
#define TIMER_CHANNEL 2
#define TIMER_PIN 3
#define TIMER_MUXVAL 1
#define TIMER_CTRL2_MASK PWM_CTRL2_CLOCK_MASTER
#define TIMER_IRQ IRQ_FLEXPWM4_2
#define TIMER_CMPF_MASK CMPF_MASKB_ON

/* Note that we do not configure the flexPWM to use this pin (CNVST_PIN),
   It is not on the mux list for this module,
   We operate this pin using digitalwrite, from the isr.
*/
#define CNVST_PIN 10
#define SETCNVST (CORE_PIN10_PORTSET = CORE_PIN10_BITMASK)

#define CNVST_PULSE_SECS 630.E-9
//#define CNVST_OFFSET_CLOCKS 1

#define CLEARCNVST (CORE_PIN10_PORTCLEAR = CORE_PIN10_BITMASK)

#define CLK_MONITOR_PIN 3

#define SYNC_PIN 0
#define BUSY_PIN 1
#define INTERRUPT_PIN 2
//#define SPARE_PIN 3

#define SYNC_PIN_DEFAULT LOW
#define BUSY_PIN_DEFAULT HIGH

// Fast nofrills BUSY pin set, lear, flip state
#define SETBUSYPIN (CORE_PIN1_PORTSET = CORE_PIN1_BITMASK)
#define CLEARBUSYPIN (CORE_PIN1_PORTCLEAR = CORE_PIN1_BITMASK)
#define TOGGLEBUSYPIN (CORE_PIN1_PORTTOGGLE = CORE_PIN1_BITMASK)

// Fast nofrills SYNC pin set/clear
#define SETSYNCPIN (CORE_PIN0_PORTSET = CORE_PIN0_BITMASK)
#define CLEARSYNCPIN (CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK)
#define TOGGLESYNCPIN (CORE_PIN0_PORTTOGGLE = CORE_PIN0_BITMASK)

// Sensor data readout
#define NREADOUT 3694
#define DATASTART 16
#define DATASTOP 3680

// Size of the useful part
#define NPIXELS (DATASTOP-DATASTART)
#define NBYTES (NPIXELS*2)
#define NBYTES32 (NPIXELS*4)

// And first part of that is dark
#define NDARK 13

#ifdef  ALLINONEBOARD
#define NBITS 12
#else
#define NBITS 16
#endif

#define VFS (0.6)
#define VPERBIT (VFS/(1<<NBITS))

#define SHUTTERMIN 5

#define SETCNVST (CORE_PIN10_PORTSET = CORE_PIN10_BITMASK)
#define CLEARCNVST (CORE_PIN10_PORTCLEAR = CORE_PIN10_BITMASK)

#define USBSPEED 480.0E6
#define USBTRANSFERSECS ((float)NREADOUT*(16/USBSPEED))

#ifdef DEBUG
int debugprintf(const char* format, ...)
{
  char buffer[128] = {0};
  int n;
  va_list argptr;
  va_start(argptr, format);
  n = vsprintf (buffer, format, argptr );
  va_end(argptr);
  if (n > 0) {
    Serial.println(buffer);
  }
  else if (n<0) {
    Serial.print( "Error: debugprintf vsprintf");
    Serial.println(format);
  }
  return n;
}
#endif

// ------------------------------------------------------------------------------
// NOTE this is not designed to be thread safe.
// (Consider making it a singleton)

/********************************************************************
 * TCD1304 Device implemented in FlexPWM2, pins 3,4,5,6 and 10
 */
enum TCD1304_Mode {
  NOTCONFIGURED,
  FRAMESET,
  SINGLE
};

enum TCD1304_Timer_Trigger_Mode {
  NOTIMERTRIGGER,
  INTERVALTIMER,
  TRIGGER
};


class TCD1304Device
{
public:

  // struct to simplify our namespace for the flexpwm submodules
  struct SubModule {
    const char *name;
    const uint8_t submod;
    const uint8_t mask;
    const uint8_t pinA;
    const uint8_t muxvalA;
    const uint8_t pinB;
    const uint8_t muxvalB;
    const uint16_t ctrl2_mask;
    unsigned int period_counts = 0;
    unsigned int onA_counts = 0;
    unsigned int offA_counts = 0;
    unsigned int onB_counts = 0;
    unsigned int offB_counts = 0;
    uint16_t divider = 1;
    uint8_t prescale = 0;
    uint8_t filler = 0;
    float period_secs = 0;

    bool invertA = false;
    bool invertB = false;

    SubModule(const char *name_,
              uint8_t submod_, uint8_t mask_,
              uint8_t pinA_, uint8_t muxvalA_,
              uint8_t pinB_, uint8_t muxvalB_,
              uint16_t ctrl2_mask_) :
      name(name_),
      submod(submod_),mask(mask_),
      pinA(pinA_), muxvalA(muxvalA_),
      pinB(pinB_), muxvalB(muxvalB_),
      ctrl2_mask(ctrl2_mask_) {};
  };

    
  inline static enum TCD1304_Mode mode = NOTCONFIGURED;

  inline static enum TCD1304_Timer_Trigger_Mode timer_trigger_mode = NOTIMERTRIGGER;

  inline static uint64_t icg_cyccnt64_start = 0;
  inline static uint64_t icg_cyccnt64_now = 0;
  
  inline static uint64_t frame_cyccnt64_start = 0;
  inline static uint64_t frame_cyccnt64_now = 0;
  //inline static float frame_elapsed_secs = 0.;
  
  inline static unsigned int frameset_counter = 0;
  inline static unsigned int frameset_counts = 0;

  inline static unsigned int frame_counter = 0;
  inline static unsigned int frame_counts = 0;

  inline static unsigned int icg_counter = 0;

  inline static void (*frames_completed_callback)() = 0;
  inline static void (*framesets_completed_callback)() = 0;
  
  // For the interval timer
  inline static uint64_t interval_timer_cyccnt64_start = 0;
  inline static uint64_t interval_timer_cyccnt64_now = 0;

  // this is controls the frame readouts
  inline static float interval_timer_secs_incr = 1;
  inline static float interval_timer_secs_next = 0;
  inline static float interval_timer_period_secs = 0;
  
  inline static unsigned int interval_timer_counter = 0;
  inline static unsigned int interval_timer_counts  = 1;

  // For the interrupt timer
  inline static uint64_t interrupt_cyccnt64_start = 0;
  inline static uint64_t interrupt_cyccnt64_now = 0;
  
  // For the interrupt handler
  inline static unsigned int interrupt_counter = 0;
  inline static unsigned int interrupt_counts  = 1;
  inline static uint8_t interrupt_pin = INTERRUPT_PIN;
  inline static uint8_t interrupt_edge_mode = RISING;
  inline static uint8_t interrupt_pin_mode = INPUT;
  
  // For the Reader
  inline static unsigned int read_counter = 0;
  inline static unsigned int read_counts = 0;
  inline static uint16_t *read_buffer = 0;
  inline static uint16_t *read_pointer = 0;
  inline static void (*read_callback)() = 0;

  // timing adjustment
#ifdef  ALLINONEBOARD
  inline static uint16_t cnvst_extra_delay_counts = 0;   // best signal, steadily decreases with increasing delay
#else
  inline static uint16_t cnvst_extra_delay_counts = 1;   // this gives the lowest noise, does not improve at 2
#endif
  
  // This is for exposure == frame interval, we skip the first icg readout
  inline static bool skip_one = false;
  inline static bool skip_one_reload = false;

  // State of the flexpwm interface
  inline static bool busy = false;

  // Sync pin management
  inline static bool synctoggled = false;
  inline static unsigned int sync_counter = 0;
  inline static unsigned int sync_counts = 0;
  inline static  bool sync_enabled = true;

  // And now.... the submodules
  inline static IMXRT_FLEXPWM_t * const flexpwm = &IMXRT_FLEXPWM2;  
  inline static SubModule clk   = {"clk",CLK_SUBMODULE, CLK_MASK, CLK_PIN,CLK_MUXVAL, 0xFF,0, CLK_CTRL2_MASK};
  inline static SubModule sh    = {"sh", SH_SUBMODULE, SH_MASK,  SH_PIN,SH_MUXVAL, 0xFF,0, SH_CTRL2_MASK};
  inline static SubModule icg   = {"icg", ICG_SUBMODULE, ICG_MASK, ICG_PIN,ICG_MUXVAL, 0xFF,0, ICG_CTRL2_MASK};
  inline static SubModule cnvst = {"cnvst", CNVST_SUBMODULE, CNVST_MASK, 0xFF,0, 0xFF,0, CNVST_CTRL2_MASK}; // no pin for this submodule.

  // this is our interval clock, implemented on PWM4. option for pin3 output.
  inline static IMXRT_FLEXPWM_t * const timerflexpwm = &IMXRT_FLEXPWM4;
  inline static SubModule timer = {"timer", TIMER_SUBMODULE, TIMER_MASK, 0xFF,0, TIMER_PIN,TIMER_MUXVAL, TIMER_CTRL2_MASK};
   
  //---------------------------------------------
  TCD1304Device(unsigned int period=CLK_DEFAULT)
  {    
    stop_flexpwm();

    pinMode(interrupt_pin, interrupt_pin_mode);

    pinMode(BUSY_PIN, OUTPUT);
    digitalWrite(BUSY_PIN, BUSY_PIN_DEFAULT);

    pinMode(SYNC_PIN, OUTPUT);
    digitalWrite(SYNC_PIN, SYNC_PIN_DEFAULT);

    pinMode(CNVST_PIN, OUTPUT);   
    digitalWrite(CNVST_PIN, LOW);
 
  }

  /* ---------------------------------------------------------------------
     64 bit elapsed time clock based on cpu cycles
  */
  static uint64_t cycles64()
  {
    static uint32_t oldCycles = ARM_DWT_CYCCNT;
    static uint32_t highDWORD = 0;
    uint32_t newCycles = ARM_DWT_CYCCNT;
    if (newCycles < oldCycles)
    {
        ++highDWORD;
    }
    oldCycles = newCycles;
    return (((uint64_t)highDWORD << 32) | newCycles);
  }

  /// ----------------------------------------------------
  /// Elapsed time from flexpwm (frame) start to most recent frame (icg) interrupt
  static double frame_elapsed_secs()
  {
    return (double)(frame_cyccnt64_now - frame_cyccnt64_start)/F_CPU;
  }

  /// ----------------------------------------------------
  /// Elapsed time from first frame (icg) to most recent (cf actual timer interval)
  static double icg_elapsed_secs()
  {
    return (double)(icg_cyccnt64_now - icg_cyccnt64_start)/F_CPU;
  }

  /// ----------------------------------------------------
  /// Elapsed time from timer start to most recent timer interrupt
  static double interval_timer_elapsed_secs()
  {
    return (double)(interval_timer_cyccnt64_now - interval_timer_cyccnt64_start)/F_CPU;
  }
  
  /// ----------------------------------------------------
  /// Elapsed time from timer start to most recent timer interrupt
  static double interrupt_elapsed_secs()
  {
    return (double)(interrupt_cyccnt64_now - interrupt_cyccnt64_start)/F_CPU;
  }
  
  /* -----------------------------------------------------------------------------------------
     reads one word from the ADC, triggered by cnvst trailing edge
  */
  static void read_isr()
  {
    uint16_t status;
    
    flexpwm->SM[CNVST_SUBMODULE].STS = CNVST_CMPF_MASK;

    if (read_counter < read_counts) {

#ifdef ALLINONEBOARD
      adc->adc0->startReadFast(ANALOGPIN);
      while ( adc->adc0->isConverting() );
      *read_pointer = adc->adc0->readSingle();
#else      
      // we do have to assert the convert pin
      SETCNVST;
      delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
      CLEARCNVST;               // need 30 nanoseconds after this

      *read_pointer = SPI.transfer16(0xFFFF);
      *read_pointer ^= (0x1<<15);
#endif
      
      read_pointer++;
      read_counter++;

      // we're done with cnvst
      if (read_counter == read_counts) {
        NVIC_DISABLE_IRQ(CNVST_IRQ);

        // stop the cnvst clock
        flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);
        flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~CNVST_MASK);

#ifdef SH_STOP_IN_READ
	// Re-enable SH gate after readout
#if SH_CHANNEL==1 
        flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(SH_MASK);
#elif SH_CHANNEL == 2
        flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(SH_MASK);
#endif
#endif

        // If this is the end of the frameset, or we are running singles, we should stop clk,icg,sh here
        if ((frame_counter == frame_counts) || mode==SINGLE) {

          NVIC_DISABLE_IRQ(ICG_IRQ);            // no more icg intgerrupts (next one happens before sh)

          // stop the clk, sh, icg clocks
          flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((CLK_MASK|SH_MASK|ICG_MASK));
          flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~(CLK_MASK|SH_MASK|ICG_MASK));
          
          status = flexpwm->SM[ICG_SUBMODULE].STS;  // clear icg interrupts, everything
          flexpwm->SM[ICG_SUBMODULE].STS = status;

          if (synctoggled) {
            digitalToggleFast(SYNC_PIN);
            synctoggled = false;
          }
          
        }

        // This is how the user does per frame processing.
        if (read_callback) {
          read_callback();
        }

        // If this is the end of the frameset, we are not busy anymore.
        if (frame_counter == frame_counts) {
          if (busy) {
            busy = false;
            digitalToggleFast(BUSY_PIN);
          }
          
          // hook for clocked or triggered individual frames
          if (frames_completed_callback) {
            frames_completed_callback();
          }

          // frameset completed, update frameset counter
          frameset_counter++;
          
          // hook for clocked or triggered frame sets
          if ((frameset_counter >= frameset_counts) && framesets_completed_callback) {
            framesets_completed_callback();
          }
          
        }
        
      }
    }
  
    else {
      // oops!
      NVIC_DISABLE_IRQ(CNVST_IRQ);

      // stop the cnvst clock
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);
      flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~CNVST_MASK);
      
      Serial.println("Warning: cnvst isr called after read completed");
    }
  }

  /* -------------------------------------------------------------------------------
     Attach the read isr to the cnvst submodule interrupt
  */
  void cnvst_attach_read_isr(uint16_t *bufferpointer, unsigned int counts, void (*callback)())
  {
    NVIC_DISABLE_IRQ(CNVST_IRQ);

    // try to clear that first mysterious misaligned train
    read_callback = 0;
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);
    flexpwm->SM[CNVST_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;

    // here is the stop
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);
    flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~(CNVST_MASK));

    // setup the read buffer and callback on buffer completed
    read_buffer   = bufferpointer;   // pointer to the read buyffer
    read_counts   = counts;          // ength of the read
    read_callback = callback;        // invoked at readout completion

    // setup the interrupt enable and isr
    flexpwm->SM[CNVST_SUBMODULE].STS = CNVST_CMPF_MASK;    // clear whatever we are going to be looking at
    flexpwm->SM[CNVST_SUBMODULE].INTEN = CNVST_CMPF_MASK;  // enable whatever we are going to be looking at

    attachInterruptVector(CNVST_IRQ, read_isr);
  }

  // Start the readout, invoke this from the ICG interrupt
  static void read_start()
  {
    read_pointer = read_buffer;
    read_counter = 0;

    flexpwm->SM[CNVST_SUBMODULE].STS = CNVST_CMPF_MASK;  // clear interrupt status bit

#ifdef SH_STOP_IN_READ
    // Disable SH gate during readout
#if SH_CHANNEL==1 
    flexpwm->OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(SH_MASK);
#elif SH_CHANNEL == 2
    flexpwm->OUTEN &= ~FLEXPWM_OUTEN_PWMB_EN(SH_MASK);
#endif
#endif
  
    NVIC_ENABLE_IRQ(CNVST_IRQ);                              // enable interrupts

    // this is the start
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);             // set load ok
    flexpwm->SM[CNVST_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;   // force counter to the init value
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);              // and set the run bit
  }

  /* -------------------------------------------------------------------------------------------
     This is the integration clear gate interrupt, readout starts on falling edge of the ICG
  */
  static void icg_isr()
  {
    uint64_t cyccnt64;
    uint16_t status;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;            // clear everything

    cyccnt64 = cycles64();
    
    if (frame_counter < frame_counts) {

      // ICG B channel, Interrupt for the sync pin
      if (status&CMPF_MASKB_ON_OFF) {
        switch(status&CMPF_MASKB_ON_OFF)
          {
          case CMPF_MASKB_ON:  // leading eadge
            if (!synctoggled && (sync_counter<sync_counts)) {
              digitalToggleFast(SYNC_PIN);
              synctoggled = true;
              sync_counter++;
            }
            break;
          case CMPF_MASKB_OFF: // trailing edge
            if (synctoggled) {
              digitalToggleFast(SYNC_PIN);
              synctoggled = false;
            }
            break;
          }
      }
      
      // ICG A channel, Interrupt to launch the read
      if ((status & CMPF_MASKA_OFF)) {

        icg_counter++;

        if (skip_one) {

#ifdef DEBUG          
          Serial.println("icg_isr skip one");
#endif
          // if single, the read would stop the flexpwm, we do it here instead
          if (mode==SINGLE) {
            NVIC_DISABLE_IRQ(ICG_IRQ);            // no more icg intgerrupts (next one happens before sh)
            flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((CLK_MASK|SH_MASK|ICG_MASK));
            flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~(CLK_MASK|SH_MASK|ICG_MASK));
            status = flexpwm->SM[ICG_SUBMODULE].STS;  // clear icg interrupts, everything
            flexpwm->SM[ICG_SUBMODULE].STS = status;
          }
          
          skip_one = false;
        }

        else {
#ifdef DEBUG          
          Serial.println("icg_isr processing");
#endif
          // now is the time to update the counter, we'll use this at the end of the read
          frame_counter++;

          // readout, first interrupt comes in 3usecs
          read_start();
        }

        frame_cyccnt64_now = cyccnt64;  // starts in the flexpwm start function

        icg_cyccnt64_now = cyccnt64;
        if (icg_counter == 1) {
          icg_cyccnt64_start = cyccnt64;
        }
        
      }
    }

    /* If we get here, the readout failed to stop the clocks in time.
       That means our frame rate is too fast.
       Note this is not associated with a read, therefore we
       do not update the elapsed time cycle counters
    */
    else if ((status & CMPF_MASKA_OFF)) {
    
      NVIC_DISABLE_IRQ(ICG_IRQ);

      // stop the clk, sh, icg clocks
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((CLK_MASK|SH_MASK|ICG_MASK));
      flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~(CLK_MASK|SH_MASK|ICG_MASK));

      status = flexpwm->SM[ICG_SUBMODULE].STS;
      flexpwm->SM[ICG_SUBMODULE].STS = status; // clear everything
      
      if (synctoggled) {
        digitalToggleFast(SYNC_PIN);
        synctoggled = false;
      }

      if (busy) {
        busy = false;
        digitalToggleFast(BUSY_PIN);
      }
      Serial.println("Error: ICG interrupt after frameset complete, try slower frame rate");
      Serial.print( "read counter "); Serial.print(read_counter); Serial.print("/"); Serial.print(read_counts);
      Serial.print( " frame counter "); Serial.print(frame_counter); Serial.print("/"); Serial.print(frame_counts);
      Serial.print( " mode ");
      if (mode==SINGLE) {
        Serial.println("SINGLE");
      }
      else {
        Serial.println("FRAMESET");
      }
    }
  }

  /* ---------------------------------------------------------------------------------
     Attach the ICG ISR, this is called from setup_flexpwm()
  */  
  void icg_attach_isr(unsigned int counts)
  {
    uint16_t status;
    
    NVIC_DISABLE_IRQ(ICG_IRQ);

    icg_counter = 0;
    
    frame_counter = 0;
    frame_counts = counts;

    sync_counter = 0;
    sync_counts = skip_one_reload ? frame_counts: frame_counts+1;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;
    
    if (sync_enabled) {
      flexpwm->SM[ICG_SUBMODULE].INTEN = ICG_CMPF_MASK;  // enable interrupt both A and B
    } else {
      flexpwm->SM[ICG_SUBMODULE].INTEN = CMPF_MASKA_OFF; // enable interrupt A only
    }

    attachInterruptVector(ICG_IRQ, icg_isr);
    NVIC_ENABLE_IRQ(ICG_IRQ);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Starts flexpwm frames and framesets
  ////////////////////////////////////////////////////////////////////////////////////////////
  static bool start_flexpwm_()
  {
    uint16_t status;

    if (busy && mode != SINGLE) {
      Serial.println( "Error: pwm lccd already busy");
      return false;
    }

    // Busy flag starts here
    busy = true;
    digitalToggleFast(BUSY_PIN);
    
    // Stop all four submodules
    flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(0);

    // Reset sync
    if (synctoggled) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = false;
    }

    // Restarting frames?
    if (frame_counter==frame_counts) {
      skip_one = skip_one_reload;                    // reset the skip first gate logic 
      frame_counter = 0;                             // reset the frame counter
      icg_counter = 0;
      if (!frame_counts) {
        frame_counts = 1;
      }
    }

    // Restarting framesets?
    if (frameset_counter == frameset_counts) {
      frameset_counter = 0;
      if (!frameset_counts) {
        frameset_counts = 1;
      }
    }

    // Reset the counters
    sync_counter = 0;
    sync_counts = skip_one_reload ? frame_counts: frame_counts+1;    // if we are not skipping the first, sync needs one extra

    // Reset the readout pointer
    read_pointer = read_buffer;
    read_counter = 0;

    // Initialize the counters in the three submodules that are slaved to the master clock
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CLK_MASK|SH_MASK|ICG_MASK);  // set ldok 
    flexpwm->SM[CLK_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;         // force out while ldok, initializes the counters

    // let's init the cnvst counter too
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);                 // set ldok 
    flexpwm->SM[CNVST_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;       // force out while ldok, initializes the counters
    
    // Clear the interrupt status bits in the icg and cnvst (readout) modules (assume others are not used)
    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;       // clear any pending icg interrupts

    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;     // clear any pending cnvst interrupts

    // Enable the ICG interrupt
    NVIC_ENABLE_IRQ(ICG_IRQ);

    // And ... start!
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CLK_MASK|SH_MASK|ICG_MASK);   // set run for three submodules -- this is the actual start

    // if first frame, record the mcu cycle counter
    if (!icg_counter) {
      frame_cyccnt64_start = cycles64();
    }

    // if the interrupt is already set when this happens, the isr will still be launched.
    /*
    NVIC_ENABLE_IRQ(ICG_IRQ);
    */
    
    return true;
  }

  // start and mark as not trigger or timer
  bool start_flexpwm()
  {
    timer_trigger_mode = NOTIMERTRIGGER;
    return start_flexpwm_();    
  }
  
  // This is a version that can be attached as an ISR
  static void start_flexpwm_isr()
  {
    start_flexpwm_();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Stop a flexpwm frameset or single frame readout
  ////////////////////////////////////////////////////////////////////////////////////////////
  
  static void stop_flexpwm()
  {
    uint16_t u16;

    // Stop all four submodules
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(0xF);
    flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(0);

    // Disable all of the interrupts
    NVIC_DISABLE_IRQ(ICG_IRQ);
    //NVIC_DISABLE_IRQ(SH_IRQ);
    NVIC_DISABLE_IRQ(CNVST_IRQ);

    // Clear all of the interrupts
    u16 = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = u16;   // clear pending ICG interrupts if any

    //u16 = flexpwm->SM[SH_SUBMODULE].STS;
    //flexpwm->SM[SH_SUBMODULE].STS = u16;   // clear pending SH interrupts if any

    u16 = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = u16; // clear pending CNVST interrupts if any

    // Reset busy flag and pin
    if (busy) {
      busy = false;
      digitalToggleFast(BUSY_PIN);
    }

    // Reset sync flag and pin
    if (synctoggled) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = false;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Setup callbacks for frames (a single frameset) completed.
  ////////////////////////////////////////////////////////////////////////////////////////////

  void clear_frames_completed_callback()
  {
    frames_completed_callback = 0;
  }

  void load_frames_completed_callback(void (*callback)(), unsigned int nframes)
  {
    // allow for rearm callback. same nframes
    if (nframes) {
      frame_counts = nframes;
    }
    frames_completed_callback = callback;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Setup callbacks for frameset completed.
  ////////////////////////////////////////////////////////////////////////////////////////////
  
  void clear_framesets_completed_callback()
  {
    framesets_completed_callback = 0;
  }
  
  void load_framesets_completed_callback(void (*callback)(), unsigned int nsets)
  {
    // allow for rearm callback. same nframes
    if (nsets) {
      frameset_counts = nsets;
    }
    frameset_counter = 0;
    framesets_completed_callback = callback;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Diagnostic print and check for submodule configuration
  ////////////////////////////////////////////////////////////////////////////////////////////
  
  void print_submodule(SubModule *p)
  {
    char cbuffer[128] = {0};


    float frequency = p->period_counts ? F_BUS_ACTUAL/(p->divider*p->period_counts) : 0;
    float period_secs = (float) (p->period_counts*p->divider)/F_BUS_ACTUAL;

    float A_on_secs = (float)(p->onA_counts*p->divider)/F_BUS_ACTUAL;
    float A_off_secs = (float)(p->offA_counts*p->divider)/F_BUS_ACTUAL;
    float A_duration_secs = (float)((p->offA_counts-p->onA_counts)*p->divider)/F_BUS_ACTUAL;

    float B_on_secs = (float)(p->onB_counts*p->divider)/F_BUS_ACTUAL;
    float B_off_secs = (float)(p->offB_counts*p->divider)/F_BUS_ACTUAL;
    float B_duration_secs = (float)((p->offB_counts-p->onB_counts)*p->divider)/F_BUS_ACTUAL;
    
    snprintf( cbuffer, sizeof(cbuffer),
	      "flexpwm: %s period %u presc %d div %d => %.6g secs %.6g Hz",
	      p->name, p->period_counts, p->prescale, p->divider, period_secs, frequency);
    Serial.println(cbuffer);

    snprintf( cbuffer, sizeof(cbuffer),
	      "flexpwm: %s A pin %d mux 0x%x on %u off %u %s => %.6g to %.6g, %.6g secs",
	      p->name, p->pinA, p->muxvalA, p->onA_counts, p->offA_counts,
	      p->invertA?"inverting":"noninverting",
	      A_on_secs, A_off_secs, A_duration_secs);
    Serial.println(cbuffer);

    snprintf( cbuffer, sizeof(cbuffer),
	      "flexpwm: %s B pin %d mux 0x%x on %u off %u %s => %.6g to %.6g, %.6g secs",
	      p->name, p->pinB, p->muxvalB, p->onB_counts, p->offB_counts,
	      p->invertB?"inverting":"noninverting",
	      B_on_secs, B_off_secs, B_duration_secs);
    Serial.println(cbuffer);
  }

  // ---------------------------------------------------------------------------------------------------
  void print_submodule_error(const char *name, const char *errmsg)
  {
    Serial.print("Error: ");
    Serial.print(name);
    Serial.print(" ");
    Serial.println(errmsg);
  }

  bool check_submodule(SubModule *p)
  {
    bool retv = true;
    if (p->divider<1) {
      print_submodule_error(p->name, "divider < 1");
      retv = false;
    }
    if (p->divider>128) {
      print_submodule_error(p->name, "divider > 128");
      retv = false;
    }
    
    if (p->onA_counts || p->offA_counts) {
      if (p->onA_counts>=p->offA_counts) {
        print_submodule_error(p->name, "onA > offA");
        retv = false;
      }
      if (p->offA_counts>p->period_counts) {
        print_submodule_error(p->name, "offA >= period");
        retv = false;
      }
    }
    
    if (p->onB_counts || p->offB_counts) {
      if (p->onB_counts>=p->offB_counts) {
        print_submodule_error(p->name, "onB > offB");
        retv = false;
      }
      if (p->offB_counts>p->period_counts) {
        print_submodule_error(p->name, "offB >= period");
        retv = false;
      }
    }
    
    return retv;
  }

  bool print_and_check_submodule(SubModule *p)
  {
    print_submodule(p);
    return check_submodule(p);
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Load the submodule configuration into hardware
  ////////////////////////////////////////////////////////////////////////////////////////////
  
  void load_submodule(SubModule *p, IMXRT_FLEXPWM_t * const  q=flexpwm)
  {
    unsigned int submod = p->submod;
    uint16_t mask = p->mask;
      
    // stop should be done globally outside
    //flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    //flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(mask);
    //stop_(mask);

    Serial.print("loading sub_module ");
    Serial.println(p->name);

    q->SM[submod].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(p->prescale);
      
    q->SM[submod].INIT = 0;      
    q->SM[submod].VAL0 = 0;
    q->SM[submod].VAL1 = p->period_counts - 1;    
    q->SM[submod].VAL2 = p->onA_counts;
    q->SM[submod].VAL3 = (p->offA_counts > 0) ? p->offA_counts: 0;
    q->SM[submod].VAL4 = p->onB_counts;
    q->SM[submod].VAL5 = (p->offB_counts > 0) ? p->offB_counts : 0;

    p->period_secs = (float) p->period_counts * ((float)(1<<p->prescale)/ F_BUS_ACTUAL);
    
    // Do we have a Pin for the A channel?
    if (p->pinA != 0xFF && p->offA_counts) {
      q->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
      *(portConfigRegister(p->pinA)) =  p->muxvalA;
      
      if (p->invertA) {
        q->SM[submod].OCTRL |= 1<<10;  // Is inverted
      }
      else {
        q->SM[submod].OCTRL &= ~(1<<10);  // Is not inverted
      }
    }
    else {
      q->OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(mask);
    }

    // Do we have a Pin for the B channel?
    if (p->pinB != 0xFF && p->offB_counts) {
      q->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
      *(portConfigRegister(p->pinB)) = p->muxvalB;

      if (p->invertB) {
        q->SM[submod].OCTRL |= 1<<9;  // Is inverted
      }
      else {
        q->SM[submod].OCTRL &= ~(1<<9);  // Is not inverted
      }
    }
    else {
      q->OUTEN &= ~FLEXPWM_OUTEN_PWMB_EN(mask);
    }

    // Setup shared clocks and forced starts
    if (p->ctrl2_mask) {
      q->SM[submod].CTRL2 |= p->ctrl2_mask;
    }
    
      
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Configure the flexpwm to operate the TCD1304
  ////////////////////////////////////////////////////////////////////////////////////////////
  bool setup_flexpwm(unsigned int clk_base, float exposure_secs, float pulse_secs, float frame_secs,
                     unsigned int nframes, uint16_t *buffer, unsigned int nbuffer,
                     void (*framecallback)())
  {

    char cbuffer[128] = {0};
    unsigned int cnvst_delay;

    // stop the lccd clocks
    stop_flexpwm();

    sprintf(cbuffer,"tcd1304 flexpwm setup with clk base %d exposure_secs %.5g pulse %.5g frames %d",
	    clk_base, exposure_secs, pulse_secs, nframes);
    Serial.println(cbuffer);
    
    // globals
    frame_cyccnt64_start = 0;
    icg_cyccnt64_start = 0;
    
    frameset_counter = 0;
    frameset_counts = 0;
    frame_counter = 0;
    frame_counts = 0;


    float secs_min = 0.;
    float secs_max = 0.;

    // ------------------------------------------------------------------
    // Setup the dividers, etc
    if (!clk_base) clk_base = 64;
      
    clk.prescale = 0;             // clk needs to be prescale 0
    
    switch(clk_base) {
    case 64:                      // 2.34MHz, 427nsecs/clock, readout at 586KHz, 1.7usecs/pixel
      cnvst.prescale = 3;         // 53nsec timing for the cnvst pulse
      sh.prescale = 6;            // divider = 32; it is 1/2 period, but we need .01 secs for readout
      icg.prescale = 6;           // forces icg onto leading edge of clock
      cnvst_delay = 0;            // this plus overheads puts the cnvst at the third clock
      if (frame_secs<=0.) frame_secs = 0.01;
      break;
    case 128:                     // 1.17Mhz, 853nsecs/clock, readout at 292KHz, 3.41usecs/pixel
      cnvst.prescale = 4;         // 106nsec timing for the cnvst pulse
      sh.prescale = 7;            // 833nsec timing for shift gate
      icg.prescale = 7;           // forces icg onto leading edige of clock
      cnvst_delay = clk_base;     // this plus overheads puts the cnvst at the third clock
      if (frame_secs<=0.) frame_secs = 0.02;
      break;
    default:                      // note this includes clk base is 0
      Serial.println("td1304 setup requires base 64 (10msec readout) or 128 (20msec readout)");
      return false;
    }

    // setup the other dividers
    clk.divider   = (1<<clk.prescale);
    cnvst.divider = (1<<cnvst.prescale);
    sh.divider    = (1<<sh.prescale);
    icg.divider   = (1<<icg.prescale);

    // ------------------------------------------------------------------
    // Frame period in range?
    secs_min = (float) NREADOUT*4*clk_base/F_BUS_ACTUAL + USBTRANSFERSECS + 50.E-6;
    secs_max = (float) (65535 * icg.divider) / F_BUS_ACTUAL;
    
    if (frame_secs <= secs_min || frame_secs >= secs_max ) {
      Serial.print("Error: setup_flexpwm with frame interval out of range ");
      Serial.print( frame_secs, 6);
      Serial.print(", min ");
      Serial.print(secs_min,6);
      Serial.print(" max ");
      Serial.println(secs_max,6);
      return false;
    }
          
    // Exposure in range?
    secs_min = pulse_secs + 5e-6;
    secs_max = frame_secs;
    
    if (exposure_secs <= secs_min || exposure_secs > secs_max ) {
      Serial.print("Error: setup_flexpwm with exposure out of range ");
      Serial.print(exposure_secs,6);
      Serial.print(", min ");
      Serial.print(secs_min,6);
      Serial.print(" max ");
      Serial.println(secs_max,6);
      return false;
    }
    
    // ------------------------------------------------------------------
    // Setup the master clock
    clk.period_counts  = clk_base/clk.divider;
    clk.onA_counts     = 0;
    clk.offA_counts    = (clk.period_counts/2);

    if (!print_and_check_submodule(&clk)) {
      return false;
    }
    
    // ------------------------------------------------------------------
    // Setup the CNVST clock, 4 times the clock period
    cnvst.divider = (1<<cnvst.prescale);
    cnvst.period_counts = 4*clk_base/cnvst.divider;
    cnvst.onA_counts    = cnvst_delay/cnvst.divider + cnvst_extra_delay_counts; // allow extra delay for testing
    cnvst.offA_counts   = cnvst.onA_counts + ceil(CNVST_PULSE_SECS*F_BUS_ACTUAL/cnvst.divider);

    if (!print_and_check_submodule(&cnvst)) {
      return false;
    }
    
    // ------------------------------------------------------------------
    // Setup the shift gate
    sh.period_counts = ceil(exposure_secs*F_BUS_ACTUAL/sh.divider);
    sh.onA_counts = 1;
    sh.offA_counts = sh.onA_counts + ceil(pulse_secs*F_BUS_ACTUAL/sh.divider);

    if (!print_and_check_submodule(&sh)) {
      return false;
    }
    
    // ------------------------------------------------------------------
    // setup icg
    icg.divider = (1<<icg.prescale);
    icg.period_counts = ceil(frame_secs*F_BUS_ACTUAL/icg.divider);
    icg.period_counts = (icg.period_counts/sh.period_counts)*sh.period_counts;  // icg period must be multiple of sh period
    icg.onA_counts = 0;
    icg.offA_counts = sh.offA_counts + ceil(2.E-6*F_BUS_ACTUAL/icg.divider);

    icg.invertA = true;
    
    icg.onB_counts = sh.offA_counts;
    icg.offB_counts = icg.onB_counts + sh.period_counts*sh.divider/icg.divider;

    if (icg.period_counts >= 2*sh.period_counts) {
      skip_one = false;
      skip_one_reload = false;
      icg.onA_counts  += sh.period_counts;
      icg.offA_counts += sh.period_counts;
      icg.onB_counts  += sh.period_counts;
      icg.offB_counts += sh.period_counts;
    }
    else {
      skip_one = true;
      skip_one_reload = true;
    }

    if (icg.offB_counts >= icg.period_counts) {
      icg.offB_counts = icg.period_counts;
    }
    
    if (!print_and_check_submodule(&icg)) {
      return false;
    }

    // ====================================================================
    load_submodule(&clk);

    load_submodule(&sh);

    load_submodule(&icg);

    load_submodule(&cnvst);

    // ---------------------------------------------
    // Setup the Read ISR  
    cnvst_attach_read_isr(buffer, nbuffer,framecallback);

    // Setup the ICG ISR
    icg_attach_isr(nframes);

    
    // stuff that other routines need to know
    mode = FRAMESET;
    timer_trigger_mode = NOTIMERTRIGGER;
    frameset_counts = 1;
    
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  ///   Configure the flexpwm for single pulse operation, for use with clock and trigger isrs
  ////////////////////////////////////////////////////////////////////////////////////////////

  bool setup_flexpwm_singles( unsigned int clk_base, float pulse_width_secs, unsigned int nframes,
                                  uint16_t *buffer, unsigned int nbuffer, void (*sendcallback)() )
  {
    bool retv;

    if (!clk_base) clk_base = 64;
 
    retv = setup_flexpwm(clk_base, 0.01, pulse_width_secs, 0.01, nframes, buffer, nbuffer, sendcallback);
    if (!retv) return false;

    skip_one = true;
    skip_one_reload = true;
    
    mode = SINGLE;

    return true;
  }
  
  /* =========================================================================================
     Interval timer for precise long time scale frame clocking
  */
  static void interval_timer_isr()
  {
    double elapsed_secs;

    interval_timer_cyccnt64_now = cycles64();
    
    elapsed_secs = (double)(interval_timer_cyccnt64_now-interval_timer_cyccnt64_start)/F_CPU;
      
    if (elapsed_secs >= interval_timer_secs_next) {

      interval_timer_secs_next += interval_timer_secs_incr;
      interval_timer_counter++;

#ifdef DEBUG      
      Serial.print("interval timer isr at ");
      Serial.print(elapsed_secs, 6);
      Serial.print(" skip one ");
      Serial.println( skip_one );
#endif
      start_flexpwm_isr();
      
      if (interval_timer_counter >= interval_timer_counts) {
#ifdef DEBUG
        Serial.println("interval timer isr:  stop");
#endif
        interval_timer_stop();
      }
    }
    
  }

  /// --------------------------------------------
  /// Start the interval timer  
  static void interval_timer_start()
  {
    uint16_t status;

    timer_trigger_mode= INTERVALTIMER;
    
    status = timerflexpwm->SM[ICG_SUBMODULE].STS;
    timerflexpwm->SM[ICG_SUBMODULE].STS = status;                    // everything

    timerflexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(TIMER_MASK);            // set ldok 
    timerflexpwm->SM[TIMER_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;  // force out while ldok, initializes the counters
    
    interval_timer_counter = 0;
    interval_timer_secs_next = 0.;

    frameset_counter = 0;
    frameset_counts = 1;

    // start the 64 bit elapsed time
    interval_timer_cyccnt64_start = cycles64();
    
    // And ... start!
    timerflexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(TIMER_MASK);    

    // Enable the ICG interrupt
    NVIC_ENABLE_IRQ(TIMER_IRQ);

#ifdef DEBUG              
    Serial.println("interval timer start done");
#endif
  }

  static void interval_timer_stop()
  {
    uint16_t status;

    NVIC_DISABLE_IRQ(TIMER_IRQ);
    timerflexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(TIMER_MASK);    

    status = timerflexpwm->SM[ICG_SUBMODULE].STS;
    timerflexpwm->SM[ICG_SUBMODULE].STS = (status & ~(1<<4)); 
  }
  

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Setup the flexpwm for the interval timer
  ////////////////////////////////////////////////////////////////////////////////////////////
  bool setup_interval_timer( float period_secs, unsigned int ncounts, bool connect_to_pin=false)
  {
    unsigned int prescale = 0;
    unsigned int divider = 1;
    unsigned int period_counts;

    uint16_t u16;

    NVIC_DISABLE_IRQ(TIMER_IRQ);
    timerflexpwm->MCTRL = 0;    
    
    if ( (mode == NOTCONFIGURED) || !icg.period_secs) {
      Serial.println("Error: setup_interval_timer, but flexpwm is not configured, call setup_flexpwm() first");
      return false;
    }
    if (period_secs < icg.period_secs) {
      Serial.print("Error: setup_interval_timer, but period is too fast, min is ");
      Serial.println(icg.period_secs);
      return false;
    }

    // This is what controls which timer interrupts issue the flexpwm start
    interval_timer_secs_incr = period_secs;    
    interval_timer_secs_next = 0.;

    interval_timer_period_secs = period_secs;
    
    // Setup the call back and stop count
    if (!ncounts) {
      if (mode == SINGLE) {
        ncounts = frame_counts + 1;
      }
      else if (mode == FRAMESET) {
        ncounts = frameset_counts;
      }
    }
    
    interval_timer_counts   = ncounts;
    interval_timer_counter  = 0;

    // Setup clock period and divider
    divider       = floor(period_secs/icg.period_secs);
    period_counts = ceil((period_secs/divider)*F_BUS_ACTUAL);

    prescale = 0;
    while((period_counts/(1<<prescale)) >= (1<<16)) {
      prescale++;
    }
    divider       = (1<<prescale);
    period_counts = ceil(period_counts/divider);  // we need ceil else we fall behind and loose frames
                         
    // Configure the flexpwm submodule
    timer.prescale      = prescale;
    timer.divider       = divider;
    timer.period_counts = period_counts; 
    timer.onB_counts    = 1;
    timer.offB_counts   = connect_to_pin? timer.onB_counts + ceil(10.E-6*F_BUS_ACTUAL/divider): 0;

    if (!print_and_check_submodule(&timer)) {
      return false;
    }

    // Load the flexpwm submodule registers
    timerflexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(TIMER_MASK);            // clear lodk

    load_submodule(&timer, timerflexpwm);

    // setup the interrupt
    u16 = timerflexpwm->SM[TIMER_SUBMODULE].STS;        // clear all of the interrupts except val2
    timerflexpwm->SM[TIMER_SUBMODULE].STS = (u16 & ~(TIMER_CMPF_MASK));
    
    timerflexpwm->SM[TIMER_SUBMODULE].INTEN = TIMER_CMPF_MASK; // enable interrupt B only

    attachInterruptVector(TIMER_IRQ, interval_timer_isr);

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Interrupt routines
  ////////////////////////////////////////////////////////////////////////////////////////////

  static void interrupt_isr()
  {
    interrupt_cyccnt64_now = cycles64();

#ifdef DEBUG      
    double elapsed_secs;
    elapsed_secs = (double)(interrupt_cyccnt64_now-interrupt_cyccnt64_start)/F_CPU;
          Serial.print("interrupt isr at ");
    Serial.print(elapsed_secs, 6);
    Serial.print(" skip one ");
    Serial.println( skip_one );
#endif
    
    start_flexpwm_isr();
    
    if (interrupt_counter >= interrupt_counts) {
      detachInterrupt(digitalPinToInterrupt(interrupt_pin));

#ifdef DEBUG
      Serial.println("interrupt isr:  stop");
#endif
    }
  }

  void stop_interrupts()
  {
    detachInterrupt(digitalPinToInterrupt(interrupt_pin));
  }
  
  void start_interrupts( )
  {
    timer_trigger_mode = TRIGGER;
    
    interrupt_counter = 0;

    interrupt_cyccnt64_now = cycles64();
    
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), interrupt_isr, interrupt_edge_mode);  
  }
  
  bool setup_interrupts(unsigned int ncounts = 0)
  {
    if ( (mode == NOTCONFIGURED) || !icg.period_secs) {
      Serial.println("Error: setup_interrupt, but flexpwm is not configured, call setup_flexpwm() first");
      return false;
    }

    // Setup the call back and stop count
    if (!ncounts) {
      if (mode == SINGLE) {
        ncounts = frame_counts + 1;
      }
      else if (mode == FRAMESET) {
        ncounts = frameset_counts;
      }
    }
    interrupt_counts = ncounts;

    return true;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Low level functions for command line register access.
  ////////////////////////////////////////////////////////////////////////////////////////////
  
  void set_clock_master(uint8_t submod){
      flexpwm->SM[submod].CTRL2 &= 0xFFF0;
      flexpwm->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_MASTER;
  }
  void set_clock_slave(uint8_t submod){
      flexpwm->SM[submod].CTRL2 &= 0xFFF0;
      flexpwm->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_SLAVE;
  }
  void set_clock_sync(uint8_t submod){
      flexpwm->SM[submod].CTRL2 &= 0xFFF0;
      flexpwm->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_SYNC;
  }
  
  void clear_ldok(uint8_t mask) {
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
  }
  void set_ldok(uint8_t mask) {
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
  }
  void clear_run(uint8_t mask) {
    flexpwm->MCTRL &= FLEXPWM_MCTRL_RUN(~mask);
  }
  void set_run(uint8_t mask) {
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(mask);
  }

  void force(uint8_t submod) {
    if(submod<4) {
      flexpwm->SM[submod].CTRL2 = FLEXPWM_SMCTRL2_FORCE;
    }
  }

  void set_prescale(uint8_t submod, uint8_t prescale=0) {
    if (submod<4) {
      flexpwm->SM[submod].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);
    }
  }

  void set_init(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].INIT = value;
    }
  }

  void set_val0(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL0 = value;
    }
  }

  void set_val1(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL1 = value;
    }
  }

  void set_val2(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL2 = value;
    }
  }

  void set_val3(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL3 = value;
    }
  }

  void set_val4(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL4 = value;
    }
  }

  void set_val5(uint8_t submod,uint16_t value) {
    if (submod<4) {
      flexpwm->SM[submod].VAL5 = value;
    }
  }
  // -----------------------------------------------------------
  uint16_t set_outen( uint16_t mask16) {    
    flexpwm->OUTEN = mask16;
    return flexpwm->OUTEN;
  }
  
  uint16_t set_outen_on( uint16_t mask16) {    
    flexpwm->OUTEN |= mask16;
    return flexpwm->OUTEN;
  }

  uint16_t set_outen_off( uint16_t mask16) {    
    flexpwm->OUTEN &= ~mask16;
    return flexpwm->OUTEN;
  }
  
  uint16_t set_outenA_on( uint8_t mask8) {    
    flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask8);
    return flexpwm->OUTEN;
  }
  
  

  // ===========================================================
  
  void printbits16_(uint16_t u16, int bits)
  {
    while(bits> 0) {
      bits--;
      Serial.print((u16>>bits)&1);
    }
  }

  void register_dump(IMXRT_FLEXPWM_t * const  q=flexpwm, uint8_t mask=0xFF)
  {
    uint16_t u16;

    Serial.print("flexpwm ");
    Serial.println((unsigned int)q, HEX);
    
    u16 = q->MCTRL;
    Serial.print("MCTRL "); printbits16_(u16,16); // Serial.print(u16,BIN);
    Serial.print(" IPOL "); printbits16_((u16>>12),4); // Serial.print((u16>>12)&0XF,BIN);
    Serial.print(" RUN "); printbits16_((u16>>8),4); // Serial.print((u16>>8)&0XF,BIN);
    Serial.print(" LDOK "); printbits16_(u16,4); // Serial.print(u16&0XF,BIN);
    Serial.println("");
  
    u16 = q->MCTRL2;
    Serial.print("MCTRL2 MONPLL "); Serial.print(u16&0XF,BIN);
    Serial.println("");
 
    u16 = q->OUTEN;
    Serial.print("OUTEN "); printbits16_(u16,16); // Serial.print(u16,BIN);
    Serial.print(" PWMA "); printbits16_((u16>>8),4); // Serial.print((u16>8)&0xF,BIN);
    Serial.print(" PWMB "); printbits16_((u16>>4),4); // Serial.print((u16>4)&0xF,BIN);
    Serial.print(" PWMX "); printbits16_(u16,4); // Serial.print(u16&0xF,BIN);
    Serial.println("");

    for (uint8_t submodule = 0; submodule<4; submodule++) {
      
      if (mask & (1<<submodule)) {

        Serial.println("#-------------------------------------");
        Serial.print("Submodule "); Serial.print(submodule);
        Serial.println("");

        u16 = q->SM[submodule].CNT;
        Serial.print("CNT "); Serial.println(u16);

        u16 = q->SM[submodule].INIT;
        Serial.print("INIT "); Serial.println(u16);

        u16 = q->SM[submodule].CTRL2;
        Serial.print("CTRL2 "); printbits16_(u16,16); // Serial.print(u16,BIN);
        Serial.print(" DBGEN "); Serial.print((u16>>15));
        Serial.print(" WAITEN "); Serial.print(((u16>>14)&1));
        Serial.print(" INDEP "); Serial.print(((u16>>13)&1));
        Serial.print(" PWM23_INIT "); Serial.print(((u16>>12)&1));
        Serial.print(" PWM45_INIT "); Serial.print(((u16>>11)&1));
        Serial.print(" PWMX_INIT "); Serial.print(((u16>>10)&1));
        Serial.print(" INIT_SEL "); Serial.print(((u16>>8)&3));
        Serial.print(" FRCEN "); Serial.print(((u16>>7)&1));
        Serial.print(" FORCE "); Serial.print(((u16>>6)&1));
        Serial.print(" FORCE_SEL "); Serial.print(((u16>>3)&7));
        Serial.print(" RELOAD_SEL "); Serial.print(((u16>>2)&1));
        Serial.print(" CLK_SEL "); Serial.print((u16&3));
        Serial.println("");

        u16 = q->SM[submodule].CTRL;
        Serial.print("CTRL "); printbits16_(u16,16); // Serial.print(u16,BIN);
        Serial.print(" LDFQ "); Serial.print(((u16>>12)&0xF));
        Serial.print(" HALF "); Serial.print(((u16>>11)&1));
        Serial.print(" FULL "); Serial.print(((u16>>10)&1));
        Serial.print(" DT "); Serial.print(((u16>>9)&0x1));Serial.print(((u16>>8)&0x1));
        Serial.print(" COMPMODE "); Serial.print(((u16>>10)&1));
        Serial.print(" PRSC "); Serial.print(((u16>>4)&0x7));
        Serial.print(" SPLIT "); Serial.print(((u16>>3)&1));
        Serial.print(" LDMOD "); Serial.print(((u16>>2)&1));
        Serial.print(" DBLX "); Serial.print(((u16>>1)&1));
        Serial.print(" DBLEN "); Serial.print((u16&1));
        Serial.println("");

        u16 = q->SM[submodule].OCTRL;
        Serial.print("OCTRL "); printbits16_(u16,16); // Serial.print(u16,BIN);
        Serial.print(" PWMA_IN "); Serial.print(((u16>>15)&1));
        Serial.print(" PWMB_IN "); Serial.print(((u16>>14)&1));
        Serial.print(" PWMX_IN "); Serial.print(((u16>>13)&1));
        Serial.print(" POLA "); Serial.print((u16>>10)&1);
        Serial.print(" POLB "); Serial.print((u16>>9)&1);
        Serial.print(" POLX "); Serial.print((u16>>8)&1);
        Serial.print(" PWMA_FS "); Serial.print((u16>>4)&3);
        Serial.print(" PWMB_FS "); Serial.print((u16>>2)&3);
        Serial.print(" PWMX_FS "); Serial.print(u16&3);
        Serial.println("");
  
        u16 = q->SM[submodule].VAL0;
        Serial.print("VAL0 "); Serial.print(u16);

        u16 = q->SM[submodule].VAL1;
        Serial.print(" VAL1 "); Serial.print(u16);

        u16 = q->SM[submodule].VAL2;
        Serial.print(" VAL2 "); Serial.print(u16);

        u16 = q->SM[submodule].VAL3;
        Serial.print(" VAL3 "); Serial.print(u16);

        u16 = q->SM[submodule].VAL4;
        Serial.print(" VAL4 "); Serial.print(u16);

        u16 = q->SM[submodule].VAL5;
        Serial.print(" VAL5 "); Serial.print(u16);
        Serial.println("");

        u16 = q->SM[submodule].STS;
        Serial.print("STS "); printbits16_(u16,16); // Serial.print(u16,BIN);
        Serial.print(" RUF "); Serial.print(((u16>>14)&0x1));
        Serial.print(" REF "); Serial.print(((u16>>13)&0x1));
        Serial.print(" RF "); Serial.print(((u16>>12)&0x1));
        Serial.print(" CFA1 "); Serial.print(((u16>>11)&0x1));
        Serial.print(" CFA0 "); Serial.print(((u16>>10)&0x1));
        Serial.print(" CFB1 "); Serial.print(((u16>>9)&0x1));
        Serial.print(" CFB0 "); Serial.print(((u16>>8)&0x1));
        Serial.print(" CFX1 "); Serial.print(((u16>>7)&0x1));
        Serial.print(" CFX0 "); Serial.print(((u16>>6)&0x1));
        Serial.print(" CMPF0 "); printbits16_(u16,6); // Serial.print(u16&0x3F,BIN);
        Serial.println("");
  
        u16 = q->SM[submodule].INTEN;
        Serial.print("INTEN "); printbits16_(u16,16); // Serial.print(u16,BIN);
        Serial.print(" REIE "); Serial.print(((u16>>13)&0x1));
        Serial.print(" RIE "); Serial.print(((u16>>12)&0x1));
        Serial.print(" CA1IE "); Serial.print(((u16>>11)&0x1));
        Serial.print(" CA0IE "); Serial.print(((u16>>10)&0x1));
        Serial.print(" CB1IE "); Serial.print(((u16>>9)&0x1));
        Serial.print(" CB0IE "); Serial.print(((u16>>8)&0x1));
        Serial.print(" CX1IE "); Serial.print(((u16>>7)&0x1));
        Serial.print(" CX0IE "); Serial.print(((u16>>6)&0x1));
        Serial.print(" CMPIE "); printbits16_(u16,6); // Serial.print(u16&0x3F,BIN);
        Serial.println("");
      }
    }
  }

  // -----------------------------------------
  static void setup_digital_pins()
  {
    pinMode(INTERRUPT_PIN, INPUT);

    pinMode(BUSY_PIN, OUTPUT);
    digitalWriteFast(BUSY_PIN, BUSY_PIN_DEFAULT);

    pinMode(SYNC_PIN, OUTPUT);
    digitalWriteFast(SYNC_PIN, SYNC_PIN_DEFAULT);

    // Clock
    pinMode(CLK_PIN,  OUTPUT);   
    digitalWriteFast(CLK_PIN, LOW);

    pinMode(CLK_MONITOR_PIN, INPUT);

    // SH gate
    pinMode(SH_PIN, OUTPUT);
    digitalWriteFast(SH_PIN, LOW);

    // Integratinon Clear Gate
    pinMode(ICG_PIN, OUTPUT);
    digitalWriteFast(ICG_PIN, HIGH);

    // CNVST to ADC
    pinMode(CNVST_PIN, OUTPUT);   
    digitalWriteFast(CNVST_PIN, LOW);

    mode = NOTCONFIGURED;
  }
  
  static void close()
  {
    stop_flexpwm();  
  
    NVIC_DISABLE_IRQ(CNVST_IRQ);
    flexpwm->SM[CNVST_SUBMODULE].STS = CNVST_CMPF_MASK; // clear pending interrupt if any

    // Reactivation as digital i/o pins
    setup_digital_pins();
    
    if (busy) {
      busy = false;
      digitalToggleFast(BUSY_PIN);
    }
    
    if (synctoggled) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = false;
    }
  }
  
};

#endif
