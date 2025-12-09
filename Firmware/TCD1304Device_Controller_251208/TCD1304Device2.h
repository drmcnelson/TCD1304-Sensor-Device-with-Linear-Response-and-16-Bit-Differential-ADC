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

// =========================================
// All In One Board

//#define ALLINONEBOARD

// CONFIGURE PULLUPS/PULLDOWNS
#define PINPULLS

// =========================================

#ifdef  ALLINONEBOARD
#include <ADC.h>
#include <ADC_util.h>
extern ADC *adc;
#define ANALOGPIN A0
#define TCD1304_MAXCLKHZ 2.35E6
#else
#define TCD1304_MAXCLKHZ 4.E6
#endif
#define TCD1304_MINCLKHZ 0.8E6

// =========================================

#define TDIFF(a,b) ((double)(a-b)/F_CPU)

// debug statements in setup
//#define DEBUG

// we'll comment this out for the new boards
#define SH_STOP_IN_READ

// clearing pulses, leave at 0 as default
#define SH_CLEARING_DEFAULT 0

#define ROUNDUP(a,b) (ceil((float)a/b)*b)
#define ROUNDTO(a,b) ((a/b)*b)
#define ROUNDTOMOD(a,b,c) (((a/b)*b)%c)

// cpu cycle counter values to seconds
// usage
#ifndef CYCCNT2SECS
#define CYCCNT2SECS(a) ((double)(a)/F_CPU)
#endif

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
#define CLK_IRQ IRQ_FLEXPWM2_0
#define CLK_CMPF_MASK CMPF_MASKA_ON

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
#define TRIGGER_PIN 2
//#define SPARE_PIN 3

#define SYNC_PIN_DEFAULT LOW
#define BUSY_PIN_DEFAULT HIGH

// Fast nofrills BUSY pin set, lear, flip state
/*
#define SETBUSYPIN (CORE_PIN1_PORTSET = CORE_PIN1_BITMASK)
#define CLEARBUSYPIN (CORE_PIN1_PORTCLEAR = CORE_PIN1_BITMASK)
#define TOGGLEBUSYPIN (CORE_PIN1_PORTTOGGLE = CORE_PIN1_BITMASK)
*/

// Fast nofrills SYNC pin set/clear
/*
#define SETSYNCPIN (CORE_PIN0_PORTSET = CORE_PIN0_BITMASK)
#define CLEARSYNCPIN (CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK)
#define TOGGLESYNCPIN (CORE_PIN0_PORTTOGGLE = CORE_PIN0_BITMASK)
*/
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

#define COUNTER_MAX_SECS ((float)(65535 * 128) / F_BUS_ACTUAL)

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
    Serial.print("Error: debugprintf vsprintf");
    Serial.println(format);
  }
  return n;
}
#define DEBUGPRINTF(...) debuprintf( __VA_ARGS__ )
#else
#define DEBUGPRINTF(...)
#endif

// ------------------------------------------------------------------------------
// NOTE this is not designed to be thread safe.
// (Consider making it a singleton)

/********************************************************************
 * TCD1304 Device implemented in FlexPWM2, pins 3,4,5,6 and 10
 */
typedef enum {
  NOTCONFIGURED,
  PULSE,
  FRAMESET,
  TIMER // we only have timer+pulse,  not timer+frameset
} TCD1304_Mode_t;

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
    IRQ_NUMBER_t irq;
    IMXRT_FLEXPWM_t *flexpwm;
    uint16_t period_counts = 0;
    uint16_t onA_counts = 0;
    uint16_t offA_counts = 0;
    uint16_t onB_counts = 0;
    uint16_t offB_counts = 0;
    uint16_t ctrl2_mask = 0;
    uint16_t intena_mask = 0;
    uint16_t divider = 1;
    uint8_t prescale = 0;
    uint8_t filler = 0;
    float period_secs = 0;

    bool invertA = false;
    bool invertB = false;

    bool newvals = false;

    void (*isr)() = nullptr;
    uint16_t inten_mask = 0;
    
    SubModule(const char *name_,
              uint8_t submod_, uint8_t mask_,
              uint8_t pinA_, uint8_t muxvalA_,
              uint8_t pinB_, uint8_t muxvalB_,
              IRQ_NUMBER_t irq_,
              IMXRT_FLEXPWM_t *flexpwm_) :
      name(name_),
      submod(submod_),mask(mask_),
      pinA(pinA_), muxvalA(muxvalA_),
      pinB(pinB_), muxvalB(muxvalB_),
      irq(irq_), flexpwm(flexpwm_) {};
  };
    
  inline static uint64_t sh_cyccnt64_start = 0;
  inline static uint64_t sh_cyccnt64_now = 0;
  inline static uint64_t sh_cyccnt64_prev = 0;
  inline static uint64_t sh_cyccnt64_exposure = 0;
  
  inline static uint64_t timer_cyccnt64_start = 0;
  inline static uint64_t timer_cyccnt64_now = 0;
  inline static uint64_t timer_cyccnt64_prev = 0;

  // For the trigger timer
  inline static uint64_t trigger_cyccnt64_start = 0;
  inline static uint64_t trigger_cyccnt64_now = 0;
  inline static uint64_t trigger_cyccnt64_prev = 0;

  // Refrenced by timer and trigger setups and by the header routine
  inline static TCD1304_Mode_t mode = NOTCONFIGURED;
  inline static bool trigger_mode = false;
  inline static bool trigger_attached = false;
  
  // bookkeeping counters
  inline static unsigned int sh_counter    = 0;
  inline static unsigned int icg_counter   = 0;
  inline static unsigned int cnvst_counter = 0;
  inline static unsigned int sh_counts_per_icg = 0;

  // bookkeeping for the charge clearing pulses on SH
  inline static unsigned int sh_clearing_counts = SH_CLEARING_DEFAULT;
  inline static unsigned int sh_clearing_counter = 0;
  inline static unsigned int sh_short_period_counts = 0;
  
  // read counters and callback
  inline static unsigned int read_counter = 0;
  inline static unsigned int read_counts  = 0;
  inline static uint16_t *read_buffer  = 0;
  inline static uint16_t *read_pointer = 0;
  inline static void (*read_callback)() = 0;

  // frame counters and frameset callback
  inline static unsigned int frame_counter = 0;
  inline static unsigned int frame_counts = 0;
  inline static void (*frames_completed_callback)() = 0;

  // framesets counters and framsets completed callback
  inline static unsigned int frameset_counter = 0;
  inline static unsigned int frameset_counts = 0;
  inline static void (*framesets_completed_callback)() = 0;

  // timer stops at timer_outer_counts, timer_counter is for extended timer
  inline static unsigned int timer_inner_counter = 0;
  inline static unsigned int timer_inner_counts = 0;
  inline static unsigned int timer_outer_counter = 0;
  inline static unsigned int timer_outer_counts = 0;
  inline static void (*timer_callback)() = 0;

  inline static float timer_interframe_min_secs = 0.;
  inline static float timer_period_secs = 0;     // inner loop period
  inline static float timer_interval_secs = .0;  // the resulting exposure interval
  
  // interrupt (trigger) support
  inline static unsigned int trigger_counter = 0;
  inline static unsigned int trigger_counts  = 1;
  inline static void (*trigger_callback)() = 0;
  
  inline static uint8_t trigger_pin = TRIGGER_PIN;
  inline static uint8_t trigger_edge_mode = RISING;
  inline static uint8_t trigger_pin_mode = INPUT;
 
  // timing adjustment
#ifdef  ALLINONEBOARD
  inline static uint16_t cnvst_extra_delay_counts = 0;   // best signal, steadily decreases with increasing delay
#else
  inline static uint16_t cnvst_extra_delay_counts = 1;   // this gives the lowest noise, does not improve at 2
#endif
  
  // This is for frameset with exposure == frame interval, we skip the first icg for readout
  inline static bool skip_one = false;
  inline static bool skip_one_reload = false;

  // State of the flexpwm interface
  inline static bool busytoggled = false;

  // Sync pin management
  inline static uint8_t sync_pin = SYNC_PIN;
  inline static bool synctoggled = false;
  inline static bool sync_enabled = true;

  // And now.... the submodules
  inline static IMXRT_FLEXPWM_t * const flexpwm = &IMXRT_FLEXPWM2;  
  inline static SubModule clk   = {"clk",CLK_SUBMODULE, CLK_MASK, CLK_PIN,CLK_MUXVAL, 0xFF,0, CLK_IRQ, &IMXRT_FLEXPWM2};
  inline static SubModule sh    = {"sh", SH_SUBMODULE, SH_MASK,  SH_PIN,SH_MUXVAL, 0xFF,0, SH_IRQ, &IMXRT_FLEXPWM2};
  inline static SubModule icg   = {"icg", ICG_SUBMODULE, ICG_MASK, ICG_PIN,ICG_MUXVAL, 0xFF,0, ICG_IRQ, &IMXRT_FLEXPWM2};
  inline static SubModule cnvst = {"cnvst", CNVST_SUBMODULE, CNVST_MASK, 0xFF,0, 0xFF,0, CNVST_IRQ, &IMXRT_FLEXPWM2}; // no pin 

  // this is our interval clock, implemented on PWM4. option for pin3 output.
  inline static IMXRT_FLEXPWM_t * const timerflexpwm = &IMXRT_FLEXPWM4;
  inline static SubModule timer = {"timer", TIMER_SUBMODULE, TIMER_MASK, 0xFF,0, TIMER_PIN,TIMER_MUXVAL, TIMER_IRQ, &IMXRT_FLEXPWM4};

  // error bookkeeping
  inline static bool error_flag = false;
  inline static bool oops_flag = false;


  //---------------------------------------------
  typedef struct Frame_Header_struct {
    uint16_t *buffer;
    unsigned int nbuffer;

    // from the dummy outputs, first 16 elements
    unsigned int avgdummy;
    float offset;

#ifdef DEBUG
    uint64_t sh_cyccnt64_start = 0;
    uint64_t sh_cyccnt64_now = 0;
    uint64_t sh_cyccnt64_prev = 0;
    uint64_t sh_cyccnt64_exposure = 0;
  
    uint64_t timer_cyccnt64_start = 0;
    uint64_t timer_cyccnt64_now = 0;
    uint64_t timer_cyccnt64_prev = 0;

    uint64_t trigger_cyccnt64_start = 0;
    uint64_t trigger_cyccnt64_now = 0;
    uint64_t trigger_cyccnt64_prev = 0;

    float frame_difference_secs;
 #endif
    
    float frame_elapsed_secs;
    float frame_exposure_secs;

    float timer_elapsed_secs;
    float timer_difference_secs;

    float trigger_elapsed_secs;
    float trigger_difference_secs;

    unsigned int frame_counter;
    unsigned int frameset_counter;
    unsigned int trigger_counter;

    TCD1304_Mode_t mode;
    bool trigger_mode;
    
    bool error_flag;
    bool oops_flag;

    bool frames_completed ;
    bool framesets_completed ;

    bool ready_for_send;
  } Frame_Header;

  
  //---------------------------------------------
  TCD1304Device(unsigned int period=CLK_DEFAULT)
  {    
    stop_with_irqs();

    pinMode(trigger_pin, trigger_pin_mode);

    pinMode(BUSY_PIN, OUTPUT);
    digitalWrite(BUSY_PIN, BUSY_PIN_DEFAULT);

    pinMode(SYNC_PIN, OUTPUT);
    digitalWrite(SYNC_PIN, SYNC_PIN_DEFAULT);

    pinMode(CNVST_PIN, OUTPUT);   
    digitalWrite(CNVST_PIN, LOW);

#ifdef PINPULLS    
    pinMode(CLK_PIN, INPUT_PULLDOWN);
    pinMode(SH_PIN, INPUT_PULLDOWN);
    pinMode(ICG_PIN, INPUT_PULLUP);
#endif

  }

  void stop_all()
  {
    stop_with_irqs();
    clear_sync_busy_pins();
  }
  
  static void stop_runs_only()
  {
    // Stops triggers
    stop_triggers();
        
    // Stop all four submodules
    flexpwm_stop();

    // Stop the timer
    timer_stop();
  }

  static void disable_irqs()
  {
    // Disable all of the interrupts
    NVIC_DISABLE_IRQ(CLK_IRQ);
    NVIC_DISABLE_IRQ(ICG_IRQ);
    NVIC_DISABLE_IRQ(SH_IRQ);
    NVIC_DISABLE_IRQ(CNVST_IRQ);

    // Disable the the timer interrupt
    NVIC_DISABLE_IRQ(TIMER_IRQ);
  }
  
  static void stop_with_irqs()
  {
    // stop all of the flexpwms
    stop_runs_only();   
    
    // Disable all of the interrupts
    disable_irqs();
  }

  static void clear_error_flags()
  {
    oops_flag = false;
    error_flag = false;
  }
  
  static void clear_mode()
  {
    mode = NOTCONFIGURED;
    trigger_mode = false;
  }

  static void clear_sync_busy_pins()
  {
    if (busytoggled) {
      busytoggled = false;
      digitalToggleFast(BUSY_PIN);
    }

    // reset sync pin
    if (synctoggled) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = false;
    }
  }

  static void toggle_busypin()
  {
    busytoggled = !busytoggled;
    digitalToggleFast(BUSY_PIN);
  }
  
  static void toggle_syncpin()
  {
    synctoggled = !synctoggled;
    digitalToggleFast(SYNC_PIN);
  }
  
  static void clear_busypin()
  {
    if (busytoggled) {
      busytoggled = false;
      digitalToggleFast(BUSY_PIN);
    }
  }
  
  static void clear_syncpin()
  {
    if (synctoggled) {
      synctoggled = false;
      digitalToggleFast(SYNC_PIN);
    }
  }
  
  /* ==========================================================================
     For ring buffering, we'll use this in the read callback to point to the next buffer
  */
  static void update_read_buffer(uint16_t *buffer)
  {
    read_buffer   = buffer;
    //read_counts   = nbuffer;
      
    read_pointer  = read_buffer;
    read_counter  = 0;
  }

  static void fill_frame_header(Frame_Header *p)
  {
    unsigned int utmp = 0;
    int n;
    
    p->buffer = read_buffer;
    p->nbuffer = read_counts;

    for (n=0; n<DATASTART; n++) {
      utmp += read_buffer[n];
    }    
    p->avgdummy = utmp/DATASTART;
    p->offset = ((float)utmp * VPERBIT)/DATASTART;

#ifdef DEBUG    
    p->sh_cyccnt64_start = sh_cyccnt64_start;
    p->sh_cyccnt64_now = sh_cyccnt64_now;
    p->sh_cyccnt64_prev = sh_cyccnt64_prev;
    p->sh_cyccnt64_exposure = sh_cyccnt64_exposure;
  
    p->timer_cyccnt64_start = timer_cyccnt64_start;
    p->timer_cyccnt64_now = timer_cyccnt64_now;
    p->timer_cyccnt64_prev = timer_cyccnt64_prev;

    p->trigger_cyccnt64_start = trigger_cyccnt64_start;
    p->trigger_cyccnt64_now = trigger_cyccnt64_now;
    p->trigger_cyccnt64_prev = trigger_cyccnt64_prev;

    p->frame_difference_secs = sh_difference_secs();
#endif
    
    p->frame_counter = frame_counter;
    p->frameset_counter = frameset_counter;

    p->trigger_counter = trigger_counter;

    p->frames_completed = (frame_counter+1) == frame_counts ? true : false;
    p->framesets_completed = p->frames_completed && ((frameset_counter+1) == frameset_counts) ? true : false;
    
    p->frame_elapsed_secs = sh_elapsed_secs();
    p->frame_exposure_secs = sh_exposure_secs();

    p->timer_elapsed_secs = timer_elapsed_secs();
    p->timer_difference_secs = timer_difference_secs();

    p->trigger_elapsed_secs = trigger_elapsed_secs();
    p->trigger_difference_secs = trigger_difference_secs();

    p->mode = mode;
    p->trigger_mode = trigger_mode;

    p->error_flag = error_flag;
    p->oops_flag = oops_flag;

    p->ready_for_send = true;
  }
  
  /* ==========================================================================
     High level API
   */

  inline static float read_expected_time = 0.;
  
  bool read(uint nframes, float exposure, uint16_t *bufferp,
            void (*frame_callbackf)(),
            void (*frameset_callbackf)(),
            void (*completion_callbackf)(),
            void (*setup_callbackf)(),
            bool start=true)
  {
    float clk_secs = 0.5E-6;
    float sh_secs = 1.0E-6;
    float sh_offset_secs = 0.6E-6;
    float icg_secs = 2.6E-6;
    float icg_offset_secs = 0.5E-6;

    if (!setup_pulse(clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs,
                     bufferp, NREADOUT, frame_callbackf)) {
      Serial.println("Error: failed to setup pulse");
      return false;
    }

    frame_counts = nframes;
    frames_completed_callback = frameset_callbackf;
    framesets_completed_callback = completion_callbackf;
    
    if (!setup_timer(exposure,0.,nframes)) {
      Serial.println("Error: failed to setup timer");
      return false;
    }

    if (setup_callbackf) {
      setup_callbackf();
    }
    
    if (!start) {
      return true;
    }

    // start the reads here
    timer_start();
    if (error_flag) {
      return false;
    }
    
    read_expected_time = exposure * nframes;
    
    return true;
  }
  
  bool read(uint nframes, float exposure, float frame_interval, uint16_t *bufferp,
            void (*frame_callbackf)(),
            void (*frameset_callbackf)(),
            void (*completion_callbackf)(),
            void (*setup_callbackf)(),
            bool start=true)
  {

    float clk_secs = 0.5E-6;
    float sh_secs = 1.0E-6;
    float sh_offset_secs = 0.6E-6;
    float icg_secs = 2.6E-6;
    float icg_offset_secs = 0.5E-6;

    if (!setup_frameset(clk_secs, sh_secs, sh_offset_secs, icg_secs, icg_offset_secs,
                        exposure, frame_interval, nframes,
                        bufferp, NREADOUT, frame_callbackf)) {
      
      Serial.println("Error: failed to setup frameset");
      return false;
    }
      
    frames_completed_callback = frameset_callbackf;
    framesets_completed_callback = completion_callbackf;

 
    if (setup_callbackf) {
      setup_callbackf();
    }
   
    if (!start) {
      return true;
    }

    // start the reads here
    frameset_start();
    if (error_flag) {
      return false;
    }
    
    read_expected_time = frame_interval * nframes;
    
    return true;
  }
  
  bool start_read()
  {
    if (mode == TIMER) {
      timer_start();
      return !error_flag;
    }
    else if (mode == FRAMESET) {
      frameset_start();
      return !error_flag;
    }

    Serial.println("Error: start_read, not configured");
    return false;
  }

  bool wait_read(float timeout=0., float timestep=0.01, bool verbose=false)
  {
    if (!timeout) timeout = read_expected_time - timer_elapsed_secs() + 0.010;
    if (timeout < 0.1) timeout = 0.1;

    if (mode == TIMER) {
      return timer_wait(timeout,timestep, verbose);
    }
    else if (mode == FRAMESET) {
      return flexpwm_wait(timeout,timestep, verbose);
    }

    Serial.println("Error: wait_read not configured");
    return false;
  }

  // -------------------------------------------------------------------------------------
  bool triggered_read(uint ntriggers, uint nframes, float exposure, uint16_t *bufferp,
                      void (*frame_callbackf)(),
                      void (*frameset_callbackf)(),
                      void (*completion_callbackf)(),
                      void (*setup_callbackf)(),
                      bool start=true)
  {
    if (!read(nframes, exposure, bufferp,frame_callbackf, frameset_callbackf, completion_callbackf, nullptr, false)) {
      Serial.println("Error: triggerd_read setup read");
      return false;
    }

    if (!setup_triggers(ntriggers)) {
      Serial.println("Error: triggerd_read setup triggers");
      return false;
    }

    if (setup_callbackf) setup_callbackf();
  
    if (start) {
      return start_triggers();
    }

    return true;    
  }
  
  bool triggered_read(uint ntriggers, uint nframes, float exposure, float interval, uint16_t *bufferp,
                      void (*frame_callbackf)(),
                      void (*frameset_callbackf)(),
                      void (*completion_callbackf)(),
                      void (*setup_callbackf)(),
                      bool start=true)
  {
    if (!read(nframes, exposure, interval, bufferp,
              frame_callbackf, frameset_callbackf, completion_callbackf, nullptr, false)) {
      Serial.println("Error: triggerd_read setup read");
      return false;
    }

    if (!setup_triggers(ntriggers)) {
      Serial.println("Error: triggerd_read setup trigger");
      return false;
    }

    if (setup_callbackf) setup_callbackf();

    if (start) {
      return start_triggers();
    }

    return true;    
  }

  bool wait_triggered_read(float timeout=1., float timestep=0.01, bool verbose=false)
  {
    if (wait_triggers(timeout,timestep,verbose)) {
      return wait_read(timeout,timestep,verbose);
    }

    return false;    
  }


  /* ==========================================================================
     64 bit elapsed time clock based on cpu cycles
     note that this is in the tcd1304 namespace
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

  /* ----------------------------------------------------
     Elapsed time from first frame (sh) to most recent (cf actual timer interval)
  */
  static double sh_elapsed_secs()
  {
    return (double)(sh_cyccnt64_now - sh_cyccnt64_start)/F_CPU;
  }

  static double sh_difference_secs()
  {
    return (double)(sh_cyccnt64_now - sh_cyccnt64_prev)/F_CPU;
  }

  static double sh_exposure_secs()
  {
    return (double)sh_cyccnt64_exposure/F_CPU;
  }
  /* ----------------------------------------------------
     Elapsed time from timer start to most recent timer interrupt
  */
  static double timer_elapsed_secs()
  {
    return (double)(timer_cyccnt64_now - timer_cyccnt64_start)/F_CPU;
  }

  static double timer_difference_secs()
  {
    return (double)(timer_cyccnt64_now - timer_cyccnt64_prev)/F_CPU;
  }
  
  /* ----------------------------------------------------
     Elapsed time from timer start to most recent timer interrupt
  */
  static double trigger_elapsed_secs()
  {
    return (double)(trigger_cyccnt64_now - trigger_cyccnt64_start)/F_CPU;
  }

  static double trigger_difference_secs()
  {
    return (double)(trigger_cyccnt64_now - trigger_cyccnt64_prev)/F_CPU;
  }

  /* =============================================================
     Print error message
  */
  static void print_errormsg(const char *name, const char *errmsg)
  {
    Serial.print("Error: ");
    Serial.print(name);
    Serial.print(" ");
    Serial.println(errmsg);
  }

  static void print_errormsg(SubModule *p, const char *errmsg)
  {
    print_errormsg(p->name,errmsg);
  }

  static void print_counters()
  {
    Serial.print(" sh "); Serial.print(sh_counter);
    Serial.print(" icg "); Serial.print(icg_counter);
    Serial.print(" cnvst "); Serial.print(cnvst_counter);
    Serial.print(" read "); Serial.print(read_counter);
    Serial.print(" "); Serial.print(read_counts);
    Serial.print(" frame "); Serial.print(frame_counter);
    Serial.print(" "); Serial.print(frame_counts);
    Serial.print(" frameset "); Serial.print(frameset_counter);
    Serial.print(" "); Serial.print(frameset_counts);
    if (mode==TIMER) {
      Serial.print(" timer "); Serial.print(timer_inner_counter);
      Serial.print(" "); Serial.print(timer_inner_counts);
      Serial.print(" outer "); Serial.print(timer_outer_counter);
      Serial.print(" "); Serial.print(timer_outer_counts);
    }
    if (trigger_mode) {
      Serial.print(" triggers "); Serial.print(trigger_counter);
      Serial.print(" "); Serial.print(trigger_counts);
    }
    Serial.println("");
  }

  
  /* ======================================================================================
     Print, check, setup submodule configuration, load into hardware
  */
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
              "FLEXPWM: %s submod %d mask 0x%x pinA %d muxA %d pinB %d muxB %d irq %d flexpwm %p",
              p->name, p->submod, p->mask, p->pinA, p->muxvalA, p->pinB, p->muxvalB, p->irq, p->flexpwm);
    Serial.println(cbuffer);
    
    snprintf( cbuffer, sizeof(cbuffer),
	      "flexpwm: %s period %u presc %d div %d => %.6g secs %.6g Hz ctrl2_mask x%02x",
	      p->name, p->period_counts, p->prescale, p->divider, period_secs, frequency, p->ctrl2_mask);
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

  // ------------------------------------------------------
  bool check_submodule(SubModule *p)
  {
    bool retv = true;
    if (p->divider<1) {
      print_errormsg(p->name, "divider < 1");
      retv = false;
    }
    if (p->divider>128) {
      print_errormsg(p->name, "divider > 128");
      retv = false;
    }
    
    if (p->onA_counts || p->offA_counts) {
      if (p->onA_counts>=p->offA_counts) {
        print_errormsg(p->name, "onA > offA");
        retv = false;
      }
      if (p->offA_counts>p->period_counts) {
        print_errormsg(p->name, "offA >= period");
        retv = false;
      }
    }
    
    if (p->onB_counts || p->offB_counts) {
      if (p->onB_counts>=p->offB_counts) {
        print_errormsg(p->name, "onB > offB");
        retv = false;
      }
      if (p->offB_counts>p->period_counts) {
        print_errormsg(p->name, "offB >= period");
        retv = false;
      }
    }
    
    return retv;
  }

  // ------------------------------------------------------
  bool print_and_check_submodule(SubModule *p)
  {
    print_submodule(p);
    return check_submodule(p);
  }

  // ------------------------------------------------------
  void load_submodule(SubModule *p)
  {
    IMXRT_FLEXPWM_t *q = p->flexpwm;
    unsigned int submod = p->submod;
    uint16_t mask = p->mask;
    
    Serial.print("loading sub_module ");
    Serial.println(p->name);

    // stop this channel - harmless if it was done globally already
    q->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    q->MCTRL &= ~FLEXPWM_MCTRL_RUN(mask);

    q->SM[submod].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(p->prescale);
      
    q->SM[submod].INIT = 0;      
    q->SM[submod].VAL0 = 0;
    q->SM[submod].VAL1 = p->period_counts - 1;    
    q->SM[submod].VAL2 = p->onA_counts;
    q->SM[submod].VAL3 = (p->offA_counts > 0) ? p->offA_counts: 0;
    q->SM[submod].VAL4 = p->onB_counts;
    q->SM[submod].VAL5 = (p->offB_counts > 0) ? p->offB_counts : 0;

    // Convenience, save the clock period in seconds
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
    q->SM[submod].CTRL2 = p->ctrl2_mask;

    p->newvals = false;
  }

  // -----------------------------------------------------------------------------
  bool setup_submodule( SubModule *p, uint8_t prescale, uint16_t period_counts,
                        uint16_t onA_counts, uint16_t offA_counts, bool invertA,
                        uint16_t onB_counts, uint16_t offB_counts, bool invertB,
                        uint16_t ctrl2_mask)
  {
    if (p) {
      p->divider = (1<<p->prescale);
      p->period_counts = period_counts;

      p->onA_counts = onA_counts;
      p->offA_counts = offA_counts;
      p->invertA = invertA;
    
      p->onB_counts = onB_counts;
      p->offB_counts = offB_counts;
      p->invertB = invertB;

      p->ctrl2_mask = ctrl2_mask;

      if (print_and_check_submodule(p)) {
        load_submodule(p);
        return true;
      }
    }    
    return false;
  }
  
  /* ==========================================================================
     Attach ISR and enable
     see above, CMPF_MASKA_ON, CMPF_MASKA_OFF, etc
  */
  void attach_isr( SubModule *p, uint16_t cmpf_mask, void (*isrf)())
  {
    IMXRT_FLEXPWM_t *q = p->flexpwm;
    unsigned int submod = p->submod;
    uint16_t status;

    // bookkeeping to support renabling
    p->inten_mask = cmpf_mask;
    p->isr = isrf;
    
    // disable the irq for this submodule
    NVIC_DISABLE_IRQ(p->irq);
    
    // clear all of this module's interrupt status bits
    status = q->SM[submod].STS;
    q->SM[submod].STS = status;

    // enable the specified bits only
    q->SM[submod].INTEN = cmpf_mask;
    p->intena_mask = cmpf_mask;

    // register the isr to this irq
    attachInterruptVector(p->irq, isrf);

    // and now, enable interrupts on this irq
    NVIC_ENABLE_IRQ(p->irq);  
  }


  /* =============================================================
     Setup callbacks for frames (a single frameset) completed.
  */
  void clear_frames_completed_callback()
  {
    frames_completed_callback = 0;
  }

  void load_frames_completed_callback(void (*callback)(), unsigned int nframes=0)
  {
    // allow for rearm callback. same nframes
    if (nframes) {
      frame_counts = nframes;
    }
    frames_completed_callback = callback;
  }

  /* =============================================================
     Setup callbacks for frameset completed.
  */
  void clear_framesets_completed_callback()
  {
    framesets_completed_callback = 0;
  }
  
  void load_framesets_completed_callback(void (*callback)(), unsigned int nsets=0)
  {
    // allow for rearm callback. same nframes
    if (nsets) {
      frameset_counts = nsets;
    }
    framesets_completed_callback = callback;
  }

  /* =======================================================================
     FlexPWM stop and wait
  */

  inline static bool flexpwm_running = false;

  static void flexpwm_start()
  {
    oops_flag = false;
    error_flag = false;
    flexpwm_running = true;
    flexpwm->MCTRL |= 0xF;   // set load ok for all four submodules
    flexpwm->MCTRL |= ((CLK_MASK|SH_MASK|ICG_MASK) << 8); // set run clk, sh, icg
  }
  
  static void flexpwm_stop()
  {
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(0xF);
    flexpwm->MCTRL = 0; // stop everything
    flexpwm_running = false;
  }

  bool flexpwm_wait(float timeout_=1., float timestep_=0.01, bool verbose=false)
  {
    uint32_t timeout = (uint) (timeout_ * 1000);
    uint32_t increment = (uint) (timestep_ * 1000);
    uint32_t elapsed = 0;

    if (verbose) {
      Serial.print("flexpwm_wait "); Serial.print(timeout);
      Serial.print(" "); Serial.println(increment);
    }
    
    while(flexpwm_running && !error_flag && elapsed < timeout) {
      delay(increment);
      elapsed += increment;
    }
    if (error_flag) {
      Serial.println("Error: flexpwm_wait error_flag is set.");
      return false;
    }
    if (elapsed >= timeout) {
      Serial.print("Error: flexpwm_wait timeout" );
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
      return false;
    }
    if (flexpwm_running) {
      Serial.print("Error: flexpwm_wait oops!  Return with frameset still running");
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
      return false;
    }

    if (verbose) {    
      Serial.print("Success: flexpwm_wait");
      Serial.print(" running "); Serial.print(flexpwm_running);
      Serial.print(" error "); Serial.print(error_flag);
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
    }
    
    return true;
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Configure the flexpwm for single pulse, use this with a timer
  ////////////////////////////////////////////////////////////////////////////////////////////

  inline static bool pulse_armed = false;

  
  static void pulse_sh_isr()
  {
    uint16_t status;

    bool do_sync_toggle_here = false;
    
    // this is a trailing edge interrupt := exposure timer
    uint64_t cyccnt64_now = cycles64();
    
    // clear the interrupt
    status = flexpwm->SM[SH_SUBMODULE].STS;
    flexpwm->SM[SH_SUBMODULE].STS = status;

#if 1
    // ======================================
    // no clearing pulses
    if (!sh_clearing_counts) {
      // no more sh interrupts
      flexpwm->SM[SH_SUBMODULE].INTEN = 0;

      // never on again until next start
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
      flexpwm->SM[SH_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);

      do_sync_toggle_here = true;
    }

    // first pulse
    else if (!sh_clearing_counter) {      
      /* shorten the period, this will happen
	 after completing the present period. */
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
      flexpwm->SM[SH_SUBMODULE].VAL1 = sh_short_period_counts;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);

      // this is the end of the previous exposure
      do_sync_toggle_here = true;

      // need to count
      sh_clearing_counter++;
    }

    // last pulse?
    else if (++sh_clearing_counter == sh_clearing_counts) {
      // no more sh interrupts
      flexpwm->SM[SH_SUBMODULE].INTEN = 0;

      // never on again until next start
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
      flexpwm->SM[SH_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);

      sh_clearing_counter = 0;

      // this is the start of the next exposure
      do_sync_toggle_here = true;
    }
    
    // for elapsed time and exposure
    sh_cyccnt64_prev = sh_cyccnt64_now;
    sh_cyccnt64_now  = cyccnt64_now;

    // sync is toggled on start and end of exposure
    if (sync_enabled && do_sync_toggle_here) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = !synctoggled;
    }
    
    // ======================================
#else
    sh_clearing_counter++;

    // Either no clearing pulses, or clearing is done
    if (sh_clearing_counter >= sh_clearing_counts) {
      // no more sh interrupts
      flexpwm->SM[SH_SUBMODULE].INTEN = 0;

      // never on again until next start
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
      flexpwm->SM[SH_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);
      
      sh_clearing_counter = 0;
    }

    // First clearing pulse
    else if (sh_clearing_counter == 1) {
      /* shorten the period, this will happen
	 after completing the present period. */
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
      flexpwm->SM[SH_SUBMODULE].VAL1 = sh_short_period_counts;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);
    }
    
    // for elapsed time and exposure
    sh_cyccnt64_prev = sh_cyccnt64_now;
    sh_cyccnt64_now  = cyccnt64_now;
    
    // sync toggles on trailing edge of SH
    if (sync_enabled) {
      digitalToggleFast(SYNC_PIN);
      synctoggled = !synctoggled;
    }
#endif

    // bookkeeping
    sh_counter++;
    
    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.println("OOPS! pulse sh without off bit set");
    }
  }

  static void pulse_icg_isr()
  {
    uint16_t status;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    //Serial.print("icg_isr "); print_counters();
    
    // ---------------------------------------------
    // Start the cnvst clock
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);
    // ---------------------------------------------
    
    // no more icg interrupts
    flexpwm->SM[ICG_SUBMODULE].INTEN = 0;
    
    // never on again
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(ICG_MASK);
    flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(ICG_MASK);

    flexpwm->SM[CLK_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;  // force out while ldok, initializes the counters

    // here is our exposure, diff to most recent previous sh gate    
    sh_cyccnt64_exposure = sh_cyccnt64_now - sh_cyccnt64_prev;
    
    // bookkeeping
    icg_counter++;

    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.println("OOPS! pulse icg without off bit set");
    }
  }
  
  static void pulse_cnvst_isr()
  {
    uint16_t status;

    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;

    if ((status&CMPF_MASKA_OFF)) {

      if (read_counter < read_counts) {

#ifdef ALLINONEBOARD
        adc->adc0->startReadFast(ANALOGPIN);
        while ( adc->adc0->isConverting() );
        *read_pointer = adc->adc0->readSingle();
#else      
        // Assert the convert pin
        SETCNVST;
        delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
        CLEARCNVST;               // need 30 nanoseconds after this

        *read_pointer = SPI.transfer16(0xFFFF);
        *read_pointer ^= (0x1<<15);
#endif

        // bookkeeping for the read
        read_pointer++;
        read_counter++;

        // The read is done
        if (read_counter == read_counts) {

          // no more cnvst interrupts
          flexpwm->SM[CNVST_SUBMODULE].INTEN = 0;

          // Stop everything on this flexpwm
          // ** Indicate stop only after callback **
          flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(0xF);  // clear ldok for all submodules
          flexpwm->MCTRL = 0;                          // clear run for all submodules

          // ----------------------------------------
          // User supplied function, per read complete
          // Recommend that the user ignore frame 0
          if (read_callback) {
            read_callback();
          }

          // ** Now we can indicate stopped **
          flexpwm_running = false;
          
          // =============================================
          // Frame bookkeeping, re-arm etc
          frame_counter++;

          // More frames?
          if (frame_counter < frame_counts) {
            pulse_arm();
          }

          // Nope, we're done with this frameset
          else if (frame_counter == frame_counts) {
          
	    if (frames_completed_callback) {
	      frames_completed_callback();
	    }

            // frameset completed, update frameset counter
            frameset_counter++;

            // More framesets?
            if (frameset_counter < frameset_counts) {
              pulse_init_frames();  // init the icg counter too.
            }

            // Nope, we're done with all of the framesets
            else if (frameset_counter == frameset_counts) {
              
              if (framesets_completed_callback) {
                framesets_completed_callback();
              }

              // reset sync pin
              if (synctoggled) {
                digitalToggleFast(SYNC_PIN);
                synctoggled = false;
              }

              // and get ready for the next frame set
              pulse_init_frameset();              
            }
            
            // oops, shouldn't get here
            else {
              oops_flag = true;
              Serial.println("Oops!  cnvst interrupt after frameset_counts complete");
            }
                  
          }

          // oops, shouldn't get here
          else {
            oops_flag = true;
            Serial.println("Oops!  cnvst interrupt after frame_counts complete");
          }

        }        

      }
      
      // oops, shouldn't get here
      else {
        oops_flag = true;
        Serial.println("OOPS! pulse cnvst interrupt after read_counts complete");        
      }
      
    }

    // oops, shouldn't get here
    else {
      oops_flag = true;
      Serial.print("OOPS! pulse cnvst without off bit set ");
      Serial.println(status,HEX);
    }
    
    // diagnostics
    cnvst_counter++;
    
  }

  static void pulse_start()
  {
    if (pulse_armed) {
      sh_clearing_counter = 0;
      flexpwm_start();
    }
    else {
      stop_runs_only();
      Serial.println("Error: pulse_start() - but not armed");
      error_flag = true;
    }
  }

  static void pulse_arm()
  {
    uint16_t status;

    if (!busytoggled) {
      busytoggled = true;
      digitalToggleFast(BUSY_PIN);
    }

    read_pointer = read_buffer;
    read_counter = 0;

    sh_clearing_counter = 0;
    
    // Restore the SH submodule counter and interrupt enable
    status = flexpwm->SM[SH_SUBMODULE].STS;
    flexpwm->SM[SH_SUBMODULE].STS = status;

    flexpwm->SM[SH_SUBMODULE].INTEN = CMPF_MASKA_OFF;

    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
    flexpwm->SM[SH_SUBMODULE].VAL1 = sh.period_counts - 1;    
    flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;
    flexpwm->SM[SH_SUBMODULE].VAL3 = sh.offA_counts;
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);

    // Restore the ICG submodule counter and interrupt enable
    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    flexpwm->SM[ICG_SUBMODULE].INTEN = CMPF_MASKA_OFF;

    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(ICG_MASK);
    flexpwm->SM[ICG_SUBMODULE].VAL2 = icg.onA_counts;
    flexpwm->SM[ICG_SUBMODULE].VAL3 = icg.offA_counts;
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(ICG_MASK);
  
    // Restore the CNVST submodule interrupt enable
    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;
    
    flexpwm->SM[CNVST_SUBMODULE].INTEN = CMPF_MASKA_OFF;

#ifdef DEBUG
    Serial.println("pulse armed");
#endif
    pulse_armed = true;
  }

  static void pulse_init_frames()
  {
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;
    
    sh_counter    = 0;
    icg_counter   = 0;
    cnvst_counter = 0;

    sh_clearing_counter = 0;
    
    frame_counter = 0;
    
    pulse_arm();

#ifdef DEBUG
    Serial.println("pulse init frames");
#endif
  }

  static void pulse_init_frameset()
  {
    pulse_init_frames();
    
    frameset_counter = 0;
    
    pulse_arm();
  }

  bool setup_pulse(float clk_secs, float sh_secs, float sh_offset_secs, float icg_secs, float icg_offset_secs,
                   uint16_t *buffer, unsigned int nbuffer, void (*callbackf)())
  {
    char cbuffer[128] = {0};
    unsigned int cnvst_delay = 1;

    // stop everything and disconnect irqs
    stop_with_irqs();

    // clear the error flags
    clear_error_flags();

    // clear run and trigger mode
    clear_mode();

    // clear sync and busy pins
    clear_sync_busy_pins();    


    sprintf(cbuffer,
            "tcd1304 setup pulse with clk %.5gs sh %.5gs offset %.5gs icg offset %.5gs",
	    clk_secs, sh_secs, sh_offset_secs, icg_offset_secs);
    Serial.println(cbuffer);
    
    // clear elapsed times
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;

    timer_cyccnt64_start   = 0;
    timer_cyccnt64_prev    = 0;
    timer_cyccnt64_now     = 0;
    
    trigger_cyccnt64_start   = 0;
    trigger_cyccnt64_prev    = 0;
    trigger_cyccnt64_now     = 0;
    
    // clear bookkeeping counters
    sh_counter    = 0;
    icg_counter   = 0;
    cnvst_counter = 0;

    sh_clearing_counter = 0;
    
    // default to 1 frame and 1 frameset
    frame_counter = 0;
    frame_counts = 1;

    frameset_counter = 0;
    frameset_counts = 1;

    // ------------------------------------------------------------------
    // Setup the master clock
    
    if (clk_secs < 0.25E-6 || clk_secs > 1.25E-6) {
      sprintf(cbuffer,
              "Error: setup_flexpwm_single_sequence with clk interval %.6g secs out of range 1E-6 to 5.E-6",
              clk_secs);
      Serial.println(cbuffer);
      return false;
    }
    
    clk.prescale = 0;
    clk.divider   = (1<<clk.prescale);

    clk.period_counts = ceil(clk_secs*F_BUS_ACTUAL/clk.divider);
    
    clk.onA_counts    = 0;
    clk.offA_counts   = (clk.period_counts/2);
    clk.invertA       = false;

    clk.onB_counts    = 0;
    clk.offB_counts   = 0;
    clk.invertB       = false;

    clk.inten_mask    = 0;

    clk.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER;
    
    // ------------------------------------------------------------------
    // Setup the CNVST clock, 4 times the clock period,
    cnvst.prescale = clk.prescale;
    cnvst.divider = (1<<cnvst.prescale);
    
    cnvst.period_counts = 4*clk.period_counts;

    cnvst.onA_counts    = (cnvst_delay + cnvst_extra_delay_counts)*clk.period_counts;    // allow extra delay for testing
    cnvst.offA_counts   = cnvst.onA_counts + ceil(CNVST_PULSE_SECS*F_BUS_ACTUAL/cnvst.divider);
    cnvst.invertA       = false;

    cnvst.onB_counts    = 0;
    cnvst.offB_counts   = 0;
    cnvst.invertB       = false;
    
    //cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_SYNC;
    cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER; // needs to be master, because run bit is disabled when running from clk's clock.
    
    // ------------------------------------------------------------------
    // setup the sh, sh_offset-icg_offset is the icg-sh timing, runs once, isr has to set on,off to 0xffff
    sh.prescale = clk.prescale;
    sh.divider = (1<<sh.prescale);

    sh.period_counts  = 0x8000;
    sh.onA_counts     = ceil(sh_offset_secs*F_BUS_ACTUAL/sh.divider);
    sh.offA_counts    = sh.onA_counts + ceil(sh_secs*F_BUS_ACTUAL/sh.divider);
    sh.invertA        = false;

    sh.onB_counts     = 0;
    sh.offB_counts    = 0;
    sh.invertB        = false;

    sh.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //sh.ctrl2_mask     = FLEXPWM_SMCTRL2_CLK_SEL(0x2);
 
    // ------------------------------------------------------------------
    // setup icg,  icg interrupt has to stop sh and icg by moving val2  to > val1
    icg.prescale = clk.prescale;
    icg.divider = (1<<icg.prescale);

    icg.period_counts = 0x8000;
    icg.onA_counts = ceil(icg_offset_secs*F_BUS_ACTUAL/icg.divider);
    icg.offA_counts = icg.onA_counts + ceil(icg_secs*F_BUS_ACTUAL/icg.divider);
    icg.invertA       = true;

    icg.onB_counts    = 0;
    icg.offB_counts   = 0;
    icg.invertB       = false;

    icg.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //icg.ctrl2_mask    = FLEXPWM_SMCTRL2_CLK_SEL(0x2);

    // if we are going to run some clearing cycles on the shift gate
    if (sh_clearing_counts) {
      // make sure it comes after the icg isr and before the first cnvst isr
      sh.period_counts = icg.offA_counts + clk.period_counts;
    }

    // if we go to short clearing cycles, this is the period
    sh_short_period_counts = 2*(sh.offA_counts - sh.onA_counts) + 1;
    if (sh_short_period_counts <= sh.offA_counts) {
      sh_short_period_counts = sh.offA_counts + 1;
    }
    
    // ------------------------------------------------------------------
    Serial.println("");
    if (!print_and_check_submodule(&clk)) {
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&icg)) {
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&sh)) {
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&cnvst)) {
      return false;
    }

    Serial.println("");
    Serial.print("flexpwm: clearing_pulses ");
    Serial.println(sh_clearing_counts);
   
    // ------------------------------------------------------------------
    load_submodule(&clk);
    load_submodule(&icg);
    load_submodule(&sh);
    load_submodule(&cnvst);

    // ---------------------------------------------
    // Setup the readout buffer
    read_buffer   = buffer;
    read_counts   = nbuffer;
      
    read_pointer  = read_buffer;
    read_counter  = 0;

    read_callback = callbackf;
    
    // ---------------------------------------------
    attach_isr( &sh, CMPF_MASKA_OFF, pulse_sh_isr);
    attach_isr( &icg, CMPF_MASKA_OFF, pulse_icg_isr);
    attach_isr( &cnvst, CMPF_MASKA_OFF, pulse_cnvst_isr);
      
    // ---------------------------------------------
    // stuff that other routines need to know
    mode = PULSE;
    pulse_armed  = true;
    trigger_mode = false;
    
    // timer needs this
    timer_interframe_min_secs  = (icg.offA_counts/F_BUS_ACTUAL)*icg.divider;    // offset before read starts
    timer_interframe_min_secs += ((float)cnvst.period_counts/F_BUS_ACTUAL)*cnvst.divider * read_counts;  // read time
    timer_interframe_min_secs += USBTRANSFERSECS;

    // round up to 1 msec
    timer_interframe_min_secs = ceil((timer_interframe_min_secs+1.E-3)/1.E-3)*1.E-3;

    Serial.print("TCD1304 setup_pulse success, min frame interval ");
    Serial.print(timer_interframe_min_secs,6);
    Serial.println(" secs");
    
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Frameset, short exposure times
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  inline static bool frameset_armed = false;
  
  static void frameset_sh_isr()
  {
    uint16_t status;
    unsigned int remainder;

    // this is a trailing edge interrupt := exposure timer
    uint64_t cyccnt64_now = cycles64();
    
    // clear the interrupt
    status = flexpwm->SM[SH_SUBMODULE].STS;
    flexpwm->SM[SH_SUBMODULE].STS = status;

    // this is where we are relative to the icg pulse
    remainder = sh_counter % sh_counts_per_icg;

    // one test for both pulses beginning and ending the sampling interval
    if (remainder <= 1) {
      if (sync_enabled) {
        digitalToggleFast(SYNC_PIN);
        synctoggled = !synctoggled;
      }
      sh_cyccnt64_prev = sh_cyccnt64_now;
      sh_cyccnt64_now  = cyccnt64_now;
      sh_cyccnt64_exposure = cyccnt64_now - sh_cyccnt64_prev;
    }
    
    // bookkeeping
    sh_counter++;
    
    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.println("OOPS! pulse sh without off bit set");
    }
  }

  static void frameset_icg_isr()
  {
    uint16_t status;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    //Serial.print("icg_isr "); print_counters();
    
    // ---------------------------------------------
    // Start the cnvst clock
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);
    // ---------------------------------------------

    // bookkeeping
    icg_counter++;

    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.println("OOPS! pulse icg without off bit set");
    }
  }

  static void frameset_cnvst_isr()
  {
    uint16_t status;

    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;

    if ((status&CMPF_MASKA_OFF)) {

      if (read_counter < read_counts) {

#ifdef ALLINONEBOARD
        adc->adc0->startReadFast(ANALOGPIN);
        while ( adc->adc0->isConverting() );
        *read_pointer = adc->adc0->readSingle();
#else      
        // Assert the convert pin
        SETCNVST;
        delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
        CLEARCNVST;               // need 30 nanoseconds after this

        *read_pointer = SPI.transfer16(0xFFFF);
        *read_pointer ^= (0x1<<15);
#endif

        // bookkeeping for the read
        read_pointer++;
        read_counter++;

        // The read is done
        if (read_counter == read_counts) {

          // stop cnvst
          flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(CNVST_MASK);

          // ----------------------------------------
          // User supplied function, per read complete
          // Recommend that the user ignore frame 0
          if (read_callback) {
            read_callback();
          }
          
          // =========================================
          // Frame bookkeeping
          frame_counter++;

          // More frames?
          if (frame_counter < frame_counts) {
            frameset_arm();
          }
          
          // Are we done with this frameset?
          else if (frame_counter == frame_counts) {

            // stop everything on this flexpwm
            flexpwm_stop();

	    if (frames_completed_callback) {
	      frames_completed_callback();
	    }

            // frameset completed, update frameset counter
            frameset_counter++;

            // More framesets?
            if (frameset_counter < frameset_counts) {
              frameset_init_frames();  // init the icg counter too.
            }

            // Nope, we're done with all of the framesets
            else if (frameset_counter == frameset_counts) {
              
              if (framesets_completed_callback) {
                framesets_completed_callback();
              }

              // reset sync pin
              if (synctoggled) {
                digitalToggleFast(SYNC_PIN);
                synctoggled = false;
              }

              // and get ready for the next frame set
              frameset_init_frameset();

            }

            // oops, shouldn't get here
            else {
              oops_flag = true;
              Serial.println("Oops!  cnvst interrupt after frameset_counts complete");
            }
                  
          }

          // oops, shouldn't get here
          else if (frame_counter > frame_counts) {
            oops_flag = true;
            Serial.println("Oops!  cnvst interrupt after frame_counts complete.");
          }
        }
      }
      
      // oops, shouldn't get here
      else {
        oops_flag = true;
        Serial.println("OOPS! pulse cnvst interrupt after read_counts complete");        
      }
      
    }

    // oops, shouldn't get here
    else {
      oops_flag = true;
      Serial.print("OOPS! pulse cnvst without off bit set ");
      Serial.println(status,HEX);
    }
    
    // diagnostics
    cnvst_counter++;
    
  }
    

  static void frameset_start()
  {
    if (flexpwm_running) {
      error_flag = true;
      Serial.println("Error: framset already running");
      return;
    }

    if (!frameset_armed) {
      error_flag = true;
      Serial.println("Error: framset not armed");
      return;
    }
    
    timer_cyccnt64_start = cycles64();
    timer_cyccnt64_prev  = timer_cyccnt64_start;
    timer_cyccnt64_now   = timer_cyccnt64_start;

    // start the clk, sh, icg
    flexpwm_start();    
  }


  // -------------------------------------------------
  static void frameset_arm()
  {
    if (!busytoggled) {
      busytoggled = true;
      digitalToggleFast(BUSY_PIN);
    }

    read_pointer = read_buffer;
    read_counter = 0;

    frameset_armed = true;
  }

  static void frameset_init_frames()
  {
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;
    
    sh_counter    = 0;
    icg_counter   = 0;
    cnvst_counter = 0;

    frame_counter = 0;
    
    sh_clearing_counter = 0;
    
    frameset_arm();

  }

  static void frameset_init_frameset()
  {
    frameset_init_frames();
    
    frameset_counter = 0;
    
    frameset_arm();
  }
  
  bool setup_frameset(float clk_secs, float sh_secs, float sh_offset_secs, float icg_secs, float icg_offset_secs,
                      float exposure_secs, float frame_interval_secs, unsigned int nframes,
                      uint16_t *buffer, unsigned int nbuffer, void (*callbackf)())
  {
    //char cbuffer[128] = {0};
    unsigned int cnvst_delay = 1;

    unsigned int prescale = 0;
    unsigned int divider = 1;

    float clock_period_secs;
    
    unsigned int clock_period_counts;    
    unsigned int exposure_period_counts;
    unsigned int frame_period_counts;

    
    // ------------------------------------------------------------------
    // stop everything and disable irqs
    stop_with_irqs();

    // clear sync and busy pins
    clear_sync_busy_pins();
    
    // clear the error flags
    clear_error_flags();

    // clear run mode and trigger mode
    clear_mode();

    // ------------------------------------------------------------------
    // announce ourselves
    Serial.print("#tcd1304 setup frameset, clk "); Serial.print(clk_secs,8);
    Serial.print(" sh "); Serial.print(sh_secs,8);
    Serial.print(" offset "); Serial.print(sh_offset_secs,8);
    Serial.print(" icg "); Serial.print(icg_secs,8);
    Serial.print(" offset "); Serial.print(icg_offset_secs,8);
    Serial.print(" exposure "); Serial.print(exposure_secs,6);
    Serial.print(" interval "); Serial.print(frame_interval_secs,6);
    Serial.print(" frames "); Serial.println(nframes);

    // ------------------------------------------------------------------
    // clear elapsed times
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;

    timer_cyccnt64_start   = 0;
    timer_cyccnt64_prev    = 0;
    timer_cyccnt64_now     = 0;
    
    trigger_cyccnt64_start   = 0;
    trigger_cyccnt64_prev    = 0;
    trigger_cyccnt64_now     = 0;
    
    // clear bookkeeping counters
    sh_counter    = 0;
    icg_counter   = 0;
    cnvst_counter = 0;

    sh_clearing_counter = 0;
    
    // default to 1 frame and 1 frameset
    frame_counter = 0;
    //frame_counts = nframes ? nframes : 10;
    frame_counts = nframes ? nframes+1 : 10;

    frameset_counter = 0;
    frameset_counts = 1;

    // ------------------------------------------------------------------
    // Some basic checks
    if (exposure_secs*2 > frame_interval_secs) {
      Serial.println("Error: requested exposure > requested frame interval/2");
      error_flag = true;
      return false;
    }

    if (clk_secs > exposure_secs) {
      Serial.println("Error: requested clk > requested exposure");
      error_flag = true;
      return false;
    }
    
    if (clk_secs < 0.25E-6) {
      Serial.println("Error: requested clk < 0.25E-6 (4MHz)");
      error_flag = true;
      return false;
    }

    if (clk_secs > 1.25E-6) {
      Serial.println("Error: requested clk > 1.25E-6 (800kHZ)");
      error_flag = true;
      return false;
    }
    
    // ------------------------------------------------------------------
    // need to find divider for the frame interval
    while (frame_interval_secs * (F_BUS_ACTUAL/divider) > 65533) {  // reserve two time slots, 0xFFFF - 2
      if (prescale >= 7) {
        Serial.println("Error: requested frame interval is too large");
        error_flag = true;
        return false;
      }
      prescale++;
      divider = (1<<prescale);
    }
    Serial.print("#Exposure prescale "); Serial.print(prescale);
    Serial.print(" divider "); Serial.println(divider);
    
    clock_period_counts = ceil(clk_secs*F_BUS_ACTUAL);
    clock_period_counts = ceil(clock_period_counts/divider) * divider;
    clock_period_secs   = (float)clock_period_counts/F_BUS_ACTUAL;
    Serial.print("#Clock period counts "); Serial.print(clock_period_counts);
    Serial.print(" secs "); Serial.println(clock_period_secs,8);
    
    if ((clock_period_counts < 4)|| (clock_period_secs > (1./TCD1304_MINCLKHZ))|| (clock_period_secs < (1./TCD1304_MAXCLKHZ))) {
      Serial.println("Error: not able to support this combination of clock and exposure times.");
      error_flag = true;
      return false;
    }
    Serial.print("#Clock period counts "); Serial.println(clock_period_counts);    

    exposure_period_counts = ceil(exposure_secs*F_BUS_ACTUAL);
    exposure_period_counts = (exposure_period_counts/clock_period_counts) * clock_period_counts; // already a multple of divider
    Serial.print("#Exposure period counts "); Serial.println(exposure_period_counts);    

    frame_period_counts = ceil(frame_interval_secs*F_BUS_ACTUAL);
    frame_period_counts = (frame_period_counts/exposure_period_counts) * exposure_period_counts; // already a multple of divider
    Serial.print("#Frame period counts "); Serial.println(frame_period_counts);


    sh_counts_per_icg = frame_period_counts/exposure_period_counts;
    Serial.print("#Exposure to frame ratio "); Serial.println((float)frame_period_counts/exposure_period_counts);
    
    // ------------------------------------------------------------------
    // Setup the master clock, always with divider = 1    
    clk.prescale      = 0;
    clk.divider       = 1;

    clk.period_counts = clock_period_counts;
    
    clk.onA_counts    = 0;
    clk.offA_counts   = (clk.period_counts/2);
    clk.invertA       = false;

    clk.onB_counts    = 0;
    clk.offB_counts   = 0;
    clk.invertB       = false;

    clk.inten_mask    = 0;

    clk.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER;
    
    // ------------------------------------------------------------------
    // Setup the CNVST clock, 4 times the clock period, on the same clock divider
    cnvst.prescale      = clk.prescale;
    cnvst.divider       = clk.divider;
    
    cnvst.period_counts = 4*clk.period_counts;

    cnvst.onA_counts    = (cnvst_delay + cnvst_extra_delay_counts)*clk.period_counts;    // allow extra delay for testing
    cnvst.offA_counts   = cnvst.onA_counts + ceil(CNVST_PULSE_SECS*F_BUS_ACTUAL/cnvst.divider);
    cnvst.invertA       = false;

    cnvst.onB_counts    = 0;
    cnvst.offB_counts   = 0;
    cnvst.invertB       = false;
    
    //cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_SYNC;
    cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER; // needs to be master, because run bit is disabled when running from clk's clock.
    
    // ------------------------------------------------------------------
    // setup the sh, sh_offset-icg_offset is the icg-sh timing, runs once, isr has to set on,off to 0xffff
    sh.prescale       = prescale;
    sh.divider        = divider;

    sh.period_counts  = exposure_period_counts/divider;
      
    sh.onA_counts     = ceil(sh_offset_secs*F_BUS_ACTUAL/sh.divider);
    sh.offA_counts    = sh.onA_counts + ceil(sh_secs*F_BUS_ACTUAL/sh.divider);
    sh.invertA        = false;

    sh.onB_counts     = 0;
    sh.offB_counts    = 0;
    sh.invertB        = false;

    sh.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //sh.ctrl2_mask     = FLEXPWM_SMCTRL2_CLK_SEL(0x2);
 
    // ------------------------------------------------------------------
    // setup icg,  icg interrupt has to stop sh and icg by moving val2  to > val1
    icg.prescale      = prescale;
    icg.divider       = divider;

    icg.period_counts = frame_period_counts/divider;

    // icg happens on the second sh pulse
    icg.onA_counts    = sh.period_counts + ceil(icg_offset_secs*F_BUS_ACTUAL/icg.divider);
    icg.offA_counts   = icg.onA_counts + ceil(icg_secs*F_BUS_ACTUAL/icg.divider);
    icg.invertA       = true;

    icg.onB_counts    = 0;
    icg.offB_counts   = 0;
    icg.invertB       = false;

    icg.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //icg.ctrl2_mask    = FLEXPWM_SMCTRL2_CLK_SEL(0x2);

    // ------------------------------------------------------------------
    Serial.println("");
    if (!print_and_check_submodule(&clk)) {
      error_flag = true;
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&icg)) {
      error_flag = true;
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&sh)) {
      error_flag = true;
      return false;
    }

    Serial.println("");
    if (!print_and_check_submodule(&cnvst)) {
      error_flag = true;
      return false;
    }
   
    // ------------------------------------------------------------------
    load_submodule(&clk);
    load_submodule(&icg);
    load_submodule(&sh);
    load_submodule(&cnvst);

    // ---------------------------------------------
    // Setup the readout buffer
    read_buffer   = buffer;
    read_counts   = nbuffer;
      
    read_pointer  = read_buffer;
    read_counter  = 0;

    read_callback = callbackf;
    
    // ---------------------------------------------
    attach_isr( &sh, CMPF_MASKA_OFF, frameset_sh_isr);
    attach_isr( &icg, CMPF_MASKA_OFF, frameset_icg_isr);
    attach_isr( &cnvst, CMPF_MASKA_OFF, frameset_cnvst_isr);
      
    // ---------------------------------------------
    // stuff that other routines need to know
    mode            = FRAMESET;
    trigger_mode    = false;

    frameset_armed  = true;
     
    return true;
  }

  // ================================================================================
  //#define DEBUG_TCD1304_TIMER
  
  inline static bool timer_first_time_flag = true;
  inline static bool timer_running = false;
  
  static void timer_isr()
  {
    volatile uint16_t status;

    uint64_t cyccnt64_now = cycles64();
    
    status = timerflexpwm->SM[TIMER_SUBMODULE].STS;
    timerflexpwm->SM[TIMER_SUBMODULE].STS = status;
      
    // spurious first invocation
    if (!status) {
      if (timer_inner_counter) {
        error_flag = true;
        timer_stop();
        Serial.println("Error: OOPS! pulse timer with null status, stopping ");
      }
      return;
    }

    timer_inner_counter++;

    // competed the inner counter, call the callback
    if (timer_first_time_flag || timer_inner_counter == timer_inner_counts) {

      timer_cyccnt64_prev = timer_cyccnt64_now;
      timer_cyccnt64_now  = cyccnt64_now;
      
      if (timer_callback) {
        timer_callback();
      }

      // Reset the inner counter
      timer_inner_counter = 0;

      // clear the first time flag
      timer_first_time_flag = false;
      
      // Update and check the outer counter, are we done?
      timer_outer_counter++;
      if (timer_outer_counter == timer_outer_counts) {
        // stop the timer only, we might be interrupt driven
        timer_stop();
      }            
    }

    // diagnostics
    if (!(status&timer.intena_mask)) {
      timer_stop();
      error_flag = true;
      Serial.print("Error: OOPS! pulse timer without the right bit, status ");
      Serial.println(status,HEX);
      print_counters();
    }
  }
  
  static void timer_start()
  {
    if (timer_running) {
      error_flag = true;
      Serial.println("Error: timer_start() already running");
      return;
    }
    
    timer_cyccnt64_start = cycles64();
    timer_cyccnt64_prev  = timer_cyccnt64_start;
    timer_cyccnt64_now   = timer_cyccnt64_start;
    
    if (pulse_armed  && frame_counts && timer_inner_counts && timer_outer_counts) {

      oops_flag = false;
      error_flag = false;
      
      timer_running = true;  // do it before the start, in case isr changes it
      
      frame_counter = 0;
      timer_inner_counter = 0;
      timer_outer_counter = 0;
      timer_first_time_flag = true;

      timerflexpwm->SM[TIMER_SUBMODULE].STS = 0xFF;     // clear all interrupt bits

      timerflexpwm->MCTRL |= TIMER_MASK;        // set load ok for timer submodule
      timerflexpwm->MCTRL |= (TIMER_MASK << 8); // set run for timer submodule
    }
    else {
      
      error_flag = true;
      
      Serial.print("Error: timer_start() - but not setup ");
      Serial.print(" pulse armed " ); Serial.print(pulse_armed);
      Serial.print(" frame_counts "); Serial.print(frame_counts);
      Serial.print(" timer_inner_counts "); Serial.print(timer_inner_counts);
      Serial.print(" timer_outer_counts "); Serial.println(timer_inner_counts);

      timer_stop();
    }
  }

  static void timer_stop()
  {
    timerflexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(TIMER_MASK);
    timerflexpwm->MCTRL = 0;      
    timer_running = false;
  }

  static void timer_stop_with_irq()
  {
    timerflexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(TIMER_MASK);
    timerflexpwm->MCTRL = 0;      
    timer_running = false;
    NVIC_DISABLE_IRQ(TIMER_IRQ);    
  }

  bool timer_wait(float timeout_=1., float timestep_=0.01, bool verbose=false)
  {
    uint32_t timeout = (uint) (timeout_ * 1000);
    uint32_t increment = (uint) (timestep_ * 1000);
    uint32_t elapsed = 0;

    if (verbose) {
      Serial.print("timer_wait "); Serial.print(timeout);
      Serial.print(" "); Serial.println(increment);
    }
    
    while(timer_running && !error_flag && elapsed < timeout) {
      delay(increment);
      elapsed += increment;
    }
    if (error_flag) {
      Serial.println("Error: timer_wait error_flag is set.");
      return false;
    }
    if (elapsed >= timeout) {
      Serial.print("Error: timer_wait timeout" );
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
      return false;
    }
    if (timer_running) {
      Serial.print("Error: timer_wait oops!  Return with timer still running");
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
      return false;
    }

    if (verbose) {
      Serial.print("Success: timer_wait");
      Serial.print(" running "); Serial.print(timer_running);
      Serial.print(" error "); Serial.print(error_flag);
      Serial.print(" elapsed "); Serial.print(elapsed);
      Serial.print(" timeout "); Serial.print(timeout);
      Serial.println(" millisecs");
    }
    
    return true;
  }
  
  bool setup_timer(float exposure_secs, float exposure_offset_secs, unsigned int ncounts=0)
  {
    unsigned int u32;
    uint16_t cmpf_mask;
    char cbuffer[128] = {0};
    
    // stop everything
    stop_runs_only();    // this clears run bits but leaves interrupts attached

    // we cannot touch the run mode, only clear the trigger mode
    trigger_mode = false;

    // clear error and oops flags
    clear_error_flags();
    
    // reset elapsed time counters
    timer_cyccnt64_start   = 0;
    timer_cyccnt64_prev    = 0;
    timer_cyccnt64_now     = 0;

    // Check, timer is for single pulse flexpwm only
    if ((mode!=PULSE) && (mode!=TIMER)) {
      Serial.println("Error: setup_timer, need to call setup_pulse() first");
      return false;
    }

    // Check, minimum exposure is the readout and transfer time
    if (exposure_secs < timer_interframe_min_secs) {
      sprintf(cbuffer,
              "Error: setup_timer exposure %.6gs is too short for this pulse configuration, need at least %.6g",
              exposure_secs,timer_interframe_min_secs);
      Serial.println(cbuffer);
      return false;
    }

    // Check, maximum offset is length of one exposure
    if (exposure_offset_secs > exposure_secs) {
      sprintf(cbuffer,
              "Error: setup_timer exposure offset %.6gs is too long, need offset less than exposure",
              exposure_offset_secs);
      Serial.println(cbuffer);
      return false;
    }

    /* ======================================================
       Setup timer callback, counts and period
    */
    timer_callback     = pulse_start;
    timer_outer_counts = ncounts ? ncounts+1 : frame_counts;
    frame_counts = timer_outer_counts;

    // Exposure within one iteration of the counter
    if (exposure_secs <= COUNTER_MAX_SECS) {
      timer_period_secs   = exposure_secs;
      timer_interval_secs = exposure_secs;
      timer_inner_counts  = 1;
    }

    // Exposure requires multuple iterations, set period greater than transfer and offset
    else {
      if (USBTRANSFERSECS > 0.024 || exposure_offset_secs > 0.025) {
        timer_period_secs = 0.050;
      }
      else if (USBTRANSFERSECS > 0.009 || exposure_offset_secs > 0.010) {
        timer_period_secs = 0.025;
      }
      else {
        timer_period_secs = 0.010;
      }      
      timer_inner_counts  = 0;
    }

    /* ======================================================
       From here we setup the control structure and hardware
    */    
    timer.prescale = 0;
    timer.divider = 1;
    u32 = (timer_period_secs*F_BUS_ACTUAL)/timer.divider;
    while (u32 > 65535 && timer.prescale < 7) {
      timer.prescale++;
      timer.divider = (1<<timer.prescale);
      u32 = (timer_period_secs*F_BUS_ACTUAL)/timer.divider;
    }

    if (u32 > 65535) {
      Serial.println("Error: unable to find prescale divider for exposure time");
      return false;
    }

    timer.period_counts = u32;

    timer_period_secs = (float) timer.period_counts * timer.divider / F_BUS_ACTUAL;

    // round to nearest multiple of timer_period_secs
    if (!timer_inner_counts) {
      timer_inner_counts = floor((exposure_secs+timer_period_secs/2)/timer_period_secs);
    }
    
    timer_interval_secs = timer_period_secs * timer_inner_counts;
    
    Serial.print("timer: timer inner period secs "); Serial.print(timer_period_secs,6);
    Serial.print(" counts ") ; Serial.print(timer_inner_counts);
    Serial.print(" actual interval ") ; Serial.println(timer_interval_secs,6);

    timer.onA_counts    = 0;
    timer.offA_counts   = 0;
    timer.invertA       = false;

    timer.onB_counts    = 0;
    timer.offB_counts   = ceil((exposure_offset_secs*F_BUS_ACTUAL)/timer.divider);
    cmpf_mask           = CMPF_MASKB_OFF;
    if (!timer.offB_counts) {
      timer.offB_counts = 1;
      cmpf_mask         = CMPF_MASKB_ON;
    }
    timer.invertB       = false;    

    timer.inten_mask    = 0;

    timer.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER;

    Serial.println("");
    if (!print_and_check_submodule(&timer)) {
      return false;
    }
    
    // ==============================================
    // timer mode is enabled
    mode = TIMER;  // we only have timer+pulse, we do not have timer+frameset

    // setup the hardware
    load_submodule(&timer);

    // and, attach to the timer irq
    attach_isr( &timer, cmpf_mask, timer_isr);

    return true;
  }

  /* =============================================================
     Setup triggers
  */
  //#define DEBUG_TRIGGERS

  inline static bool trigger_busy = false;
  
  static void trigger_isr()
  {
    uint64_t cyccnt_now = cycles64();

#ifdef DEBUG_TRIGGERS
    Serial.print("interrupt "); Serial.println(trigger_counter);
    Serial.print(" t="); Serial.println( (float)(cyccnt_now-interrup_cyccnt64_start)/F_CPU, 6);
#endif

    if (trigger_busy) {
      oops_flag = true;
      Serial.println("Oops! trigger interrupt while busy");
      return;
    }
    trigger_busy = true;
    
    // need to do this before sending data to the host
    trigger_cyccnt64_prev = trigger_cyccnt64_now;
    trigger_cyccnt64_now  = cyccnt_now;

    // the callback should have the right counter
    trigger_counter++;
    
    // this might result in a data send
    if (trigger_callback) {
      trigger_callback();
    }

    // are we stopping?
    if (trigger_counter >= trigger_counts) {
      detachInterrupt(digitalPinToInterrupt(trigger_pin));
      trigger_attached = false;

#ifdef DEBUG_TRIGGERS
      Serial.println("interrupt stop");
#endif
    }

    trigger_busy = false;
  }

  static void stop_triggers()
  {
    if (trigger_attached) {
      detachInterrupt(digitalPinToInterrupt(trigger_pin));
      trigger_attached = false;
    }
  }
  
  bool start_triggers( )
  {
    oops_flag = false;
    error_flag = false;

    if (!trigger_mode) {
      Serial.println("start triggers but not trigger mode");
      return false;
    }
    if (trigger_attached) {
      Serial.println("start triggers but trigger already attached");
      return false;
    }

    Serial.printf("start_triggers %d\n",trigger_pin);
    
    trigger_counter = 0;

    trigger_cyccnt64_start = cycles64();
    trigger_cyccnt64_now = trigger_cyccnt64_start;
    trigger_cyccnt64_prev = trigger_cyccnt64_start;

    // attach the trigger interrupt
    trigger_attached = true;
    attachInterrupt(digitalPinToInterrupt(trigger_pin), trigger_isr, trigger_edge_mode);  

    return true;
  }

  bool wait_triggers(float timeout_=1., float timestep_=0.01, bool verbose=false)
  {
    uint32_t timeout = (uint) (timeout_ * 1000);
    uint32_t increment = (uint) (timestep_ * 1000);
    uint32_t elapsed = 0;

    if (verbose) {
      Serial.printf("trigger_wait %d %d\n", timeout, increment);
    }
    
    while(trigger_attached && !error_flag && elapsed < timeout) {
      delay(increment);
      elapsed += increment;
    }
    if (error_flag) {
      Serial.println("Error: trigger_wait error_flag is set.");
      return false;
    }
    if (elapsed >= timeout) {
      Serial.printf("Error: wait_interrupt timedout %d / %d msecs\n",elapsed,timeout );
      return false;
    }
    if (timer_running) {
      Serial.printf("Error: timer_wait oops! still running %d / %d msecs\n",elapsed,timeout);
      return false;
    }

    if (verbose) {
      Serial.printf("Success: timer_wait running %x error %x %d/%d msecs\n",
                    trigger_attached, error_flag, elapsed, timeout);
    }
    
    return true;
  }

  
  bool setup_triggers(unsigned int ncounts=0)
  {
    stop_runs_only();    // clear the run bits

    // we cannot clear the operating mode, only the trigger mode.
    trigger_mode = false;
    error_flag = false;
    oops_flag = false;
    
    if (mode==NOTCONFIGURED) {
      Serial.println("Error: setup_interrupt, but flexpwm is not configured, call setup_flexpwm() first");
      error_flag = true;
      return false;
    }

    else if (mode==PULSE) {

      trigger_callback = pulse_start;
      trigger_counts   = ncounts ? ncounts : frame_counts;

      if(!trigger_counts) {
        Serial.println("Error: setup_interrupt pulse, but no frame_counts");
        error_flag = true;
        return false;
      }

      frame_counts       = trigger_counts;
      trigger_mode = true;
    }

    else if (mode==TIMER) {
      trigger_callback = timer_start;
      trigger_counts   = ncounts ? ncounts : 1;
      trigger_mode = true;
    }

    else if (mode==FRAMESET) {
      
      trigger_callback = frameset_start;
      trigger_counts   = ncounts ? ncounts : frameset_counts;

      if(!trigger_counts) {
        Serial.println("Error: setup_interrupt frameset, but no frameset_counts");
        error_flag = true;
        return false;
      }
      
      frameset_counts    = trigger_counts;
      trigger_mode = true;
    }
    
    else {
      Serial.println("Error: setup_interrupt, but mode not recognized");
      error_flag = true;
    }
    
    return trigger_mode;
  }
    
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Low level functions for command line register access.
  ////////////////////////////////////////////////////////////////////////////////////////////
    /*
    IMXRT_FLEXPWM_t *q = p->flexpwm;
    unsigned int submod = p->submod;
    uint16_t mask = p->mask;
    */
  
  void set_clock_master(uint8_t submod, IMXRT_FLEXPWM_t *q=flexpwm)
  {
    q->SM[submod].CTRL2 &= 0xFFF0;
    q->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_MASTER;
  }
  
  void set_clock_master(SubModule *p)
  {
    set_clock_master(p->submod, p->flexpwm);
  }

  // ------------------------------------------------------------
  void set_clock_slave(uint8_t submod, IMXRT_FLEXPWM_t *q=flexpwm)
  {
    q->SM[submod].CTRL2 &= 0xFFF0;
    q->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_SLAVE;
  }

  void set_clock_slave(SubModule *p)
  {
    set_clock_slave(p->submod, p->flexpwm);
  }
  
  // ------------------------------------------------------------
  void set_clock_sync(uint8_t submod, IMXRT_FLEXPWM_t *q=flexpwm){
    q->SM[submod].CTRL2 &= 0xFFF0;
    q->SM[submod].CTRL2 |= PWM_CTRL2_CLOCK_SYNC;
  }

  void set_clock_sync(SubModule *p)
  {
    set_clock_sync(p->submod, p->flexpwm);
  }
  
  // ------------------------------------------------------------
  void clear_ldok(uint8_t mask, IMXRT_FLEXPWM_t *q=flexpwm) {
    q->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
  }
  
  void set_ldok(uint8_t mask, IMXRT_FLEXPWM_t *q=flexpwm) {
    q->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
  }
  
  // ------------------------------------------------------------
  void clear_run(uint8_t mask, IMXRT_FLEXPWM_t *q=flexpwm) {
    q->MCTRL &= FLEXPWM_MCTRL_RUN(~mask);
  }
  void set_run(uint8_t mask, IMXRT_FLEXPWM_t *q=flexpwm) {
    q->MCTRL |= FLEXPWM_MCTRL_RUN(mask);
  }

  // ------------------------------------------------------------
  void force(uint8_t submod, IMXRT_FLEXPWM_t *q=flexpwm) {
    if(submod<4) {
      q->SM[submod].CTRL2 = FLEXPWM_SMCTRL2_FORCE;
    }
  }

  // ------------------------------------------------------------
  void set_prescale(uint8_t submod, uint8_t prescale=0, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);
    }
  }

  void set_init(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].INIT = value;
    }
  }

  void set_val0(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL0 = value;
    }
  }

  void set_val1(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL1 = value;
    }
  }

  void set_val2(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL2 = value;
    }
  }

  void set_val3(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL3 = value;
    }
  }

  void set_val4(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL4 = value;
    }
  }

  void set_val5(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].VAL5 = value;
    }
  }
  // -----------------------------------------------------------
  uint16_t set_outen( uint16_t mask16, IMXRT_FLEXPWM_t *q=flexpwm) {    
    q->OUTEN = mask16;
    return q->OUTEN;
  }
  
  uint16_t set_outen_on( uint16_t mask16, IMXRT_FLEXPWM_t *q=flexpwm) {    
    q->OUTEN |= mask16;
    return q->OUTEN;
  }

  uint16_t set_outen_off( uint16_t mask16, IMXRT_FLEXPWM_t *q=flexpwm) {    
    q->OUTEN &= ~mask16;
    return q->OUTEN;
  }
  
  uint16_t set_outenA_on( uint8_t mask8, IMXRT_FLEXPWM_t *q=flexpwm) {    
    q->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask8);
    return q->OUTEN;
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
        Serial.print(" CMPF "); printbits16_(u16,6); // Serial.print(u16&0x3F,BIN);
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
    pinMode(TRIGGER_PIN, INPUT);

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

    // clear the mode
    clear_mode();
  }
  
  static void close()
  {
    // stop both flexpwms, disable interrrupts and reset clear and busy pins
    stop_with_irqs();  

    // Reactivation as digital i/o pins
    setup_digital_pins();
    
  }
  


};

#endif
