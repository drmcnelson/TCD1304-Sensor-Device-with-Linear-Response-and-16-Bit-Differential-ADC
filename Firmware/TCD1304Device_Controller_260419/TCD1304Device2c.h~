/* tcd1304 flexpwm library

   Author Mitchell C Nelson, PhD
   Copyright 2025

   Free for non-commercial use.

   No warranty and no representation of suitability for any purpose whatsoever

*/

#ifndef TCD1304DEVICE_H
#define TCD1304DEVICE_H

#define TCD1304DEVICE_LIBARY_VERSION 0X002

#include "Arduino.h"
#include <SPI.h>

#include "imxrt.h"

#include <limits.h>

// =========================================
// All-In-One Board or other that uses analog input
// #define ALLINONEBOARD

// CONFIGURE PULLUPS/PULLDOWNS
#define PINPULLS

// For PIT mode, the CNVST idle is our virtual buffer for vref - leave it on!
// #define CNVST_IDLE_DISABLE

// For PIT mode, clearing pulses default
#define SH_CLEARING_DEFAULT 0

// Do voltage offset calculation
//#define TCD13404_OFFSET_ENABLED

// debug statements in setup
//#define DEBUG

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
// used in seconds to/from counter value
#define COUNTSCEIL(a) (a<1.?(int)ceil((a)*F_BUS_ACTUAL):(int)a)
#define COUNTSFLOOR(a) (a<1.?(int)floor((a)*F_BUS_ACTUAL):(int)a)
#define SECONDS(a) ((float)(a)/F_BUS_ACTUAL)

#define ROUNDUP(a,b) (ceil((float)a/b)*b)
#define ROUNDTO(a,b) ((a/b)*b)
#define ROUNDTOMOD(a,b,c) (((a/b)*b)%c)

// -------------------------------------------
// cpu cycle counter values to seconds
// usage
#ifndef CYCCNT2SECS
#define CYCCNT2SECS(a) (((double)(a))/F_CPU)
#endif

#ifndef SECS2CYCCNT
#define SECS2CYCCNT(a) ((uint64_t)(((double)(a))*F_CPU))
#endif

#define TDIFFSECS(a,b) (((double)(a-b))/F_CPU)

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
#define CLK_IRQ IRQ_FLEXPWM2_0

#define ICG_SUBMODULE 1
#define ICG_MASK 0x2
#define ICG_CHANNEL 1
#define ICG_PIN 5
#define ICG_MUXVAL 1
#define ICG_IRQ IRQ_FLEXPWM2_1

#define SH_SUBMODULE 2
#define SH_MASK 0x4
#define SH_CHANNEL 1
#define SH_PIN 6
#define SH_MUXVAL 2
#define SH_IRQ IRQ_FLEXPWM2_2

#define CNVST_SUBMODULE 3
#define CNVST_MASK 0x8
#define CNVST_CHANNEL 1
#define CNVST_IRQ IRQ_FLEXPWM2_3

/* Note that we do not configure the flexPWM to use this pin (CNVST_PIN),
   It is not on the mux list for this module,
   We operate this pin using digitalwrite, from the isr.
*/
#define CNVST_PIN 10
#define SETCNVST (CORE_PIN10_PORTSET = CORE_PIN10_BITMASK)

#define CLEARCNVST (CORE_PIN10_PORTCLEAR = CORE_PIN10_BITMASK)

#define CLK_MONITOR_PIN 3

#define SYNC_PIN 0
#define BUSY_PIN 1
#define TRIGGER_PIN 2
//#define SPARE_PIN 3

#define SYNC_PIN_DEFAULT LOW
#define BUSY_PIN_DEFAULT HIGH

// Fast nofrills BUSY pin set, clear, flip state
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

// Perhaps redundant but more mnemonic
#define NPIXELS NREADOUT

// Size for sending the data
#define SENDSTART 0
#define SENDLENGTHWORDS NREADOUT
#define SENDLENGTHBYTES (2*NREADOUT)
#define SENDLENGTHBYTES32 (4*NREADOUT)

// And first part of that is dark
#define DARKSTART (DATASTART-SENDSTART)
#define NDARK 13

#ifdef  ALLINONEBOARD
#define NBITS 12
#define VFS (3.3/5)
#else
#define NBITS 16
#define VFS (4.096/5)
#endif

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
  PULSE_LOOP,
  PIT,
  FLEXPWM
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
    //uint16_t period_counts = 0;
    uint32_t period_counts = 0;
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
    float offset = 0; // offset in nanoseconds for start from time = 0, for cnvst

    bool invertA = false;
    bool invertB = false;

    bool newvals = false;

    void (*isr)() = nullptr;

    uint8_t interrupt_priority = 128;
    
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
  
  inline static uint64_t pit_cyccnt64_start = 0;
  inline static uint64_t pit_cyccnt64_now = 0;
  inline static uint64_t pit_cyccnt64_prev = 0;

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

  // framesets counters and framesets completed callback
  inline static unsigned int frameset_counter = 0;
  inline static unsigned int frameset_counts = 0;
  inline static void (*framesets_completed_callback)() = 0;

  // old code?
  inline static float pulse_interframe_min_secs = 0.;
  
  // interrupt (trigger) support
  inline static unsigned int trigger_counter = 0;
  inline static unsigned int trigger_counts  = 1;
  inline static void (*trigger_callback)() = 0;
  
  inline static uint8_t trigger_pin = TRIGGER_PIN;
  inline static uint8_t trigger_edge_mode = RISING;
  inline static uint8_t trigger_pin_mode = INPUT;
   
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

  // error bookkeeping
  inline static bool error_flag = false;
  inline static bool oops_flag = false;

  // completion flags
  inline static bool frames_complete = false;
  inline static bool framesets_complete = false;
  
  char oopsbuffer[128] = { 0 };

  //---------------------------------------------
  typedef struct Frame_Header_struct {
    uint16_t *buffer;
    unsigned int nbuffer;

    // from the dummy outputs, first 16 elements
    unsigned int rawoffset;
    float offset;
    bool offsetdone;
    
    float frame_elapsed_secs;
    float frame_exposure_secs;

    float pit_elapsed_secs;
    float pit_difference_secs;

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
    stop_all();

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

    set_pulse_parameters(0.4E-6);
  }

  static void stop_all()
  {
    flexpwm->MCTRL = 0;
    flexpwm_running = false;

    stop_pit();

    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(0xF);

    disable_irqs();
    
    clear_sync_busy_pins();
 
    clear_mode();  // if the irqs are detached there is not a valid mode anymore

  }
  
  static void disable_irqs()
  {
    // Disable all of the flexpwm interrupts
    NVIC_DISABLE_IRQ(CLK_IRQ);
    NVIC_DISABLE_IRQ(ICG_IRQ);
    NVIC_DISABLE_IRQ(SH_IRQ);
    NVIC_DISABLE_IRQ(CNVST_IRQ);
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
    read_counter  = read_counts;  // will be set to 0 in the icg isr
  }

  static void fill_frame_header(Frame_Header *p)
  {
#ifdef TCD13404_OFFSET_ENABLED
    unsigned int utmp = 0;
    int n;
#endif
    
    p->buffer = read_buffer;
    p->nbuffer = read_counts;

    p->rawoffset = adc_raw_offset;
    p->offset = adc_offset;
    p->offsetdone = adc_offset_done;
    
    p->frame_counter = frame_counter;
    p->frameset_counter = frameset_counter;

    p->trigger_counter = trigger_counter;

    p->frames_completed = frames_complete;
    
    p->framesets_completed = framesets_complete;
    
    p->frame_elapsed_secs = sh_elapsed_secs();
    p->frame_exposure_secs = sh_exposure_secs();

    p->pit_elapsed_secs = pit_elapsed_secs();
    p->pit_difference_secs = pit_difference_secs();

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

  // Pulse duration
#define CNVST_SECS 630.E-9

  // Overhead to start the pulse (isr + digitalwrite)
  #define CNVST_START_OVERHEAD 300.0E-9
  //#define CNVST_START_OVERHEAD 950.0E-9

  // Number of clock cycles to place the CNVST
  //#define CNVST_CLKS_FROM_ICG 3.75
#define CNVST_CLKS_FROM_ICG 3.75
  
  // CNVST offset before the next rising edge clock pulse (see above)
  //#define SAR_OFFSET 740E-9 // finish the sar just before the pixel transition
#define SAR_OFFSET 100.E-9 // CNVST at last feasible moment
  
  inline static float pulse_clk_secs = 0.;

  inline static float pulse_sh_offset_secs = 0.;
  inline static float pulse_sh_secs = 0.;

  inline static float pulse_icg_offset_secs = 0.;
  inline static float pulse_icg_secs = 0.;

  inline static float pulse_cnvst_offset_secs = 0.;
  inline static float pulse_cnvst_secs = 0.;

  inline static float pulse_loop_period_secs = 0.; // sh and icg shared period

  // helper functions
  double get_next_phase_time(double t, double p, double phase_fraction) {
    double target_offset = phase_fraction * p;
    double current_pos = fmod(t, p);
        
    if (current_pos < 0) current_pos += p;
        
    double wait_time = fmod(target_offset - current_pos, p);
    if (wait_time < 0) wait_time += p;

    return t + wait_time;
  }

  uint16_t get_next_phase_tick(uint16_t t, uint32_t p, double phase_fraction) {
    uint32_t target_offset = (uint32_t)round(phase_fraction * (double)p);
    uint32_t current_pos = t % p;
    
    int32_t wait_ticks = (int32_t)target_offset - (int32_t)current_pos;
    
    if (wait_ticks < 0) {
        wait_ticks += p;
    }
    
    return (uint16_t)((t + wait_ticks) % p);
  }

  // get next multiple of a greater than b
  uint32_t get_next_multiple(uint32_t a, uint32_t b) {
    uint32_t remainder = b % a;
    return b + (remainder ? a - remainder:0);
  }

  void print_pulse_parameters()
  {
    Serial.printf("timings clk %.4g sh duration %.4g offset %.4g icg duration %.4g offset %.4g cnvst duration %.4g offset %.4g period %4g\n",
                  pulse_clk_secs,
                  pulse_sh_secs,
                  pulse_sh_offset_secs,
                  pulse_icg_secs,
                  pulse_icg_offset_secs,
                  pulse_cnvst_secs,
                  pulse_cnvst_offset_secs,
                  pulse_loop_period_secs);
  }
    
  bool set_pulse_parameters(float clock_period_secs=0.4E-6, float period_secs=10.E-6)
  {
    if (clock_period_secs>100.E3) {
      // interpret as frequency
      clock_period_secs = 1./clock_period_secs;
    }
    else if (clock_period_secs>1.) {
      // interpret as counts
      clock_period_secs = clock_period_secs/F_BUS_ACTUAL;
    }

    if (clock_period_secs<0.4E-6) {
      Serial.println("Error: clock speeds is too fast, limit 2.5MHz or 0.4E-6secs");
      return false;
    }
    if (clock_period_secs>1.25E-6) {
      Serial.println("Error: clock speeds is too slow, limit 800KHz or 1.25E-6secs");
      return false;
    }
    
    pulse_clk_secs          = clock_period_secs;

    pulse_sh_offset_secs    = pulse_clk_secs;
    pulse_sh_secs           = 1.E-6;

    pulse_icg_offset_secs   = pulse_clk_secs/2;

    pulse_icg_secs          = pulse_sh_offset_secs + pulse_sh_secs + 1.E-6;
    pulse_icg_secs          = get_next_phase_time(pulse_icg_secs, clock_period_secs, 0.75);
    pulse_icg_secs          -= pulse_icg_offset_secs;
    
    pulse_cnvst_offset_secs = (CNVST_CLKS_FROM_ICG * pulse_clk_secs) - SAR_OFFSET - CNVST_START_OVERHEAD;

    pulse_loop_period_secs  = period_secs;

    return true;
  }

  // ============================================================
  
  inline static float read_expected_time = 0.;

  static void start_read()
  {
    switch(mode) {
    case PIT:
      pit_start();
      break;
    case PULSE:
      pulse_start();
      break;
    case PULSE_LOOP:
      pulse_loop_start();
      break;
    default:
      Serial.println("Error: start_read mode not configured.");
      error_flag = true;
    }
  }
  
  bool wait_read(float timeout=0., float timestep=0.01, bool interruptible=false, bool verbose=false)
  {
    bool retv = false;
    
    if (!timeout) timeout = read_expected_time - pit_elapsed_secs() + 0.010;
    if (timeout < 0.1) timeout = 0.1;

    switch(mode) {
    case PIT:
      retv = pit_wait(timeout,timestep, interruptible, verbose);
      break;
    case PULSE_LOOP:
      retv = pulse_loop_wait(timeout,timestep, interruptible, verbose);
      break;
    case PULSE:
      retv = flexpwm_wait(timeout,timestep, interruptible, verbose);
      break;
    default:
      Serial.println("Error: wait_read not configured");
      retv = false;
      break;
    }

    return retv;
  }
   
  // -------------------------------------------------------------
  
  bool read(uint nframes, float exposure, uint16_t *bufferp,
            void (*frame_callbackf)(),
            void (*frameset_callbackf)(),
            void (*completion_callbackf)(),
            void (*setup_callbackf)(),
            bool start=true)
  {

    if (!setup_pulse(pulse_clk_secs,
                     pulse_sh_secs, pulse_sh_offset_secs,
                     pulse_icg_secs, pulse_icg_offset_secs,
                     bufferp, NREADOUT,
                     frame_callbackf)) {
      Serial.println("Error: failed to setup pulse");
      return false;
    }

    frames_complete = false;
    framesets_complete = false;

    frame_counts = nframes+1;
    frames_completed_callback = frameset_callbackf;
    framesets_completed_callback = completion_callbackf;

    read_expected_time = exposure * nframes;

    if (exposure > 0.) {
      if (!setup_pit(exposure,pit_delay_secs,0)) {
        return false;
      }
    }

    if (setup_callbackf) {
      setup_callbackf();
    }
    
    if (start) {
      start_read();
    }

    return !error_flag;
  }
  
  // -------------------------------------------------------------
  
  bool read(uint nframes, float exposure, float frame_interval, uint16_t *bufferp,
            void (*frame_callbackf)(),
            void (*frameset_callbackf)(),
            void (*completion_callbackf)(),
            void (*setup_callbackf)(),
            bool start=true)
  {

    if (!setup_pulse_loop(pulse_clk_secs, pulse_sh_secs, pulse_sh_offset_secs,
                          pulse_icg_secs, pulse_icg_offset_secs, pulse_loop_period_secs,
                          exposure, frame_interval, nframes, 
                          bufferp, NREADOUT, frame_callbackf)) {
      Serial.println("Error: failed to setup pulse loop");
      return false;
    }
      
    read_expected_time = frame_interval * nframes;
    frames_completed_callback = frameset_callbackf;
    framesets_completed_callback = completion_callbackf;

    if (setup_callbackf) {
      setup_callbackf();
    }
   
    if (start) {
      pulse_loop_start();
    }
    
    return !error_flag;
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

  bool wait_triggered_read(float timeout=1., float timestep=0.01, bool interruptible=true, bool verbose=false)
  {
    if (wait_trigger(timeout,timestep,interruptible,verbose)) {
      return wait_read(timeout,timestep,interruptible,verbose);
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
  static double pit_elapsed_secs()
  {
    return (double)(pit_cyccnt64_now - pit_cyccnt64_start)/F_CPU;
  }

  static double pit_difference_secs()
  {
    return (double)(pit_cyccnt64_now - pit_cyccnt64_prev)/F_CPU;
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
    if (mode==PIT) {
      Serial.print(" pit "); Serial.print(pit_counter);
      Serial.print(" "); Serial.print(pit_counts);
    }
    if (trigger_mode) {
      Serial.print(" triggers "); Serial.print(trigger_counter);
      Serial.print(" "); Serial.print(trigger_counts);
    }
    Serial.println("");
  }

  void print_setup_diagnostic(SubModule *p)
  {
    Serial.printf("setup %s prescale %d, divider %d (%.6gs) period %lu (%.6gs) onA %u (%.6gs) offA %u (%.6gs)\n",
                  p->name,
                  p->prescale,p->divider,SECONDS(p->divider),
                  p->period_counts, SECONDS(p->period_counts*p->divider),
                  p->onA_counts, SECONDS(p->onA_counts*p->divider),
                  p->offA_counts, SECONDS(p->offA_counts*p->divider));
  }
    
  /* ======================================================================================
     Print, check, setup submodule configuration, load into hardware
  */
  void print_setup()
  {
    print_submodule(&clk);
    print_submodule(&icg);
    print_submodule(&sh);
    print_submodule(&cnvst);

    Serial.print("configured mode:");
    if (trigger_mode) {
      Serial.print(" triggered");
    }
    
    switch(mode) {
    case PULSE:
      Serial.println(" PULSE");
      break;
    case PULSE_LOOP:
      Serial.println(" PULSE_LOOP");
      break;
    case PIT:
      Serial.printf(" PIT %f %f\n", pit_delay_secs, pit_interval_secs);
      break;
    default:
      Serial.println(" NONE");
    }
  }
  
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
	      "flexpwm: %s period %lu presc %d div %d offset %.1f => %.6g secs %.6g Hz ctrl2_mask x%02x",
	      p->name, p->period_counts, p->prescale, p->divider, p->offset, period_secs, frequency, p->ctrl2_mask);
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
    if (p->period_counts > 65536) {
      print_errormsg(p->name, "period counts > 2^16");
    }
    if (p->divider<1) {
      print_errormsg(p->name, "divider < 1");
      retv = false;
    }
    if (p->divider>128) {
      print_errormsg(p->name, "divider > 128");
      retv = false;
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
    q->SM[submod].CNT  = 0;
    q->SM[submod].VAL0 = 0;
    q->SM[submod].VAL1 = (uint16_t) (p->period_counts - 1);
    q->SM[submod].VAL2 = p->onA_counts;
    q->SM[submod].VAL3 = (p->offA_counts > 0) ? p->offA_counts: 0;
    q->SM[submod].VAL4 = p->onB_counts;
    q->SM[submod].VAL5 = (p->offB_counts > 0) ? p->offB_counts : 0;

    // Convenience, save the clock period in seconds
    p->period_secs = (float) p->period_counts * ((float)(1<<p->prescale)/ F_BUS_ACTUAL);
    
    // Do we have a Pin for the A channel?
    if (p->pinA != 0xFF && (p->offA_counts|p->onA_counts)) {

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
    if (p->pinB != 0xFF && (p->offB_counts|p->onB_counts)) {
      
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
  bool setup_submodule( SubModule *p, uint8_t prescale, uint32_t period_counts,
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
        // Here we actually load it into the registers
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
  void attach_isr( SubModule *p, uint16_t cmpf_mask, void (*isrf)(), uint8_t priority=0, bool enable=true)
  {
    IMXRT_FLEXPWM_t *q = p->flexpwm;
    unsigned int submod = p->submod;
    uint16_t status;

    // disable the irq for this submodule
    NVIC_DISABLE_IRQ(p->irq);
    
    // clear all of this module's interrupt status bits
    status = q->SM[submod].STS;
    q->SM[submod].STS = status;

    // enable the specified bits only
    q->SM[submod].INTEN = enable ? cmpf_mask : 0;
    p->intena_mask = cmpf_mask;

    // bookkeeping
    p->isr = isrf;

    // override the configured priority
    if (priority > 0) {
      p->interrupt_priority = priority;
    }

    // register the isr to this irq
    attachInterruptVector(p->irq, isrf);

    // set priority
    if (p->interrupt_priority > 0) {
      NVIC_SET_PRIORITY(p->irq, p->interrupt_priority);
    }
    
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

  /* ========================================================================================
     Generic wait routine
  */
  bool generic_wait(bool *flag, float timeout_=1., float timestep_=0.01,
                    bool interruptible=false, bool verbose=false)
  {
    uint32_t timeout = (uint) (timeout_ * 1000);
    uint32_t increment = (uint) (timestep_ * 1000);
    uint32_t elapsed = 0;

    if (verbose) {
      Serial.printf("wait %d msec %d msec\n", timeout, increment);
    }
    
    while((*flag) && !error_flag && elapsed < timeout) {
      delay(increment);
      elapsed += increment;   
      if (interruptible && Serial.available( )) {
        Serial.println("Abort: Wait interrupted by serial input available");
        return false;
      }
    }

    if (error_flag) {
      Serial.println("Error: wait - error_flag is set.");
      return false;
    }

    if (elapsed >= timeout) {
      Serial.printf("Error: wait timeout elapsed %d timeout %d msecs\n", elapsed, timeout);
      return false;
    }

    if ((*flag)) {
      Serial.printf("Error: wait oops!  Return with timer still running, elapsed %d timeout %d msecs\n", elapsed, timeout);
      return false;
    }

    if (verbose) {
      Serial.printf("Success: wait running %d err0r %d elapsed %d timeout %d msecs\n", *flag, error_flag, elapsed, timeout);
    }
    
    return true;
  }

  /* =======================================================================
     CNVST idle start
  */
  static void cnvst_start()
  {
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK); // set run clk, sh, icg
  }
  
  /* =======================================================================
     FlexPWM stop and wait
  */

  inline static bool flexpwm_running = false;
  
  static void flexpwm_stop(uint8_t mask=0xF)
  {
    flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(mask);
    flexpwm_running = false;
  }

  static void flexpwm_start(uint8_t mask=0xF)
  {
    oops_flag = false;
    error_flag = false;
    flexpwm_running = true;

    // Set load ok bits - clk, sh, icg
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);

    // set run bits - clk, sh, icg
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(mask);
  }
  
  static void flexpwm_resetcounters(uint8_t mask=0xFF)
  {
    flexpwm_running = false;
    
    // stop
    flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(mask); // clear run clk, sh, icg

    // clear the load bits
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);

    // clear the counters  - clk, sh, icg
    if (mask&0x1) flexpwm->SM[0].CNT = 0;
    if (mask&0x2) flexpwm->SM[1].CNT = 0;
    if (mask&0x4) flexpwm->SM[2].CNT = 0;
    if (mask&0x8) flexpwm->SM[3].CNT = 0;

    // now we load and init  - clk, sh, icg 
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);

    // and force initialization - through clk - sh and icg are slaved
    //flexpwm->SM[CLK_SUBMODULE].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;  // we probably dont need this
  }

  bool flexpwm_wait(float timeout_=1., float timestep_=0.01, bool interruptible=false, bool verbose=false)
  {
    if (verbose) {
      Serial.print("waiting for flexpwm_running");
    }
    return generic_wait(&flexpwm_running,timeout_,timestep_,interruptible,verbose);
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// Configure the flexpwm for single pulse, use this with a timer or interrupt
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
    // bookkeeping
    sh_counter++;
    
    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.printf("OOPS! pulse sh without off bit set 0x%04x", status);
    }
  }

  static void pulse_icg_isr()
  {
    uint16_t status;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    //Serial.print("icg_isr "); print_counters();
    if (status & CMPF_MASKA_ON) {

      //Serial.println("pulse icg maska_on");

      // prepare to restart the CNVST clock
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);   // clear ldok
      flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(CNVST_MASK);    // stop
      
      flexpwm->SM[CNVST_SUBMODULE].STS = flexpwm->SM[CNVST_SUBMODULE].STS; 
      NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + CNVST_SUBMODULE);

      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);    // set ldok
      flexpwm->SM[CNVST_SUBMODULE].CNT = 0;                // zero the submodule counter
      flexpwm->SM[CNVST_SUBMODULE].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;  // force init
      read_counter = 0;                                    // zero the readout counter
    }
    
    else if (status & CMPF_MASKA_OFF) {

      //Serial.println("pulse icg maska_off");
      
      // Start the cnvst clock
      flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);
    
      // no more icg interrupts
      flexpwm->SM[ICG_SUBMODULE].INTEN = 0;
    
      // never on again - slaved, run bit is not effective
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(ICG_MASK);
      flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(ICG_MASK);

      // here is our exposure, diff to most recent previous sh gate    
      sh_cyccnt64_exposure = sh_cyccnt64_now - sh_cyccnt64_prev;
    
      // bookkeeping
      icg_counter++;
    }

    // diagnostics
    else {
      oops_flag = true;
      Serial.println("OOPS! pulse icg without on or off bit set");
    }
  }
  
  static void pulse_cnvst_isr()
  {
    uint16_t status;

    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;

    if ((status&CMPF_MASKA_OFF)) {

#ifndef ALLINONEBOARD
        // Assert the convert pin
        SETCNVST;
        delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
        CLEARCNVST;               // need 30 nanoseconds after this
#endif
      
      if (read_counter < read_counts) {

#ifdef ALLINONEBOARD
        adc->adc0->startReadFast(ANALOGPIN);
        while ( adc->adc0->isConverting() );
        *read_pointer = adc->adc0->readSingle();
#else      
        *read_pointer = SPI.transfer16(0xFFFF);
        *read_pointer ^= (0x1<<15);
#endif

        // bookkeeping for the read
        read_pointer++;
        read_counter++;

        // The read is done
        if (read_counter == read_counts) {

          // wait for start of next clk period
          if (flexpwm->MCTRL & FLEXPWM_MCTRL_RUN(CLK_MASK)) {
            while (!(flexpwm->SM[CLK_SUBMODULE].STS & FLEXPWM_SMSTS_RF));
            flexpwm->SM[CLK_SUBMODULE].STS = FLEXPWM_SMSTS_RF;
          }
          
          // Stop clk, sh, icg 
          // ** Indicate stop only after callback **
          flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((CLK_MASK|SH_MASK|ICG_MASK));

          // now stop everything
          flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN((CLK_MASK|SH_MASK|ICG_MASK));

          flexpwm_running = false;

#ifdef  CNVST_IDLE_DISABLE
          flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(CNVST_MASK);
#endif
          
          // ----------------------------------------
          // User supplied function, per read complete
          // First exposure is frame 1, frame 0 is the pre-exposure
          
          frames_complete = (frame_counter+1 >= frame_counts);
          framesets_complete = frames_complete && ((frameset_counter+1) >= frameset_counts); 
          
          if (read_callback) {
            read_callback();
          }

          // =============================================
          // Frame bookkeeping, re-arm etc
          frame_counter++;

          // More frames?
          if (frame_counter < frame_counts) {
            pulse_reset_and_arm();
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

              sh_cyccnt64_start   = 0;
              sh_cyccnt64_prev    = 0;
              sh_cyccnt64_now     = 0;
    
              sh_counter    = 0;
              icg_counter   = 0;
              cnvst_counter = 0;

              frame_counter = 0;
    
              pulse_reset_and_arm();
            }

            // Nope, we're done with all of the framesets
            else if (frameset_counter == frameset_counts) {
              
              if (framesets_completed_callback) {
                framesets_completed_callback();
              }

              // and get ready for the next frame set
              sh_cyccnt64_start   = 0;
              sh_cyccnt64_prev    = 0;
              sh_cyccnt64_now     = 0;
    
              sh_counter    = 0;
              icg_counter   = 0;
              cnvst_counter = 0;

              frame_counter = 0;
              frameset_counter = 0;
    
              pulse_reset_and_arm();
            }
            
            // oops, shouldn't get here
            else {
              oops_flag = true;
              Serial.printf("line %d, read_counter %d/%d frameset %d/%d\n", __LINE__, read_counter, read_counts, frameset_counter, frameset_counts);
              Serial.println("Oops!  cnvst interrupt after frameset_counts complete");
              stop_all();
            }
                  
          }

          // oops, shouldn't get here
          else {
            oops_flag = true;
            Serial.printf("line %d, read_counter %d/%d frameset %d/%d\n", __LINE__, read_counter, read_counts, frameset_counter, frameset_counts);
            Serial.println("Oops!  cnvst interrupt after frame_counts complete");
            stop_all();
          }
          
        }

      }
      
    }

    // oops, shouldn't get here
    else {
      oops_flag = true;
      //Serial.printf("OOPS! pulse cnvst without off bit set 0x%x\n", status);
    }
    
    // diagnostics
    cnvst_counter++;
    
  }

  static void pulse_start()
  {
    if (!pulse_armed) {
      error_flag = true;
      Serial.println("\nError: pulse not armed");
      return;
    }
    pulse_armed = false;

    flexpwm_running = true;
    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN((CLK_MASK|SH_MASK|ICG_MASK)); // set run clk, sh, icg
  }

  static void pulse_reset_and_arm()
  {
    uint16_t status;

    // Reset the clk
    flexpwm->SM[CLK_SUBMODULE].CNT = 0;
    
    // Restore and reset the SH submodule
    status = flexpwm->SM[SH_SUBMODULE].STS;
    flexpwm->SM[SH_SUBMODULE].STS = status;

    flexpwm->SM[SH_SUBMODULE].INTEN = CMPF_MASKA_OFF;
    flexpwm->SM[SH_SUBMODULE].VAL1 = (uint16_t) (sh.period_counts - 1);
    flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;
    flexpwm->SM[SH_SUBMODULE].VAL3 = sh.offA_counts;
    flexpwm->SM[SH_SUBMODULE].CNT = 0;

    sh_clearing_counter = 0;

    // Restore and reset the ICG submodule
    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    flexpwm->SM[ICG_SUBMODULE].INTEN = CMPF_MASKA_ON_OFF;

    flexpwm->SM[ICG_SUBMODULE].VAL2 = icg.onA_counts;
    flexpwm->SM[ICG_SUBMODULE].VAL3 = icg.offA_counts;
    flexpwm->SM[ICG_SUBMODULE].CNT = 0;

    // CNVST is never disabled, but do Reset the buffer pointer
    read_pointer = read_buffer;
    read_counter = read_counts; // will be set to 0 in the icg isr

    // and renable
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((CLK_MASK|SH_MASK|ICG_MASK));
    flexpwm->SM[CLK_SUBMODULE].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;
    
    pulse_armed = true;
  }

  bool setup_pulse(float clk_secs, float sh_secs, float sh_offset_secs, float icg_secs, float icg_offset_secs,
                   uint16_t *buffer, unsigned int nbuffer, void (*callbackf)())
  {
    char cbuffer[128] = {0};

    // stops all, clears ldok, disables irqs and clears mode
    stop_all();

    // clear the error flags
    clear_error_flags();

    // and we show the settings
    sprintf(cbuffer,
            "#tcd1304 setup pulse, clk %.5gs sh %.5gs offset %.5gs icg offset %.5gs",
	    clk_secs, sh_secs, sh_offset_secs, icg_offset_secs);
    Serial.println(cbuffer);
    
    // clear elapsed times
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;

    pit_cyccnt64_start   = 0;
    pit_cyccnt64_prev    = 0;
    pit_cyccnt64_now     = 0;
    
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
    
    clk.prescale = 0;
    clk.divider  = 1;

    clk.period_counts = ceil(clk_secs*F_BUS_ACTUAL);    // rouund up

    clk.offset        = 0.;
    
    clk.offA_counts   = 0;
    clk.onA_counts    = (clk.period_counts/2);
    clk.invertA       = false;

    clk.onB_counts    = 0;
    clk.offB_counts   = 0;
    clk.invertB       = false;

    clk.intena_mask    = 0;

    clk.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER;
    
    // ------------------------------------------------------------------
    // Setup the CNVST clock, 4 times the clock period,
    cnvst.prescale = clk.prescale;
    cnvst.divider = (1<<cnvst.prescale);
    
    cnvst.period_counts = 4*clk.period_counts;

    cnvst.offset        = 0;

    cnvst.onA_counts    = ceil(pulse_cnvst_offset_secs*F_BUS_ACTUAL/cnvst.divider);    // delay to approx the fourth clock
    cnvst.offA_counts   = cnvst.onA_counts + ceil(pulse_cnvst_secs*F_BUS_ACTUAL/cnvst.divider);
    cnvst.invertA       = false;

    cnvst.onB_counts    = 0;
    cnvst.offB_counts   = 0;
    cnvst.invertB       = false;
    
    //cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_SYNC;
    cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER; // needs to be master, because run bit is disabled when running from clk's clock.
    
    // ------------------------------------------------------------------
    // setup the sh, sh_offset-icg_offset is the icg-sh timing, runs once, isr has to set on,off to 0xffff
    sh.prescale = clk.prescale;
    sh.divider  = (1<<sh.prescale);

    sh.period_counts  = 0x8000;   // long period, we are going to pulse once and stop

    sh.offset         = 0.;
    
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
    icg.divider  = (1<<icg.prescale);

    icg.period_counts = 0x8000;   // long period, we are going to pulse once and stop

    icg.offset        = 0.;
    
    icg.onA_counts    = ceil(icg_offset_secs*F_BUS_ACTUAL/icg.divider);
    icg.offA_counts   = icg.onA_counts + ceil(icg_secs*F_BUS_ACTUAL/icg.divider);
    icg.invertA       = true;

    icg.onB_counts    = 0;
    icg.offB_counts   = 0;
    icg.invertB       = false;

    icg.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //icg.ctrl2_mask    = FLEXPWM_SMCTRL2_CLK_SEL(0x2);

    // this is used by the timing graphics software
    cnvst.offset = (((float)icg.offA_counts / F_BUS_ACTUAL) + CNVST_START_OVERHEAD)*1.E9;

    // ------------------------------------------------------------------
    // optional: sh gate clearing cycles
    if (sh_clearing_counts) {
      // set sh period to after icg but before the first cnvst isr
      sh.period_counts = icg.offA_counts + clk.period_counts;   //
    }

    // for short clearing cycles, the period is twice the sh pulse width
    sh_short_period_counts = 2*(sh.offA_counts - sh.onA_counts) + 1;

    // if sh offset is too large, the period is the full offset + width
    if (sh_short_period_counts <= sh.offA_counts) {
      sh_short_period_counts = sh.offA_counts + 1;
    }
    
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
    read_counter  = read_counts; // will be set to 0 in the icg isr

    read_callback = callbackf;
    
    // ---------------------------------------------
    attach_isr( &sh, CMPF_MASKA_OFF, pulse_sh_isr, 128);
    attach_isr( &icg, CMPF_MASKA_ON_OFF, pulse_icg_isr, 64);
    attach_isr( &cnvst, CMPF_MASKA_OFF, pulse_cnvst_isr, 64);
      
    // ---------------------------------------------
    // stuff that other routines need to know
    mode = PULSE;
    trigger_mode = false;
    
    // timer needs this
    pulse_interframe_min_secs  = (icg.offA_counts/F_BUS_ACTUAL)*icg.divider;    // offset before read starts
    pulse_interframe_min_secs += ((float)cnvst.period_counts/F_BUS_ACTUAL)*cnvst.divider * read_counts;  // add read time
    pulse_interframe_min_secs += USBTRANSFERSECS;  // add usb transfer time

    // round-up minimum interframe time to 1 msec
    pulse_interframe_min_secs = ceil((pulse_interframe_min_secs+1.E-3)/1.E-3)*1.E-3;

    Serial.print("TCD1304 setup_pulse success, min frame interval ");
    Serial.print(pulse_interframe_min_secs,6);
    Serial.println(" secs");

    // now we set load ok and force init  - clk, sh, icg 
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((CLK_MASK|SH_MASK|ICG_MASK));
    flexpwm->SM[CLK_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;  // force out while ldok, initializes the counters

    pulse_armed  = true;
        
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ADC read to measure the zero
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  inline static bool adc_blank_running = false;

  inline static uint adc_blank_counter = 0;

  inline static uint adc_raw_offset = 0;
  inline static double adc_offset = 0.;

  inline static bool adc_offset_done = false;
  
  static void adc_blank_isr()
  {
    uint16_t status;

    uint16_t u16;
    
    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;

    if ((status&CMPF_MASKA_OFF)) {

#ifdef ALLINONEBOARD
      adc->adc0->startReadFast(ANALOGPIN);
      while ( adc->adc0->isConverting() );
      u16 = adc->adc0->readSingle();
#else      
      SETCNVST;
      delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
      CLEARCNVST;               // need 30 nanoseconds after this

      u16 = SPI.transfer16(0xFFFF);
      u16 ^= (0x1<<15);
#endif

      adc_blank_counter++;

      if (adc_blank_counter > NREADOUT) {

        adc_raw_offset += u16;

        if (adc_blank_counter >= 2*NREADOUT) {

          // stop the counter
          flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);
          flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(CNVST_MASK);

          // generate the results
          adc_blank_running = false;
        }

      }
      
    }
   
  }
  
  bool read_adc_offset()
  {
    stop_all();

    adc_blank_running = false;

    adc_raw_offset = 0;
    adc_offset = 0;
    
    adc_blank_counter = 0;

    // load the cnvst submodule
    
    cnvst.prescale = 0;
    cnvst.divider  = 1;
    
    cnvst.period_counts = 4*ceil(pulse_clk_secs*F_BUS_ACTUAL) ;

    cnvst.offset        = 0;

    cnvst.onA_counts    = ceil(pulse_cnvst_offset_secs*F_BUS_ACTUAL/cnvst.divider);    // delay to approx the fourth clock
    cnvst.offA_counts   = cnvst.onA_counts + ceil(pulse_cnvst_secs*F_BUS_ACTUAL/cnvst.divider);
    cnvst.invertA       = false;

    cnvst.onB_counts    = 0;
    cnvst.offB_counts   = 0;
    cnvst.invertB       = false;
    
    cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER; // needs to be master, because run bit is disabled when running from clk's clock.

    // check it
    Serial.println("");
    if (!print_and_check_submodule(&cnvst)) {
      error_flag = true;
      return false;
    }

    // load it
    load_submodule(&cnvst);
    
    attach_isr( &cnvst, CMPF_MASKA_OFF, adc_blank_isr, 64);

    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);

    flexpwm->SM[CNVST_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;  // force out while ldok, initializes the counters

    flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);
    
    adc_blank_running = true;

    while(adc_blank_running) {
      yield();
    }
    Serial.printf("adc_blank_measurement sum %u counter %d\n",adc_raw_offset,adc_blank_counter);
    
    adc_offset = (adc_raw_offset * VPERBIT)/NREADOUT;
    adc_raw_offset /= NREADOUT;
    adc_offset_done = true;
            
    Serial.printf("adc_blank_measurement mean %u ADU %.6g volts counter %d\n",adc_raw_offset,adc_offset,adc_blank_counter);

    return true;
  }
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pulse loop, exposure times and frame interval all managed by statemachine within isrs and the flexpwm
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  typedef enum {
    IDLE,
    START_EXPOSURE,
    WAIT_EXPOSURE,
    WAIT_FRAME,
    READOUT,
  } PULSE_LOOP_STATE;

  inline static bool pulse_loop_armed = false;
  inline static bool pulse_loop_running = false;

  inline static uint64_t cyccnt64_prepare_readout = 0;
  inline static uint64_t cyccnt64_prepare_exposure = 0;
  
  inline static PULSE_LOOP_STATE pulse_loop_state = IDLE;

  inline static float pulse_loop_frame_min_secs  = 0.;

#ifdef DEBUG
  inline static uint64_t cyccnt_at_wait_frame = 0;
  inline static uint64_t cyccnt_at_start_exposure = 0;
  inline static uint64_t cyccnt_at_wait_exposure = 0;

  static void print_sh_debugger()
  {
    double tdiff = 0.;
    
    Serial.print("debuggery wait_frame "); Serial.print(cyccnt_at_wait_frame);
    Serial.print(" start_exposure "); Serial.print(cyccnt_at_start_exposure);
    Serial.print(" wait_exposure "); Serial.print(cyccnt_at_wait_exposure);

    tdiff = (double)(cyccnt_at_start_exposure - cyccnt_at_wait_frame)/F_CPU;
    Serial.print(" start_exp - wait_frame "); Serial.print(tdiff,9);

    tdiff = (double)(cyccnt_at_wait_exposure - cyccnt_at_wait_frame)/F_CPU;
    Serial.print(" wait_exp - wait_frame "); Serial.print(tdiff,9);

    tdiff = (double)cyccnt64_prepare_exposure/F_CPU;
    Serial.print(" prepare exposure "); Serial.print(tdiff,9);

    tdiff = (double)cyccnt64_prepare_readout/F_CPU;
    Serial.print(" prepare readout "); Serial.print(tdiff,9);

    Serial.println("");
  }
#endif
  
  static void pulse_loop_sh_isr()
  {
    uint16_t status;

    // this is a trailing edge interrupt := exposure timer
    uint64_t cyccnt64_now = cycles64();
    
    // clear the interrupt
    status = flexpwm->SM[SH_SUBMODULE].STS;
    flexpwm->SM[SH_SUBMODULE].STS = status;

    sh_counter++; // set this to zero at start
    
    switch(pulse_loop_state) {

    case WAIT_FRAME:

      if ( (cyccnt64_now - sh_cyccnt64_prev) >= cyccnt64_prepare_exposure ) {

        flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((SH_MASK | ICG_MASK));
        flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;  // next cycle we have SH
        flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;        // but not ICG on
        flexpwm->SM[ICG_SUBMODULE].VAL3 = 0xFFFF;        // and not ICG off (interrupt)
        flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((SH_MASK | ICG_MASK));

        pulse_loop_state = START_EXPOSURE;

        // for elapsed time and exposure
        sh_cyccnt64_prev = sh_cyccnt64_now;
        sh_cyccnt64_now  = cyccnt64_now;

#ifdef DEBUG
        cyccnt_at_wait_frame = cyccnt64_now;
#endif
      }
      
      break;
      
    case START_EXPOSURE:

#ifdef DEBUG
      Serial.print("start exp ");
      Serial.println(cyccnt64_now);
#endif
      
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((SH_MASK | ICG_MASK));
      flexpwm->SM[SH_SUBMODULE].VAL2 = 0xFFFF;   // no more SH until exposure over
      flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;  // no more ICG on until exposure over
      flexpwm->SM[ICG_SUBMODULE].VAL3 = 0xFFFF;  // no more ICG off until exposure over
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((SH_MASK | ICG_MASK));

      pulse_loop_state = WAIT_EXPOSURE;

      // for elapsed time and exposure
      sh_cyccnt64_prev = sh_cyccnt64_now;
      sh_cyccnt64_now  = cyccnt64_now;

#ifdef DEBUG
      cyccnt_at_start_exposure = cyccnt64_now;
#endif

      break;
      
    case WAIT_EXPOSURE:

      /* Note that ICG has turn of SH and ICG and readout has to
         turn them back on and set state to wait_frame
       */

      if ((cyccnt64_now - sh_cyccnt64_prev) > cyccnt64_prepare_readout ) {

#ifdef DEBUG
        Serial.print("wait exp ");
        Serial.println(cyccnt64_now);
#endif

        flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((SH_MASK | ICG_MASK));

        flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;    // next cycle we pulse SH,

        flexpwm->SM[ICG_SUBMODULE].VAL2 = icg.onA_counts;  // and ICG togther
        flexpwm->SM[ICG_SUBMODULE].VAL3 = icg.offA_counts;

        flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((SH_MASK | ICG_MASK));
        
        pulse_loop_state = IDLE;
        
        // for elapsed time and exposure
        sh_cyccnt64_prev = sh_cyccnt64_now;
        sh_cyccnt64_now  = cyccnt64_now;

#ifdef DEBUG
        cyccnt_at_wait_exposure = cyccnt64_now;
#endif

      }
      break;


    default:

      break;
      
    }

    // diagnostics
    if (!(status&CMPF_MASKA_OFF)) {
      oops_flag = true;
      Serial.printf("OOPS! pulse loop sh without off bit set 0x%04x", status);
    }

    sh_counter++;
  }

  static void pulse_loop_icg_isr()
  {
    uint16_t status;

    status = flexpwm->SM[ICG_SUBMODULE].STS;
    flexpwm->SM[ICG_SUBMODULE].STS = status;

    //Serial.print("icg_isr "); print_counters();
    if (status & CMPF_MASKA_ON) {

      //Serial.println("pulse icg maska_on");

      // prepare to restart the CNVST clock
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(CNVST_MASK);   // clear ldok
      flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN(CNVST_MASK);    // stop
      
      flexpwm->SM[CNVST_SUBMODULE].STS = flexpwm->SM[CNVST_SUBMODULE].STS; 
      NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + CNVST_SUBMODULE);

      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(CNVST_MASK);    // set ldok
      flexpwm->SM[CNVST_SUBMODULE].CNT = 0;                // zero the submodule counter
      flexpwm->SM[CNVST_SUBMODULE].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;  // force init
      read_counter = 0;                                    // zero the readout counter
    }
    
    else if (status & CMPF_MASKA_OFF) {

      //Serial.println("pulse icg maska_off");
      
      // Start the cnvst clock
      flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(CNVST_MASK);
    
      /* Turnoff icg and sh completely because of the interrupts,until we are done with the readout,
         or we could disalbe the onA and disable interrupts, lets keep it as simple as possible.
         Note also that they are left in the off state  at the end of the cycle
      */
      flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((SH_MASK|ICG_MASK));
      flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->SM[ICG_SUBMODULE].VAL3 = 0xFFFF;
      flexpwm->SM[SH_SUBMODULE].VAL2 = 0xFFFF;
      flexpwm->SM[SH_SUBMODULE].VAL3 = 0xFFFF;
      flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((SH_MASK|ICG_MASK));

      // here is our exposure, diff to most recent previous sh gate    
      sh_cyccnt64_exposure = sh_cyccnt64_now - sh_cyccnt64_prev;

      // kluge to fix the reporting "bug"
      sh_cyccnt64_exposure += pulse_loop_period_secs * F_CPU;
    }

    // bookkeeping
    icg_counter++;
  }

  static void pulse_loop_cnvst_isr()
  {
    uint16_t status;

    status = flexpwm->SM[CNVST_SUBMODULE].STS;
    flexpwm->SM[CNVST_SUBMODULE].STS = status;

    if ((status&CMPF_MASKA_OFF)) {

#ifndef ALLINONEBOARD
        // Assert the convert pin
        SETCNVST;
        delayNanoseconds( 670 );  // 710 nanoseconds minus spi setup time
        CLEARCNVST;               // need 30 nanoseconds after this
#endif
      
      if (read_counter < read_counts) {

#ifdef ALLINONEBOARD
        adc->adc0->startReadFast(ANALOGPIN);
        while ( adc->adc0->isConverting() );
        *read_pointer = adc->adc0->readSingle();
#else      
        *read_pointer = SPI.transfer16(0xFFFF);
        *read_pointer ^= (0x1<<15);
#endif

        // bookkeeping for the read
        read_pointer++;
        read_counter++;

        // The read is done
        if (read_counter == read_counts) {

          // ----------------------------------------
          // User supplied function, per read complete, we need this 0 indexed
          // First exposure is frame 1, frame 0 is the pre-exposure
          
          frames_complete = (frame_counter+1 == frame_counts);    // both of these are reset in the fill header funnction
          framesets_complete = frames_complete && (frameset_counter+1 == frameset_counts);
          
          if (read_callback) {
            read_callback();
          }

#ifdef DEBUG
          print_sh_debugger();
#endif
          frame_counter++;            
          if (frame_counter < frame_counts) {
              
            // wait for start of next sh period
            if (flexpwm->MCTRL & FLEXPWM_MCTRL_RUN(SH_MASK)) {
              while (!(flexpwm->SM[SH_SUBMODULE].STS & FLEXPWM_SMSTS_RF));
              flexpwm->SM[SH_SUBMODULE].STS = FLEXPWM_SMSTS_RF;
            }

            // Setup the next state
            pulse_loop_state  = WAIT_FRAME;

            // Re-enable SH (not icg)
            flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(SH_MASK);
            flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;
            flexpwm->SM[SH_SUBMODULE].VAL3 = sh.offA_counts;
            flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(SH_MASK);

          }
          
          else if (frame_counter==frame_counts) {

            if (frames_completed_callback) {
	      frames_completed_callback();
            }

            frameset_counter++;
            if (frameset_counter==frameset_counts) {
              
              if (framesets_completed_callback) {
                framesets_completed_callback();
              }
            }

            pulse_loop_arm();
          }

        }

      }
      
    }
    
    // diagnostics
    cnvst_counter++;
    
  }

  // -------------------------------------------------------------------

  bool pulse_loop_wait( float timeout_=1., float timestep_=0.01,
                        bool interruptible=false, bool verbose=false)
  {
    if (verbose) {
      Serial.println("waiting for pulse_loop_running");
    }
    return generic_wait(&pulse_loop_running, timeout_, timestep_, interruptible, verbose);
  }
  
  static void pulse_loop_stop()
  {
    flexpwm_stop();

    pulse_loop_armed    = false;
    pulse_loop_running  = false;    
  }
  
  static void pulse_loop_start()
  {
    uint64_t cyccnt64_now = cycles64();
    
    if (pulse_loop_running) {
      //error_flag = true;
      Serial.println("Error: pulse start with flexpwm already running");
      error_flag = true;
      //return;
    }

    if (!pulse_loop_armed) {
      error_flag = true;
      Serial.println("Error: pulse loop not armed");
      return;
    }
    
    pulse_loop_armed    = false;  // means that we need to arm again before we can start again.
    pulse_loop_running  = true;
    
    pulse_loop_state = START_EXPOSURE;

    sh_cyccnt64_prev = sh_cyccnt64_now;
    sh_cyccnt64_now  = cyccnt64_now;
    
    flexpwm_running = true;
    flexpwm->MCTRL  |= FLEXPWM_MCTRL_RUN((CLK_MASK|SH_MASK|ICG_MASK|CNVST_MASK)); // set run clk, sh, icg
  }
  
  static void pulse_loop_arm()
  {
    if (mode != PULSE_LOOP) {
      Serial.println("Error: pulse_loop_arm called but not in pulse_loop mode");
      error_flag = true;
      return;
    }

    // clear ldok, everything
    flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK((CNVST_MASK|CLK_MASK|SH_MASK|ICG_MASK));
    flexpwm_running = false;

    // stop everything
    flexpwm->MCTRL &= ~FLEXPWM_MCTRL_RUN((CNVST_MASK|CLK_MASK|SH_MASK|ICG_MASK)); 

    // clear all of the interrupts and the pending interrupts
    flexpwm->SM[CLK_SUBMODULE].STS = flexpwm->SM[CLK_SUBMODULE].STS;
    NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + CLK_SUBMODULE);

    flexpwm->SM[SH_SUBMODULE].STS = flexpwm->SM[SH_SUBMODULE].STS;
    NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + SH_SUBMODULE);

    flexpwm->SM[ICG_SUBMODULE].STS = flexpwm->SM[ICG_SUBMODULE].STS;
    NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + ICG_SUBMODULE);

    flexpwm->SM[CNVST_SUBMODULE].STS = flexpwm->SM[CNVST_SUBMODULE].STS; 
    NVIC_CLEAR_PENDING(IRQ_FLEXPWM1_0 + CNVST_SUBMODULE);

    // Setup for SH.  CLK and CNVST are always configured
    flexpwm->SM[SH_SUBMODULE].VAL2 = sh.onA_counts;
    flexpwm->SM[SH_SUBMODULE].VAL3 = sh.offA_counts;

    // ICG is disabled, will be enabled in SH isr for the end of exposure
    flexpwm->SM[ICG_SUBMODULE].VAL2 = 0xFFFF;
    flexpwm->SM[ICG_SUBMODULE].VAL3 = 0xFFFF;
    
    /* ------------------------------------------------------------------------------
       Zero the counters - keep in mind we are now stopped and this is not the start.
       We leave everything stopped so that we can have an almost immediate start if we
       are triggered. The alternative would have clk and cnvst running, but then we
       would have jitter equal to one clk period while we resync. Therefore, full stop.
       ------------------------------------------------------------------------------
    */
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((CNVST_MASK|CLK_MASK|SH_MASK|ICG_MASK));    // set ldok

    flexpwm->SM[CLK_SUBMODULE].CNT = 0;           // zero the submodule counter
    flexpwm->SM[SH_SUBMODULE].CNT = 0;            // zero the submodule counter
    flexpwm->SM[ICG_SUBMODULE].CNT = 0;           // zero the submodule counter
    flexpwm->SM[CNVST_SUBMODULE].CNT = 0;         // zero the submodule counter

    // Force init, SH and ICG are slaved to CLK, CNVST only shares the clock
    flexpwm->SM[CLK_SUBMODULE].CTRL2   |= FLEXPWM_SMCTRL2_FORCE;  // force init
    //flexpwm->SM[SH_SUBMODULE].CTRL2    |= FLEXPWM_SMCTRL2_FORCE;  // force init
    //flexpwm->SM[ICG_SUBMODULE].CTRL2   |= FLEXPWM_SMCTRL2_FORCE;  // force init
    flexpwm->SM[CNVST_SUBMODULE].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;  // force init

    // Reset pointed and disable readout until enabled by ICG isr
    read_pointer = read_buffer;
    read_counter = read_counts;

    // starting a new frameset if we are here
    frame_counter = 0;
    frames_complete = false;

    // starting a new set of framesets?
    if (frameset_counter >= frameset_counts) {
      frameset_counter = 0;
      framesets_complete = false;
    }

    // and this is where we start
    pulse_loop_armed = true;
    pulse_loop_running = false;
  }

  
  bool setup_pulse_loop(float clk_secs, float sh_secs, float sh_offset_secs,
                        float icg_secs, float icg_offset_secs, float period_secs,
                        float exposure_secs, float frame_secs, uint nframes, 
                        uint16_t *buffer, unsigned int nbuffer, void (*callbackf)())
  {
    char cbuffer[128] = {0};

    uint64_t u64;
    float rtmp;
    
    // stops all, clears ldok, disables irqs and clears mode
    stop_all();

    // clear the error flags
    clear_error_flags();

    sprintf(cbuffer,
            "#tcd1304 setup pulse, clk %.5gs sh %.5gs offset %.5gs icg offset %.5gs",
	    clk_secs, sh_secs, sh_offset_secs, icg_offset_secs);
    Serial.println(cbuffer);
    
    // clear elapsed times
    sh_cyccnt64_start   = 0;
    sh_cyccnt64_prev    = 0;
    sh_cyccnt64_now     = 0;

    pit_cyccnt64_start   = 0;
    pit_cyccnt64_prev    = 0;
    pit_cyccnt64_now     = 0;
    
    trigger_cyccnt64_start   = 0;
    trigger_cyccnt64_prev    = 0;
    trigger_cyccnt64_now     = 0;
    
    // clear bookkeeping counters
    sh_counter    = 0;
    icg_counter   = 0;
    cnvst_counter = 0;
    
    // default to 1 frame and 1 frameset
    frame_counter = 0;
    frame_counts  = nframes ? nframes+1 : 10;

    frameset_counter = 0;
    frameset_counts = 1;

    // ------------------------------------------------------------------
    // Setup the master clock
    
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
    
    clk.prescale = 0;
    clk.divider  = 1;

    clk.period_counts = ceil(clk_secs*F_BUS_ACTUAL);    // rouund up

    clk.offset        = 0.;
    
    clk.offA_counts   = 0;
    clk.onA_counts    = (clk.period_counts/2);
    clk.invertA       = false;

    clk.onB_counts    = 0;
    clk.offB_counts   = 0;
    clk.invertB       = false;

    clk.intena_mask    = 0;

    clk.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER;
    
    // ------------------------------------------------------------------
    // Setup the CNVST clock, 4 times the clock period,
    cnvst.prescale = clk.prescale;
    cnvst.divider = (1<<cnvst.prescale);
    
    cnvst.period_counts = 4*clk.period_counts;

    cnvst.offset        = 0;

    cnvst.onA_counts    = ceil(pulse_cnvst_offset_secs*F_BUS_ACTUAL/cnvst.divider);    // delay to approx the fourth clock
    cnvst.offA_counts   = cnvst.onA_counts + ceil(pulse_cnvst_secs*F_BUS_ACTUAL/cnvst.divider);
    cnvst.invertA       = false;

    cnvst.onB_counts    = 0;
    cnvst.offB_counts   = 0;
    cnvst.invertB       = false;
    
    //cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_SYNC;
    cnvst.ctrl2_mask    = PWM_CTRL2_CLOCK_MASTER; // needs to be master, because run bit is disabled when running from clk's clock.
    
    // ------------------------------------------------------------------
    // setup the sh, sh_offset-icg_offset is the icg-sh timing, runs once, isr has to set on,off to 0xffff
    sh.prescale = clk.prescale;
    sh.divider  = (1<<sh.prescale);

    sh.period_counts  = get_next_multiple(clk.period_counts, COUNTSCEIL(period_secs));

    sh.offset         = 0.;
    
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
    icg.divider  = (1<<icg.prescale);

    icg.period_counts = sh.period_counts;

    icg.offset        = 0.;
    
    icg.onA_counts    = ceil(icg_offset_secs*F_BUS_ACTUAL/icg.divider);
    icg.offA_counts   = icg.onA_counts + ceil(icg_secs*F_BUS_ACTUAL/icg.divider);
    icg.invertA       = true;

    icg.onB_counts    = 0;
    icg.offB_counts   = 0;
    icg.invertB       = false;

    icg.ctrl2_mask     = PWM_CTRL2_CLOCK_SLAVE;
    //icg.ctrl2_mask    = FLEXPWM_SMCTRL2_CLK_SEL(0x2);

    // this is used by the timing graphics software
    cnvst.offset = (((float)icg.offA_counts / F_BUS_ACTUAL) + CNVST_START_OVERHEAD)*1.E9;
    
    // -------------------------------------------------------------------------
    // CPU CYCCNT triggers to prepare the next state (1 1/2 SH short of the  exposure,frame)
    rtmp = SECONDS(sh.period_counts*sh.divider);
    uint64_t sh_period_u64 = (uint64_t) (rtmp*F_CPU);

#define DEBUG
#ifdef DEBUG
    Serial.printf("sh period %6g cycnt64 %lu\n", rtmp, sh_period_u64);
#endif
    
    u64 = (3*sh_period_u64)/2;
    Serial.printf("3/2  => %lu\n", u64);
        
    cyccnt64_prepare_exposure  = (uint64_t) (frame_secs*F_CPU);

#ifdef DEBUG    
    Serial.printf("frame_secs %6g prepare => cycnt64 %lu", frame_secs, cyccnt64_prepare_exposure);
#endif

    cyccnt64_prepare_exposure  -= u64;
    
#ifdef DEBUG
    Serial.printf(" => cycnt64 %lu\n", cyccnt64_prepare_exposure);
#endif
    
    cyccnt64_prepare_readout   = (uint64_t) (exposure_secs*F_CPU);

#ifdef DEBUG
    Serial.printf("exposure_secs %6g prepare => cycnt64 %lu", exposure_secs, cyccnt64_prepare_readout);
#endif
    
    cyccnt64_prepare_readout  -= sh_period_u64/2;
    //cyccnt64_prepare_readout  -= u64;

#ifdef DEBUG
    Serial.printf(" => cycnt64 %lu\n", cyccnt64_prepare_readout);
#endif
#undef DEBUG
    
    // ------------------------------------------------------------------
    // needs this to check frame interval is long enough. reu
    pulse_loop_frame_min_secs  = (icg.offA_counts/F_BUS_ACTUAL)*icg.divider;    // offset before read starts
    pulse_loop_frame_min_secs += ((float)cnvst.period_counts/F_BUS_ACTUAL)*cnvst.divider * read_counts;  // add read time
    pulse_loop_frame_min_secs += USBTRANSFERSECS;  // add usb transfer time

    if (frame_secs < pulse_loop_frame_min_secs) {
      Serial.printf("Error: frame is too short for readout time %.5g\n", pulse_loop_frame_min_secs);
      return false;
    }

    if (exposure_secs < SECONDS(sh.period_counts*sh.divider)) {
      Serial.println("Error: exposure is too short for specified pulse period");
      return false;
    }
    
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

    Serial.println("");
    Serial.print("flexpwm: clearing_pulses ");
    Serial.println(sh_clearing_counts);
   
    // ------------------------------------------------------------------

    // these set cldok (clear ldok) and clear the run bit for each submodule
    load_submodule(&clk);
    load_submodule(&icg);
    load_submodule(&sh);
    load_submodule(&cnvst);

    // ---------------------------------------------
    // Setup the readout buffer
    read_buffer   = buffer;
    read_counts   = nbuffer;
      
    read_pointer  = read_buffer;
    read_counter  = read_counts; // will be set to 0 in the icg isr
    
    read_callback = callbackf;
    
    // ---------------------------------------------
    attach_isr( &sh, CMPF_MASKA_OFF, pulse_loop_sh_isr, 128);
    attach_isr( &icg, CMPF_MASKA_ON_OFF, pulse_loop_icg_isr, 64);
    attach_isr( &cnvst, CMPF_MASKA_OFF, pulse_loop_cnvst_isr, 64);
      
    // ---------------------------------------------
    // stuff that other routines need to know
    mode = PULSE_LOOP;
    trigger_mode = false;    

    Serial.print("TCD1304 setup_pulse_loop success, min frame interval ");
    Serial.print(pulse_interframe_min_secs,6);
    Serial.println(" secs");

    // now we set load ok and force init  - clk, sh, icg
    /*  left over coode from setup_pulse()
    flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK((CLK_MASK|SH_MASK|ICG_MASK));
    flexpwm->SM[CLK_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_FORCE;  // force out while ldok, initializes the counters
    */
    // ----------------------------------------------
    // and now we arm it, it is on a hair trigger after this
    pulse_loop_arm();
        
    return true;
  }

  /* =============================================================
     Periodic interrupt timer (PIT)
   */

#define PIT_CHANNEL 0  // the teensy libraries will see that it is enabled and take something else
#define PIT_FREQ 24000000.0f
#define PITSECS(a) ((a)/PIT_FREQ)
#define PITCOUNTS(a) (uint32_t)((a)*PIT_FREQ))
  
  inline static float pit_delay_secs = 1.E-6;
  inline static float pit_interval_secs = 0.1;

  inline static uint32_t pit_delay_counts = 0;
  inline static uint32_t pit_interval_counts = 0;
  inline static uint8_t pit_state = 0;

  inline static unsigned int pit_counts = 2;
  inline static unsigned int pit_counter = 0;

  inline static bool pit_running = false;
  inline static bool pit_begun = false;

  inline static void (*pit_callback)() = 0;

  
  void begin_pit() {

    // Enable PIT Clock
    CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
    PIT_MCR = 0; // Turn on PIT module
        
    attachInterruptVector(IRQ_PIT, pit_isr);
    NVIC_SET_PRIORITY(IRQ_PIT, 64); // High priority (lower number = higher priority)
    NVIC_ENABLE_IRQ(IRQ_PIT);

    pit_begun = true;

    Serial.println("begin_pit done");
  }
  
  static void pit_isr() {

    uint64_t cyccnt64_now = cycles64();

    pit_cyccnt64_prev = pit_cyccnt64_now;
    pit_cyccnt64_now  = cyccnt64_now;

    //digitalToggleFast(9);   // 8nsecs
    
    // Check if our specific channel triggered the interrupt
    if (IMXRT_PIT_CHANNELS[PIT_CHANNEL].TFLG & 1) {   // Test flag
      IMXRT_PIT_CHANNELS[PIT_CHANNEL].TFLG = 1;       // Clear flag - both together 300 nsecs

      //digitalToggleFast(9);

      if (pit_state == 0) {
        // --- FIRST PULSE (End of Delay) ---
        pulse_start();        // 80 nsecs

        //digitalToggleFast(9);
        
        // Force an immediate reload of the recurring interval value  - this takes 480 nsecs
        IMXRT_PIT_CHANNELS[PIT_CHANNEL].TCTRL = 0; 
        IMXRT_PIT_CHANNELS[PIT_CHANNEL].LDVAL = pit_interval_counts;
        IMXRT_PIT_CHANNELS[PIT_CHANNEL].TCTRL = 3;                      // altogether 480nsecs
        
        //digitalToggleFast(9);
        
        pit_state = 1;
      } else {
        // --- SUBSEQUENT PULSES (Recurring Interval) ---
        pulse_start();       // 80 nsecs

        //digitalToggleFast(9);
      }
    }
    
    
    pit_counter++;

    if (pit_counter >= pit_counts) {
      IMXRT_PIT_CHANNELS[PIT_CHANNEL].TCTRL = 0;
      IMXRT_PIT_CHANNELS[PIT_CHANNEL].TFLG = 1;
      pit_running = false;
    }
  }


  bool pit_wait(float timeout_=1., float timestep_=0.01, bool interruptible=false, bool verbose=false)
  {
    if (verbose) {
      Serial.print("waiting for pit_running "); Serial.print(timeout_);
      Serial.print(" "); Serial.println(timestep_);
    }
    return generic_wait(&pit_running, timeout_, timestep_, interruptible, verbose);
  }
  
  static void pit_start()
  {
    // PIT runs on 24MHz or 150MHz depending on root clock. 
    // Standard Teensy 4.x is 24MHz for PIT by default.

    if (pit_running) {
      error_flag = true;
      Serial.println("Error: pit_start() but already running");
      return;
    }
    
    pit_cyccnt64_start = cycles64();
    pit_cyccnt64_prev  = pit_cyccnt64_start;
    pit_cyccnt64_now   = pit_cyccnt64_start;

    if (pulse_armed  && frame_counts && pit_counts) {
    
      oops_flag = false;
      error_flag = false;
      
      pit_running = true;  // do it before the start, in case isr changes it

      frame_counter = 0;

      pit_counter = 0;

      pit_state = 0;

      IMXRT_PIT_CHANNELS[PIT_CHANNEL].LDVAL = pit_delay_counts;      
      IMXRT_PIT_CHANNELS[PIT_CHANNEL].TCTRL = 3;    // TEN=1 (Enable), TIE=1 (Interrupt Enable)
    }

    else {
      
      error_flag = true;
      
      Serial.print("Error: pit_start() - but not setup ");
      Serial.print(" pulse armed " ); Serial.print(pulse_armed);
      Serial.print(" frame_counts "); Serial.println(frame_counts);

      stop_pit();
    }

    //Serial.println("pit_start done");
  }    

  static void stop_pit() {
    Serial.println("stop_pit");
    IMXRT_PIT_CHANNELS[PIT_CHANNEL].TCTRL = 0;
    IMXRT_PIT_CHANNELS[PIT_CHANNEL].TFLG = 1;
    pit_running = false;
  }
  

  bool setup_pit(float exposure_secs, float delay_secs=1.E-6, unsigned int ncounts=0)
  {
    // we cannot touch the run mode, only clear the trigger mode
    trigger_mode = false;

    // clear error and oops flags
    clear_error_flags();
    
    // reset elapsed time counters
    pit_cyccnt64_start   = 0;
    pit_cyccnt64_prev    = 0;
    pit_cyccnt64_now     = 0;

    // Check, timer is for single pulse flexpwm only
    if (mode!=PULSE) {
      Serial.println("Error: setup_pit, need to call setup_pulse() first");
      error_flag = true;
      return false;
    }

    // Check, minimum exposure is the readout and transfer time
    if (exposure_secs < pulse_interframe_min_secs) {
      Serial.printf( "Error: setup_pit exposure %.6gs is too short for this pulse configuration, need at least %.6g",
              exposure_secs,pulse_interframe_min_secs);
      error_flag = true;
      return false;
    }

    if (delay_secs < 0.95E-6) {
      Serial.printf( "Error: setup_pit delay %.6gs is too short, need at least 1usec", delay_secs);
      error_flag = true;
      return false;
    }

    /* ======================================================
       Setup timer callback, counts and period
    */
    pit_callback  = pulse_start;
    pit_counts    = ncounts ? ncounts+1 : frame_counts;
    frame_counts  = pit_counts;

    // Exposure 
    pit_interval_secs = exposure_secs;
    pit_delay_secs = delay_secs;

    pit_delay_counts = (uint32_t)(pit_delay_secs * PIT_FREQ);
    pit_interval_counts = (uint32_t)(pit_interval_secs * PIT_FREQ);

    // 0 is legal, but everything expires in N+1 tick.
    if (pit_delay_counts > 0) pit_delay_counts--;
    if (pit_interval_counts > 0) pit_interval_counts--;
    
    pit_state = 0;
     
    Serial.printf("setup_pit pit_counts %d frame_counts %d delay %.6g (%d) interval %.6g (%d)\n",
                  pit_counts, frame_counts, PITSECS(pit_delay_counts+1), pit_delay_counts,
                  PITSECS(pit_interval_counts+1), pit_interval_counts);
    
    // ==============================================

    if (!pit_begun) {
      begin_pit();
      pit_begun = true;
    }
    
    // ==============================================
    // pit mode is enabled
    mode = PIT; 

    Serial.println("setup_pit done");
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
      error_flag = true;
      return false;
    }
    if (trigger_attached) {
      Serial.println("start triggers but trigger already attached");
      error_flag = true;
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

  bool wait_trigger(float timeout_=1., float timestep_=0.01, bool interruptible=false, bool verbose=false)
  {
    if (verbose) {
      Serial.println("waiting for trigger attached");
    }

    return generic_wait(&trigger_attached,timeout_,timestep_,interruptible,verbose);
  }

  
  bool setup_triggers(unsigned int ncounts=0)
  {
    // we cannot clear the operating mode, only the trigger mode.
    trigger_mode = false;

    // but we must clear the error flags at this point
    clear_error_flags();

    switch(mode) {
    case PULSE:
      Serial.println("trigger callback = pulse_start()");
      trigger_callback = pulse_start;
      trigger_counts   = ncounts ? ncounts : frame_counts;
      if (trigger_counts) {
        frame_counts = trigger_counts;
        trigger_mode = true;
      }
      else {
        Serial.println("Error: setup_interrupt pulse, but no frame_counts");
        error_flag = true;
      }
      break;
    case PIT:
      Serial.println("trigger callback = pit_start()");
      trigger_callback = pit_start;
      trigger_counts   = ncounts ? ncounts : 1;
      trigger_mode = true;
      break;
    case PULSE_LOOP:
      Serial.println("trigger callback = pulse_loop_start()");
      trigger_callback = pulse_loop_start;
      trigger_counts   = ncounts ? ncounts : 1;
      trigger_mode = true;
      break;
    default:
      Serial.println("Error: setup_interrupt, but not configured");
      error_flag = true;
      break;
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

  void set_cnt(uint8_t submod,uint16_t value, IMXRT_FLEXPWM_t *q=flexpwm) {
    if (submod<4) {
      q->SM[submod].CNT = value;
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
    stop_all();

    // Reactivation as digital i/o pins
    setup_digital_pins();
    
  }
  
};

#endif
