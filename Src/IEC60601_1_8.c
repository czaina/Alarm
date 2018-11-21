/****************************************************************
 *  IEC60601-1-8 
 *
 *   This modules provides the IEC Medical Alert tones.
 *
 *
 *   Copyright(C) 2007, NXP Semiconductor
 *   All rights reserved.
 *
 *   Port to mbed 2012 (WH)
 ****************************************************************/
#include "IEC60601_1_8.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "math.h"
 
#define DEBUG_ON       0           // Activate Testpins for timing 
#if(DEBUG_ON)
// Activate Testpins for timing 
DigitalOut TimInt(p19);  
DigitalOut SeqInt(p20);  
#endif
 
#define AMPL           50//150//200         // Output Amplitude
#define PI             3.1415926
#define FSAMPLE        25000       // Timer  Reload Frequency               
  
 
// define rise and fall time of tones
#define HP_RISE        10 //15          // rise time setting for high priority tones (12 = ~20ms Tr)
                                   //   decrease to make slower, increase to make faster
#define HP_FALL        15 //15          // fall time setting for high priority tones (12 = ~20ms Tf)
                                   //   decrease to make slower, increase to make faster
#define MP_RISE        10           // rise time setting for high priority tones (8 = ~30ms Tr)
                                   //   decrease to make slower, increase to make faster
#define MP_FALL        10           // fall time setting for high priority tones (8 = ~30ms Tf)
                                   //   decrease to make slower, increase to make faster

Alarm_Type _alarm_type;
Prio_Type _priority;
volatile unsigned int _sequence;
volatile unsigned int _mscount;
volatile unsigned int _fall_begin;
volatile unsigned int _rise_begin;
volatile unsigned int _note_count;


int _envelope;
Note_Type _active_note;
bool _note_on;
int _note_level;
bool _envelope_on;
bool _envelope_off;
#ifdef STM32F303x8 //AP31P
	extern DAC_HandleTypeDef hdac1;
#else //Nucleo32 F303
	extern DAC_HandleTypeDef hdac;
#endif
extern DMA_HandleTypeDef hdma_dac1_ch1;

#define HP_DT 150
#define MP_DT 200
#define LP_DT 200
#define ___DT 0

#define HP_ST 75
#define MP_ST 200
#define LP_ST 200
#define ___ST 0

Note_Type const _TuneSequence [][11] = {{C4,C4,C4,C4,C4,C4,C4,C4,C4,C4},                    // general
                                       {C4,E4,G4,G4,C5,C4,E4,G4,G4,C5},                     // cardiovascular
                                       {C4,Fsharp4,C4,C4,Fsharp4,C4,Fsharp4,C4,C4,Fsharp4}, // perfusion
                                       {C4,A4,F4,A4,F4,C4,A4,F4,A4,F4},                     // ventilation
                                       {C5,B4,A4,G4,F4,C5,B4,A4,G4,F4},                     // oxygen
                                       {C4,D4,E4,F4,G4,C4,D4,E4,F4,G4},                     // temperature
                                       {C5,D4,G4,C5,D4,C5,D4,G4,C5,D4},                     // drug_delivery
                                       {C5,C4,C4,C5,C4,C5,C4,C4,C5,C4},                     // power_fail
                                       {E4,C4,C4,C4,C4,E4,C4,C4,C4,C4}};                    // low_alarm
//Three priority times
int const _DurationTimes[3][11] = {{HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,HP_DT,___DT},// note 1..10 td time ms
                                   {MP_DT,MP_DT,MP_DT,___DT},
                                   {LP_DT,LP_DT,LP_DT,___DT}};

int const _SpaceTimes[3][11] = {{HP_ST,HP_ST,(2*HP_ST+HP_DT),HP_ST,550,HP_ST,HP_ST,(2*HP_ST+HP_DT),HP_ST,HP_ST,___ST},// note 1..10 ts time ms
                               {MP_ST,MP_ST,MP_ST,___ST},
                               {LP_ST,LP_ST,LP_ST,___ST}};
int const _NoteLevel[3][11] = {{170,255,255,255,255,255,255,255,255,255,0},//255 max level
                               {170,255,255,0},
                               {170,255,255,0}};

int const _RiseTimes[3] = {(0.3*HP_DT),(0.3*MP_DT),(0.3*LP_DT)}; //ms

int const _FallTimes[3] = {(HP_ST-0.3*HP_DT),(MP_ST-0.3*MP_DT),(LP_ST-0.3*LP_DT)}; //ms

double const _FreqArray[][5]= {{440.000005,880.00,1320.0,1760.00,2200.00},     // A4
                              {493.883306,987.76,1481.64,1975.5,2469.4},          // D46
                              {523.251136,1046.50,1569.756,2093.00,2616.25},        // E4
                              {587.329542,1174.660,1761.990,2349.320,2936.650},        // F4
                              {659.255121,1318.500,1977.750,2637.000,3296.250},       // FSharp4
                              {698.456470,1396.9720,2095.380,2793.840,3492.300},          // G4
                              {739.988853,1479.980,2219.970,2959.960,3699.950},       // A4
                              {783.990880,1567.98,2351.97,3135.96, 3919.95},         // B4
                              {880.000009,1760.00, 2640.00,3520.00, 4400.00}};   // A5
      
unsigned char _ToneWeights[] = {255,255,255,255,255};    // used for test and  
                                                         // adjusting harmonic levels

struct wave _Waves[9][5];    // 'Waves' holds tone gen coefficients and variables
                             // the coefficients are calculated during initialization.


void IEC60601_init(void) {
//	IEC60601_InitDAC();
    IEC60601_InitSequencer();
    IEC60601_InitToneCoefArray();
//    _ticker.attach_us(this, &IEC60601::_TimerInteruptHandler, 40);
}



void IEC60601_TurnOnAlarm(Prio_Type priority, Alarm_Type alarm_type) {
   
  _priority = priority;
  _alarm_type = alarm_type;
  
  _mscount = 0;    
  _sequence = 1;

  _rise_begin = 0;
  _note_count = 0;
  _fall_begin = _rise_begin + _RiseTimes[_priority] + _DurationTimes[_priority][_note_count];
}

void IEC60601_TurnOffAlarm(void) {
  _sequence = 0;
}

void IEC60601_TestAlarm(Note_Type active_note, int w0, int w1, int w2, int w3, int w4) {
         
  _priority = TEST;
    
  _active_note = active_note;
  _ToneWeights[0]= w0;
  _ToneWeights[1]= w1;
  _ToneWeights[2]= w2;
  _ToneWeights[3]= w3;
  _ToneWeights[4]= w4;
  
  _mscount = 0;    
  _sequence = 1;
}


// This modules provides the note sequencers and envelope control 
// functions for the alarm notes for the IEC Medical Alert tone demo.

void IEC60601_InitSequencer(void) {
  _envelope_on = false;
  _envelope_off = false;
}


void IEC60601_TurnOnNote(void) {
  _envelope = 0;
  _note_on = true;
  _envelope_on = true;
}

void IEC60601_TurnOffNote(void) {
  _note_on = false;
}

void IEC60601_Sequence (void) {
	if (_mscount >= _rise_begin)
	{
        if (_DurationTimes[_priority][_note_count] == 0){
            _sequence = 0;
            return;//last note
        }

		_active_note = _TuneSequence [_alarm_type][_note_count];  // n-th note of sequence
        _note_level = _NoteLevel[_priority][_note_count];
        IEC60601_TurnOnNote();
        _rise_begin = _fall_begin + _SpaceTimes[_priority][_note_count];

	}
	if (_mscount >= _fall_begin)
	{
		_note_on = false;
		_fall_begin = _rise_begin + _RiseTimes[_priority] + _DurationTimes[_priority][_note_count];
		_note_count++;
	}
}



void IEC60601_EnvelopeControl(void) {

   if (_note_on) {
        if (_envelope >= _note_level) {
         _envelope = _note_level;
     }
     else {
    	 _envelope += _note_level/_RiseTimes[_priority];
     }
   }
  else {//_note_on == false
      if (_envelope > 0) {
    	  _envelope -= _note_level/_FallTimes[_priority];
      }
  }
  if ((_envelope <= 0) && (!_note_on) && (_envelope_on) ) {
    _envelope = 0;
    _envelope_off = true;              // synchronize with zero cross
  }
}



// This module generates multiple sine waves that are combined
// to generate tones that contain a fundamental and 4 harmonics
// per the IEC60601-1-8 Medical Alarm specification 

void IEC60601_InitDAC(void) {

//    LPC_PINCON->PINSEL1  &= ~0x00300000;         // enable DAC P0.26
//    LPC_PINCON->PINSEL1  |=  0x00200000;
//    LPC_PINCON->PINMODE1 &= ~0x00300000;         // disable Rs on P0.26
//    LPC_PINCON->PINMODE1 |=  0x00200000;

//    LPC_DAC->DACR = 0x8000;                  // init DAC to half on voltage
}


void IEC60601_InitToneCoefArray(void) {      // generate the coefficients and init array for tones

  unsigned char n;
  unsigned char j;

  for (j=0;j<9;j++)                // Initialize all nine scale tones (C4-C5)
  {
     for (n=0;n<5;n++)             // fundamental and 4 harmonics for IEC60601-1-8
     {
        _Waves[j][n].coef = ((cos (2*PI*(float)(_FreqArray[j][n]/FSAMPLE)))* 32768) ;       // 2* taken out, put in final calc as a shift
        _Waves[j][n].y1   = 0;
        _Waves[j][n].y2   = ((sin (2*PI*(float)((_FreqArray[j][n]/FSAMPLE))) * AMPL * 32768));    // Try 8388608 (+8 bits) w/ long coef
     }
  }
}


void IEC60601_GenerateMultiTone (struct wave *t) {
  long int y;
  int i;
  int env_weights;
  long int output;
  static long int output_old;

  output = 0;                // clear output accumulator 
  for (i=0; i<5; i++) {        // cycle through the 5 structures in the array
    y = ((t->coef *(long long)(t->y1)>>14)) - t->y2;   // Goertzel Calculation
    t->y2 = t->y1;                                     // store for next time
    t->y1 = y;                                         // store for next time
    env_weights = _envelope * _ToneWeights[i]>>8;
    output += ((t->y1 * env_weights) >> 8);      // sum fundamental and harmonics
    t++;                                      // increment structure pointer
  }
#ifdef STM32F303x8 //AP31P
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_L, ((output >> 8) & 0xFFF0) + 0x8000);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
#else //Nucleo32 F303
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_L, ((output >> 8) & 0xFFF0) + 0x8000);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
#endif

  
  if ((output >= 0) && (output_old <= 0)) {  // zero crossing detect
     if (_envelope_off && (!_note_on)) {
        _envelope_on = false;      // sychronizes turn off with zero cross
        _envelope_off = false;     // reset envelope flag 
     }
  }     
  output_old = output;
}

#if(0)
void IEC60601_OutputTones(Note_Type note, unsigned char level) {

  _note_level = level;                   
  IEC60601_GenerateMultiTone (&Waves[note][0]);
}
#endif


void IEC60601_TimerInteruptHandler (void){
  static int timeval = 0;
  
#if(DEBUG_ON)
// Activate Testpins for timing 
    TimInt = 1;  
#endif
  
    if (_envelope_on) {
//Oude code 
//      _OutputTones(_active_note, _note_level); // parameters are set in sequencer
    	IEC60601_GenerateMultiTone (&_Waves[_active_note][0]);    // parameters set in sequencer
    }            

    timeval++;
    if (timeval == 25) {          // millisecond interval (@ 25 khz sample rate)

#if(DEBUG_ON)
        // Activate Testpins for timing 
        SeqInt=1;  
#endif  
        if (_sequence != 0) {
        	IEC60601_Sequence();
        }
        timeval = 0;            // clear interval counter
        _mscount++;             // increment ms counter
        IEC60601_EnvelopeControl();

#if(DEBUG_ON)
        // Activate Testpins for timing 
        SeqInt=0;  
#endif
    }

#if(DEBUG_ON)
// Activate Testpins for timing 
    TimInt = 0;  
#endif

}



