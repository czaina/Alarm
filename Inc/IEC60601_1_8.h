/****************************************************************
 *  IEC601601-1-8 
 *
 *   This modules provides the IEC Medical Alert tones.
 *
 *
 *   Copyright(C) 2007, NXP Semiconductor
 *   All rights reserved.
 *
 *   Port to mbed 2012 (WH)
 ****************************************************************/
 
#ifndef _IEC60601_1_8_H
#define _IEC60601_1_8_H

typedef enum  {C4=0,D4=1,E4=2,F4=3,Fsharp4=4,G4=5,A4=6,B4=7,C5=8} Note_Type;      // Can address array rows with notes

typedef enum  {GENERAL=0, CARDIOVASCULAR=1, PERFUSION=2, VENTILATION=3,
                 TEMPERATURE=4, OXYGEN=5, DRUG_DELIVERY=6, POWER_FAIL=7, LOW_ALARM=8} Alarm_Type; // Can tone_seq array rows with alarms

typedef enum  {HIGH, MEDIUM, LOW, TEST} Prio_Type;
#define false 0
#define true 1
typedef int bool; // or #define bool int

struct wave  {                  // struct for Sine Wave Generator Signal   
  short coef;                      // IIR filter coefficient                
  long y1;                         // y[-1] value                            
  long y2;                         // y[-2] value      
};  


void  IEC60601_init(void);
  void IEC60601_TurnOnAlarm(Prio_Type priority, Alarm_Type alarm_type);
  void IEC60601_TurnOffAlarm(void);
  void IEC60601_TestAlarm(Note_Type active_note, int w0, int w1, int w2, int w3, int w4);

  void IEC60601_InitSequencer(void);
  void IEC60601_InitDAC(void);
  void IEC60601_InitToneCoefArray(void);
  void IEC60601_TimerInteruptHandler (void);

  void IEC60601_GenerateMultiTone (struct wave *t);
//  void _OutputTones(Note_Type note, unsigned char level);
  void IEC60601_TurnOnNote(void);
  void IEC60601_TurnOffNote(void);
  void IEC60601_EnvelopeControl(void);
  void IEC60601_HighPriSequence (void);
  void IEC60601_MedPriSequence (void);
  void IEC60601_LowPriSequence (void);
    

 #endif
