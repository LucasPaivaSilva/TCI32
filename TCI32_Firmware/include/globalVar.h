#ifndef GLOBALVAR_H
#define GLOBALVAR_H

// Global Variables

extern volatile float pitchTable[];

extern volatile int masterDuty;

extern volatile int bres1;
extern volatile int note1Freq;
extern volatile int note1Note;
extern volatile unsigned char note1Duty;
extern volatile unsigned char note1DutyMaster;
extern volatile unsigned char note1On;

extern volatile int bres2;
extern volatile int note2Freq;
extern volatile int note2Note;
extern volatile unsigned char note2Duty;
extern volatile unsigned char note2DutyMaster;
extern volatile unsigned char note2On;

#endif