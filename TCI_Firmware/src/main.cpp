#include <Arduino.h>
#include <BLEMidi.h>

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long bres1;       
unsigned long note1freq;  
unsigned char note1_duty;
uint8_t note1_note;
bool note1on = false;

unsigned long bres2;       
unsigned long note2freq;   
unsigned char note2_duty;
uint8_t note2_note;
bool note2on = false;

unsigned long bres3;       
unsigned long note3freq;   
unsigned char note3_duty;
uint8_t note3_note;
bool note3on = false;

void connected();

int PitchToFreq(int pitch)
{	
	return (int) (220.0 * pow(pow(2.0, 1.0/12.0), pitch - 57) + 0.5);
}

void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  Serial.printf("Received note on : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);
  if (note1on == false)
  {
    note1on = true;
    note1freq = PitchToFreq(note);
    note1_note = note;
  }
  else if (note2on == false)
  {
    note2on = true;
    note2freq = PitchToFreq(note); 
    note2_note = note;
  }
  else if (note3on == false)
  {
    note3on = true;
    note3freq = PitchToFreq(note); 
    note3_note = note;
  }
}

void onNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  Serial.printf("Received note off : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);
  if (note == note1_note)
  {
    note1on = false;
  }
  if (note == note2_note)
  {
    note2on = false;
  }
  if (note == note3_note)
  {
    note3on = false;
  }
}

void onControlChange(uint8_t channel, uint8_t controller, uint8_t value, uint16_t timestamp)
{
    Serial.printf("Received control change : channel %d, controller %d, value %d (timestamp %dms)\n", channel, controller, value, timestamp);
}

void connected()
{
  Serial.println("Connected");
}

void IRAM_ATTR onTimer() {
  if(note1on==1)           
  {
    bres1 += note1freq;    
    if(bres1 >= 100000)     
    {
      bres1 -= 100000;
      note1_duty = 20;
    }
  }
  if(note2on==1)           
  {
    bres2 += note2freq;    
    if(bres2 >= 100000)     
    {
      bres2 -= 100000;
      note2_duty = 20;
    }
  }
  if(note3on==1)           
  {
    bres3 += note3freq;    
    if(bres3 >= 100000)     
    {
      bres3 -= 100000;
      note3_duty = 20;
    }
  }



  if(note1_duty || note2_duty || note3_duty)
  {
    if (note1_duty)
    {
      note1_duty--;
    }
    if (note2_duty)
    {
      note2_duty--;
    }
    if (note3_duty)
    {
      note3_duty--;
    }
    digitalWrite(2, HIGH);
  }
  else
  {
    digitalWrite(2, LOW);
  } 
 
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1000);
  Serial.begin(115200);
  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);
  Serial.println("Sistema iniciado");
  BLEMidiServer.begin("MIDI device");
  BLEMidiServer.setOnConnectCallback(connected);
  BLEMidiServer.setOnDisconnectCallback([](){     // To show how to make a callback with a lambda function
  Serial.println("Disconnected");
  });
  BLEMidiServer.setNoteOnCallback(onNoteOn);
  BLEMidiServer.setNoteOffCallback(onNoteOff);
  BLEMidiServer.setControlChangeCallback(onControlChange);
  //BLEMidiServer.enableDebugging();
}

void loop() {
  if (BLEMidiServer.isConnected()) {
      
  }
  delay(2000);
}
