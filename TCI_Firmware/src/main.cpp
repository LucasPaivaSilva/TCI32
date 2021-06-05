#include <Arduino.h>
#include <BLEMidi.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(128 , 64, &Wire, -1);

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long bres1;       
unsigned long note1freq;  
unsigned char note1_duty;
uint8_t note1_note;
bool note1on = false;
int note1toDisplay = 0;

unsigned long bres2;       
unsigned long note2freq;   
unsigned char note2_duty;
uint8_t note2_note;
bool note2on = false;
int note2toDisplay = 0;

unsigned long bres3;       
unsigned long note3freq;   
unsigned char note3_duty;
uint8_t note3_note;
bool note3on = false;
int note3toDisplay = 0;

void connected();
void SetupMidi();
void SetupInterrupts();
void SetupDisplay();

int PitchToFreq(int pitch)
{	
  int freq = (int) (220.0 * pow(pow(2.0, 1.0/12.0), pitch - 57) + 0.5);
  if (freq>999)
  {
    return 999;
  }
	return freq;
}

void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  Serial.printf("Received note on : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);
  if (note1on == false)
  {
    note1on = true;
    note1freq = PitchToFreq(note);
    note1_note = note;
    note1toDisplay = note1freq;
  }
  else if (note2on == false)
  {
    note2on = true;
    note2freq = PitchToFreq(note); 
    note2_note = note;
    note2toDisplay = note2freq;
  }
  else if (note3on == false)
  {
    note3on = true;
    note3freq = PitchToFreq(note); 
    note3_note = note;
    note3toDisplay = note3freq;
  }
}

void onNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  Serial.printf("Received note off : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);
  if (note == note1_note)
  {
    note1on = false;
    //note1toDisplay = 0;
  }
  if (note == note2_note)
  {
    note2on = false;
    //note2toDisplay = 0;
  }
  if (note == note3_note)
  {
    note3on = false;
    //note3toDisplay = 0;
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

void SetupMidi()
{
  BLEMidiServer.begin("TCI_MIDI");
  BLEMidiServer.setOnConnectCallback(connected);
  BLEMidiServer.setOnDisconnectCallback([](){     // To show how to make a callback with a lambda function
  Serial.println("Disconnected");
  });
  BLEMidiServer.setNoteOnCallback(onNoteOn);
  BLEMidiServer.setNoteOffCallback(onNoteOff);
  BLEMidiServer.setControlChangeCallback(onControlChange);
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

void SetupInterrupts()
{
  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);
}

void UpdateMIDIMode(bool DrawEntireDisplay, bool UptadeInfo)
{
  if (DrawEntireDisplay)
  {
    display.clearDisplay();
    display.setTextSize(1);            
    display.setTextColor(SSD1306_WHITE);       
    display.setCursor(0,0);            
    display.println(F("TCI 0.5"));
    display.println(F(""));
    display.println(F("Note1 > Off 000 Hz"));
    display.println(F("Note2 > Off 000 Hz"));
    display.println(F("Note3 > Off 000 Hz"));
    display.display();
  }
  if (UptadeInfo)
  {
    display.clearDisplay();
    display.setTextSize(1);            
    display.setTextColor(SSD1306_WHITE);       
    display.setCursor(0,0);            
    display.println(F("TCI 0.5"));
    display.println(F(""));

    display.print(F("Note1 > "));
    if (note1on)
    {
      display.print(F("On  "));
    }
    else
    {
      display.print(F("Off "));
    }
    display.print((note1toDisplay));
    display.println(F(" Hz"));

    display.print(F("Note2 > "));
    if (note2on)
    {
      display.print(F("On  "));
    }
    else
    {
      display.print(F("Off "));
    }
    display.print((note2toDisplay));
    display.println(F(" Hz"));

    display.print(F("Note3 > "));
    if (note3on)
    {
      display.print(F("On  "));
    }
    else
    {
      display.print(F("Off "));
    }
    display.print((note3toDisplay));
    display.println(F(" Hz"));
    display.display();
  }

  
}

void SetupDisplay()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F(""));
  display.println(F("TCI 0.5"));
  display.println(F("By Paiva"));
  display.display();
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  Serial.begin(115200);
  SetupDisplay();
  SetupInterrupts();
  SetupMidi();
  Serial.println("Sistema iniciado");
  delay(1000);
  UpdateMIDIMode(true, false);
}

void loop() {
  delay(100);
  UpdateMIDIMode(false, true);
}
