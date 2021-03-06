#include <Arduino.h>
#include <BLEMidi.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AiEsp32RotaryEncoder.h"

/*
connecting Rotary encoder

Rotary encoder side    MICROCONTROLLER side  
-------------------    ---------------------------------------------------------------------
CLK (A pin)            any microcontroler intput pin with interrupt -> in this example pin 32
DT (B pin)             any microcontroler intput pin with interrupt -> in this example pin 21
SW (button pin)        any microcontroler intput pin with interrupt -> in this example pin 25
GND - to microcontroler GND
VCC                    microcontroler VCC (then set ROTARY_ENCODER_VCC_PIN -1) 

***OR in case VCC pin is not free you can cheat and connect:***
VCC                    any microcontroler output pin - but set also ROTARY_ENCODER_VCC_PIN 25 
                        in this example pin 25

*/
#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 25
#define ROTARY_ENCODER_BUTTON_PIN 32
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

//depending on your encoder - try 1,2 or 4 to get expected behaviour
//#define ROTARY_ENCODER_STEPS 1
//#define ROTARY_ENCODER_STEPS 2
#define ROTARY_ENCODER_STEPS 4

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(128 , 64, &Wire, -1);

int TimeSinceDisplayUpdate = 0;
int DisplayTestVaribale = 0;
int DutyBorder = 0;
int SOCTimer = 0;
int BaterrySOC = 0;

volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long bres1;       
unsigned long note1freq; 
unsigned long note1speed;  
unsigned char note1_duty;
uint8_t note1_note;
bool note1on = false;
int note1toDisplay = 0;

unsigned long bres2;       
unsigned long note2freq;  
unsigned long note2speed;  
unsigned char note2_duty;
uint8_t note2_note;
bool note2on = false;
int note2toDisplay = 0;

unsigned long bres3;       
unsigned long note3freq;   
unsigned long note3speed; 
unsigned char note3_duty;
uint8_t note3_note;
bool note3on = false;
int note3toDisplay = 0;

void SetupMidi();
void SetupInterrupts();
void SetupDisplay();

int GetOnTime(int freq)
{
	int on_time = 1;
	if (freq < 700)  {on_time = 2;}
	if (freq < 600)  {on_time = 2;}
	if (freq < 500)  {on_time = 3;}
	if (freq < 400)  {on_time = 3;}
	if (freq < 300)  {on_time = 3;}
	if (freq < 200)  {on_time = 4;}
	if (freq < 100)  {on_time = 4;}
	on_time = on_time * DutyBorder;
	return on_time;
}

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
    note1speed = GetOnTime(note1freq);
  }
  else if (note2on == false)
  {
    note2on = true;
    note2freq = PitchToFreq(note); 
    note2_note = note;
    note2toDisplay = note2freq;
    note2speed = GetOnTime(note2freq);
  }
  else if (note3on == false)
  {
    note3on = true;
    note3freq = PitchToFreq(note); 
    note3_note = note;
    note3toDisplay = note3freq;
    note3speed = GetOnTime(note3freq);
  }
}

void onNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp)
{
  if ((note == note1_note))
  {
    note1on = false;
    //note1toDisplay = 0;
    Serial.print("note 1 off");
  }
  if ((note == note2_note))
  {
    note2on = false;
    //note2toDisplay = 0;
  }
  if ((note == note3_note))
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
  BLEMidiServer.begin("Tesla Coil Interrupter");
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
      note1_duty = note1speed;
    }
  }
  if(note2on==1)           
  {
    bres2 += note2freq;    
    if(bres2 >= 100000)     
    {
      bres2 -= 100000;
      note2_duty = note2speed;
    }
  }
  if(note3on==1)           
  {
    bres3 += note3freq;    
    if(bres3 >= 100000)     
    {
      bres3 -= 100000;
      note3_duty = note3speed;
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
    display.print(F("SOC:"));
    display.print(BaterrySOC);
    display.println(F("%"));


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
    display.print(F("Var: "));
    display.println(rotaryEncoder.readEncoder());
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

void rotary_onButtonClick()
{
	static unsigned long lastTimePressed = 0;
	//ignore multiple press in that time milliseconds
	if (millis() - lastTimePressed < 500)
	{
		return;
	}
	lastTimePressed = millis();
	Serial.print("button pressed at ");
	Serial.println(millis());
  rotaryEncoder.reset();
}

void rotary_loop()
{
	//dont print anything unless value changed
	if (!rotaryEncoder.encoderChanged())
	{
		return;
	}
  DutyBorder = rotaryEncoder.readEncoder();
	Serial.print("Value: ");
	Serial.println(rotaryEncoder.readEncoder());
}

int getSOC()
{
  float BaterryVoltage = 0;
  float SOC = 0;
  int analogSampler = 0;
  for (int x = 0; x<=2; x++)
  {
    analogSampler = analogSampler + analogRead(4);
    Serial.println("Leitura!");
    
  }
  analogSampler = analogSampler/3;
  BaterryVoltage = ((analogSampler * 3.3)/2048);
  Serial.println(BaterryVoltage);
  SOC = 1- ((4.2 - BaterryVoltage));
  SOC = 100 * SOC;
  Serial.println(SOC);
  return int(SOC);
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

  	//we must initialize rotary encoder
	rotaryEncoder.begin();

	rotaryEncoder.setup(
		[] { rotaryEncoder.readEncoder_ISR(); },
		[] { rotary_onButtonClick(); });

	//set boundaries and if values should cycle or not
	//in this example we will set possible values between 0 and 1000;
	bool circleValues = false;
	rotaryEncoder.setBoundaries(0, 9, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

	/*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
	//rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
	rotaryEncoder.setAcceleration(10); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  BaterrySOC = getSOC();
}

void loop() {
  rotary_loop();
  if ((millis()-TimeSinceDisplayUpdate) >= 100)
  {
    TimeSinceDisplayUpdate = millis();
    UpdateMIDIMode(false, true);
    SOCTimer++;
    if (SOCTimer>=100){
        SOCTimer = 0;
        BaterrySOC = getSOC();
    }
  }
}
