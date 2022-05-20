#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include <math.h>

#include "ssd1306_hal/io.h"
#include "ssd1306.h"
#include "uartmidi.h"
#include "rotary_encoder.h"

#define TAG "TCI32"

rotary_encoder_t *encoder = NULL;

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (0.00001)   /*!< test interval for timer 0 */


static xQueueHandle gpio_button_evt_queue = NULL;
float pitchTable[] = {8.18, 8.66, 9.18, 9.72, 10.30, 10.91, 11.56, 12.25, 12.98, 13.75, 14.57, 15.43, 16.35, 17.32, 18.35, 19.45, 20.60, 21.83, 23.12, 24.50, 25.96, 27.50, 29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00, 58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.8, 110.0, 116.5, 123.5, 130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 196.0, 207.7, 220.0, 233.1, 246.9, 261.6, 277.2, 293.7, 311.1, 329.6, 349.2, 370.0, 392.0, 415.3, 440.0, 466.2, 493.9, 523.3, 554.4, 587.3, 622.3, 659.3, 698.5, 740.0, 784.0, 830.6, 880.0, 932.3, 987.8, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, 8372, 8869, 9397, 9956, 10548, 11175, 11840, 12544, 13290, 14080, 14917, 15804, 16744, 17740, 18795, 19912, 21096, 22350, 23679, 25088, 26579, 28160, 29834, 31608, 33505, 35428, 37480, 39680, 420};

SAppMenu menu;
const char *menuItems[] =
{
    "Main Menu",
    "MIDI Mode",
    "Fixed Mode",
    "Settings",
};

#define bresFreq 100000 // 1/timer_interval0_sec 
volatile int masterDuty = 0;

volatile float bres1 = 0;
volatile float note1Freq = 0;
volatile int note1Note = 0;
volatile unsigned char note1Duty = 0;
volatile unsigned char note1DutyMaster = 0;
bool note1On = false;

volatile float bres2 = 0;
volatile float note2Freq = 0;
volatile int note2Note = 0;
volatile unsigned char note2Duty = 0;
volatile unsigned char note2DutyMaster = 0;
bool note2On = false;

// Essa func é chamada a cada 10us

void IRAM_ATTR timer_group0_isr(void *para){// timer group 0, ISR
  int timer_idx = (int) para;
  uint32_t intr_status = TIMERG0.int_st_timers.val;
  if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
      TIMERG0.hw_timer[timer_idx].update = 1;
      TIMERG0.int_clr_timers.t0 = 1;
      TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
      ////////// Bres /////////////
      // This is kinda advanced, whe are using the bresenham timing system to generete multiple notes, the bresFreq is 100kHz (10us)
      if(note1On)           
      {
        bres1 += note1Freq;    
        if(bres1 >= bresFreq)     
        {
          bres1 -= bresFreq;
          note1Duty = note1DutyMaster;
        }
      }
      if(note2On)           
      {
        bres2 += note2Freq;    
        if(bres2 >= bresFreq)     
        {
          bres2 -= bresFreq;
          note2Duty = note2DutyMaster;
        }
      }


      if(note1Duty || note2Duty)
      {
        if (note1Duty)
        {
          note1Duty--;
        }
        if (note2Duty)
        {
          note2Duty--;
        }
        gpio_set_level(GPIO_NUM_2, 1);
      }
      else
      {
        gpio_set_level(GPIO_NUM_2, 0);
      } 

  }
}

static void IRAM_ATTR gpio_button_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_button_evt_queue, &gpio_num, NULL);
}

int GetOnTime(int freq)
{
	int on_time = 1;
	if (freq < 700)  {on_time = 1;}
	if (freq < 600)  {on_time = 1;}
	if (freq < 500)  {on_time = 2;}
	if (freq < 400)  {on_time = 2;}
	if (freq < 300)  {on_time = 3;}
	if (freq < 200)  {on_time = 3;}
	if (freq < 100)  {on_time = 3;}
	on_time = on_time * masterDuty;
	return on_time;
}

float PitchToFreq(uint8_t pitch)
{	
  float freq = pitchTable[pitch];
  if (freq>700)
  {
    return 700;
  }
	return freq;
}

void processReceivedMidiPacket(uint8_t *data, uint8_t size)
{
    printf("Received data : ");
    for(uint8_t i=0; i<size; i++)
        printf("%x ", data[i]);
    printf(" - %d ", data[0]);
    printf("-");
    uint8_t *ptr = &data[1];
    uint8_t note = ptr[0];
    uint8_t velocity = ptr[1];
    switch (data[0])
    {
    case 128: ;
      printf(" - Note Off, note %d, velocity %d\n", note, velocity);
      if (note == note1Note){
        note1On = false;
      }
      if (note == note2Note){
        note2On = false;
      }
      break;
    case 144: ;
      if (velocity != 0)
      {
        printf(" - Note On, note %d, velocity %d\n", note, velocity);
        if (note1On == false)
        {
          note1On = true;
          note1Note = note;
          note1Freq = PitchToFreq(note);
          note1DutyMaster = GetOnTime(note1Freq);
        }
        else if (note2On == false)
        {
          note2On = true;
          note2Note = note;
          note2Freq = PitchToFreq(note);
          note2DutyMaster = GetOnTime(note2Freq);
        }
      }
      else
      {
        printf(" - Note Off, note %d, velocity %d\n", note, velocity);
        if (note == note1Note)
        {
          note1On = false;
        }
        if (note == note2Note)
        {
          note2On = false;
        }
      }
      break;
    
    default:
      break;
    }
}

void uartmidi_receive_message_callback(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  printf("Received data : ");
    for(uint8_t i=0; i<len+1; i++)
        printf("%x ", remaining_message[i]);
  printf("-");
  // enable to print out debug messages
  // ESP_LOGI(TAG, "receive_message CALLBACK uartmidi_port=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", uartmidi_port, midi_status, len, continued_sysex_pos);
  // esp_log_buffer_hex(TAG, remaining_message, len);
  //printf("Status recebido: %d | Len: %d\n", midi_status, len);
  {
    // TODO: more comfortable packet creation via special APIs

    // Note: by intention we create new packets for each incoming message
    // this shows that running status is maintained, and that SysEx streams work as well

    if( midi_status == 0xf0 && continued_sysex_pos > 0 ) {
      processReceivedMidiPacket(remaining_message, len);
    } else {
      size_t loopback_packet_len = 1 + len; // includes MIDI status and remaining bytes
      uint8_t *loopback_packet = (uint8_t *)malloc(loopback_packet_len * sizeof(uint8_t));
      if( loopback_packet == NULL ) {
        // no memory...
      } else {
        loopback_packet[0] = midi_status;
        memcpy(&loopback_packet[1], remaining_message, len);
        processReceivedMidiPacket(loopback_packet, loopback_packet_len);

        free(loopback_packet);
      }
    }
  }
  
  
}

void setupGPIO() {
    // This funcition confiure the GPIO 
    gpio_config_t io_conf;

    //----------------------------------//
    // Enable GPIO2 as OUTPUT
    // This is the GPIO pin that is connected to the LED on the ESP32 DEV Board
    // This is also the pin used to control the Tesla Coil Driver
    // Disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Selects GPIO 2
    io_conf.pin_bit_mask = (1ULL<<02);
    // Disable pull-down mode
    io_conf.pull_down_en = 0;
    // Disable pull-up mode
    io_conf.pull_up_en = 0;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);
    // Makes sure that GPIO2 is on Low (important to not fire the TC)
    gpio_set_level(GPIO_NUM_2, 0);
    //----------------------------------// 

    //----------------------------------//
    // Enable GPI32 as INPUT
    // Interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // Set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<32);
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    // Configure GPIO with the given settings
    gpio_config(&io_conf); 
    //----------------------------------// 


}

void setupSSD1306(){
    ssd1306_platform_i2cConfig_t cfg = {
        .sda = 21,
        .scl = 22
    };
    ssd1306_platform_i2cInit(I2C_NUM_0, 0, &cfg);

    ssd1306_128x64_i2c_init();

    ssd1306_clearScreen();
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
    ssd1306_createMenu( &menu, menuItems, sizeof(menuItems) / sizeof(char *) );
    ssd1306_menuDown( &menu );
    //ssd1306_showMenu( &menu );
}

void setupEncoder(){
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 33, 25);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
}

void setupInterrupt(){
  //create a queue to handle gpio event from isr
  gpio_button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  //install gpio isr service
  gpio_install_isr_service(0);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_NUM_32, gpio_button_isr_handler, (void*) GPIO_NUM_32);
}

void setupTimer(){
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}
///////////////////////////////////////////////////////////////////////////////

static void task_midi(void *pvParameters)
{
  uartmidi_init(uartmidi_receive_message_callback);
  // we use uartmidi_port 0 with standard baudrate in this demo
  // this UART is connected to TX pin 17, RX pin 16
  // see also components/uartmidi/uartmidi.c
  uartmidi_enable_port(0, 31250);
  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);

    uartmidi_tick();
  }
}

static void task_bms(void *pvParameters)
{
  for (;;) {
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

static void task_mainOS(void *pvParameters)
{
    uint8_t OS_State = 0;
    uint32_t gpio_button_flag; 
    char displayBuffer[256];
    bool button_flag = false;
    int encoderLastValue = encoder->get_counter_value(encoder);
    for (;;) {
        if(xQueueReceive(gpio_button_evt_queue, &gpio_button_flag, 0)) {
            xQueueReset(gpio_button_evt_queue);
            button_flag = true;
        }
        /*
        0 = Boot
        1 = Menu
        2 = MIDI
        3 = Fixed
        4 = Settings
        */
        switch (OS_State) {
        case 0:
            // Boot
            ssd1306_print("TCI 32");
            vTaskDelay(3000 / portTICK_RATE_MS);
            OS_State = 1;
            ssd1306_clearScreen();
            ssd1306_showMenu( &menu );   
            break;
        case 1:
            // Menu
            if (encoder->get_counter_value(encoder) > encoderLastValue) {
                ssd1306_menuUp( &menu );
                if (ssd1306_menuSelection(&menu) == 0){
                    ssd1306_menuUp( &menu );
                }
                encoderLastValue = encoder->get_counter_value(encoder);
            } else if (encoder->get_counter_value(encoder) < encoderLastValue) {
                if (ssd1306_menuSelection(&menu) == 3){
                    ssd1306_menuDown( &menu );
                }
                ssd1306_menuDown( &menu );
                encoderLastValue = encoder->get_counter_value(encoder);
            }
            ssd1306_showMenu( &menu ); 
            if (button_flag){
              OS_State = ssd1306_menuSelection(&menu) + 1;
              button_flag = false;
              ssd1306_clearScreen();
            }
            break;
        case 2:
            // MIDI
            if (encoder->get_counter_value(encoder) > encoderLastValue) {
              masterDuty += -1*((encoder->get_counter_value(encoder) - encoderLastValue))/2;
              if (masterDuty<0){
                masterDuty = 0;
              }
              encoderLastValue = encoder->get_counter_value(encoder);
            } else if (encoder->get_counter_value(encoder) < encoderLastValue) {
              masterDuty += -1*((encoder->get_counter_value(encoder) - encoderLastValue))/2;
              if (masterDuty>10){
                masterDuty = 10;
              }
              encoderLastValue = encoder->get_counter_value(encoder);
            }
            ssd1306_setCursor(0, 0);
            ssd1306_print("MIDI Mode");
            ssd1306_setCursor(0, 8);
            ssd1306_print("SOC: ");
            ssd1306_drawRect(0, 16, 128 - 4, 64 - 4); 
            ssd1306_setCursor(24, 24);
            ssd1306_print("Note1: ");
            if (note1On){
              sprintf(displayBuffer, "%f", note1Freq);
            }
            else{
              sprintf(displayBuffer, "%d", 0);
            }
            ssd1306_print(displayBuffer);
            ssd1306_print("  ");
            ssd1306_setCursor(86, 24);
            ssd1306_print("Hz");
            ssd1306_setCursor(24, 32);
            ssd1306_print("Note2: ");
            if (note2On){
              sprintf(displayBuffer, "%f", note2Freq);
            }
            else{
              sprintf(displayBuffer, "%d", 0);
            }
            ssd1306_print(displayBuffer);
            ssd1306_print("  ");
            ssd1306_setCursor(86, 32);
            ssd1306_print("Hz");
            ssd1306_setCursor(24, 40);
            ssd1306_print("Power: ");
            sprintf(displayBuffer, "%d", masterDuty * 10);
            ssd1306_print(displayBuffer);
            ssd1306_print("  ");
            ssd1306_setCursor(86, 40);
            ssd1306_print("%");
            if (button_flag){
              OS_State = 1;
              button_flag = false;
              ssd1306_clearScreen();
            }
            break;
        case 3:
            // Fixed
            note1On = true;
            note1DutyMaster = 10;
            if (encoder->get_counter_value(encoder) > encoderLastValue) {
              note1Freq += ((encoder->get_counter_value(encoder) - encoderLastValue) * -5);
              if (note1Freq<0){
                note1Freq = 0;
              }
              encoderLastValue = encoder->get_counter_value(encoder);
            } else if (encoder->get_counter_value(encoder) < encoderLastValue) {
              note1Freq += ((encoder->get_counter_value(encoder) - encoderLastValue) * -5);
              if (note1Freq>700){
                note1Freq = 700;
              }
              encoderLastValue = encoder->get_counter_value(encoder);
            }
            ssd1306_setCursor(0, 0);
            ssd1306_print("Fixed Mode");
            ssd1306_setCursor(0, 8);
            ssd1306_print("SOC: ");
            ssd1306_drawRect(0, 16, 128 - 4, 64 - 4); 
            ssd1306_setCursor(24, 24);
            ssd1306_print("Freq: ");
            sprintf(displayBuffer, "%f", note1Freq);
            ssd1306_print(displayBuffer);
            ssd1306_print("  ");
            ssd1306_setCursor(80, 24);
            ssd1306_print("Hz");
            ssd1306_setCursor(24, 40);
            ssd1306_print("Power: ");
            sprintf(displayBuffer, "%d", 100);
            ssd1306_print(displayBuffer);
            ssd1306_print(" %");

            if (button_flag){
              note1On = false;
              OS_State = 1;
              button_flag = false;
              ssd1306_clearScreen();
            }
            break;
        case 4:
            // Settings
            ssd1306_clearScreen();
            ssd1306_print("SETTINGS MODE");
            if (button_flag){
              OS_State = 1;
              button_flag = false;
              ssd1306_clearScreen();
            }
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}



///////////////////////////////////////////////////////////////////////////////

void app_main()
{
    setupInterrupt();
    setupGPIO();
    setupSSD1306();
    setupEncoder();
    setupTimer();
    xTaskCreate(task_midi, "task_midi"  , 4096, NULL, 4, NULL);
    xTaskCreate(task_bms, "task_bms"  , 4096, NULL, 3, NULL);
    xTaskCreate(task_mainOS, "task_mainOS", 4096, NULL, 8, NULL);
    esp_log_level_set(TAG, ESP_LOG_WARN);
}
