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

#include "ssd1306_hal/io.h"
#include "ssd1306.h"
#include "uartmidi.h"
#include "rotary_encoder.h"

#define TAG "TCI32"

rotary_encoder_t *encoder = NULL;

static xQueueHandle gpio_button_evt_queue = NULL;

SAppMenu menu;
const char *menuItems[] =
{
    "Main Menu",
    "MIDI Mode",
    "Fixed Mode",
    "Settings",
};

static void IRAM_ATTR gpio_button_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_button_evt_queue, &gpio_num, NULL);
}

void uartmidi_receive_message_callback(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  // enable to print out debug messages
  ESP_LOGI(TAG, "receive_message CALLBACK uartmidi_port=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", uartmidi_port, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(TAG, remaining_message, len);

  // loopback received message
  {
    // TODO: more comfortable packet creation via special APIs

    // Note: by intention we create new packets for each incoming message
    // this shows that running status is maintained, and that SysEx streams work as well

    if( midi_status == 0xf0 && continued_sysex_pos > 0 ) {
      uartmidi_send_message(0, remaining_message, len); // just forward
    } else {
      size_t loopback_packet_len = 1 + len; // includes MIDI status and remaining bytes
      uint8_t *loopback_packet = (uint8_t *)malloc(loopback_packet_len * sizeof(uint8_t));
      if( loopback_packet == NULL ) {
        // no memory...
      } else {
        loopback_packet[0] = midi_status;
        memcpy(&loopback_packet[1], remaining_message, len);

        uartmidi_send_message(uartmidi_port, loopback_packet, loopback_packet_len);

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
    int encoderLastValue = encoder->get_counter_value(encoder);
    for (;;) {
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
            if (encoder->get_counter_value(encoder) > encoderLastValue + 2) {
                ssd1306_menuUp( &menu );
                if (ssd1306_menuSelection(&menu) == 0){
                    ssd1306_menuUp( &menu );
                }
                encoderLastValue = encoder->get_counter_value(encoder);
            } else if (encoder->get_counter_value(encoder) < encoderLastValue - 2) {
                if (ssd1306_menuSelection(&menu) == 3){
                    ssd1306_menuDown( &menu );
                }
                ssd1306_menuDown( &menu );
                encoderLastValue = encoder->get_counter_value(encoder);
            }
            ssd1306_showMenu( &menu ); 
            // Menu
            break;
        case 2:
            // MIDI
            break;
        case 3:
            // Fixed
            break;
        case 4:
            // Settings
            break;
        default:
            break;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}



///////////////////////////////////////////////////////////////////////////////

void app_main()
{
    setupInterrupt();
    setupGPIO();
    setupSSD1306();
    setupEncoder();
    xTaskCreate(task_midi, "task_midi"  , 4096, NULL, 4, NULL);
    xTaskCreate(task_bms, "task_bms"  , 4096, NULL, 3, NULL);
    xTaskCreate(task_mainOS, "task_mainOS", 4096, NULL, 8, NULL);
    esp_log_level_set(TAG, ESP_LOG_WARN);
}
