#include "userDefs.h"
#include "globalVar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timerandinterrupts.h"
#include "ioSetup.h"
#include "userDisplay.h"
#include "userMIDI.h"

// Functions prototypes //
void setupTasks();

void app_main() {
    setupTimer();
    setupGPIO();
    setupSSD1306();
    bootScreen();
    startTimer();
}

static void taskBMS(void *pvParameters)
{
  for (;;) {
    vTaskDelay(bmsAquirePeriod / portTICK_RATE_MS);
  }
}

static void taskMIDI(void *pvParameters)
{
  uartmidi_init(uartmidi_receive_message_callback);
  // this UART is connected to TX pin 17, RX pin 16
  uartmidi_enable_port(0, 31250);  
  for (;;) {
    vTaskDelay(1 / portTICK_RATE_MS);
    uartmidi_tick();
  }
}

static void taskDisplay(void *pvParameters)
{
  for (;;) {
    vTaskDelay(displayPeriod / portTICK_RATE_MS);
  }
}

void setupTasks(){
    xTaskCreatePinnedToCore(&taskBMS, "BMSTask", 2048, NULL, 1, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&taskMIDI, "MIDITask", 2048, NULL, 1, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&taskDisplay, "DisplayTask", 2048, NULL, 1, NULL, tskNO_AFFINITY);
}
