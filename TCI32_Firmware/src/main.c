#include "userDefs.h"
#include "globalVar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timerandinterrupts.h"
#include "ioSetup.h"
#include "userDisplay.h"

void app_main() {
    setupTimer();
    setupGPIO();
    setupSSD1306();
    bootScreen();

    while (1) {
        
    }
}
