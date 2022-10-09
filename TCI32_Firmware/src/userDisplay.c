#include "userDisplay.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void setupSSD1306(){
    ssd1306_platform_i2cConfig_t cfg = {
        .sda = i2cSDAPIN,
        .scl = i2cSCLPIN
    };
    ssd1306_platform_i2cInit(I2C_NUM_0, 0, &cfg);

    ssd1306_128x64_i2c_init();

    ssd1306_clearScreen();
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
}

void bootScreen(){
    ssd1306_setFixedFont(ssd1306xled_font8x16);
    ssd1306_print("TCI 32\n");
    ssd1306_print("TCI 32\n");
    ssd1306_print("TCI 32\n");
    vTaskDelay(3000 / portTICK_RATE_MS);
    //ssd1306_clearScreen(); 
}

void drawMIDIScreen(){
    ssd1306_clearScreen();
    ssd1306_print("MIDI");
    ssd1306_print("Channel: ");
    ssd1306_print("Note: ");
    ssd1306_print("Velocity: ");
    ssd1306_print("CC: ");
    ssd1306_print("Value: ");
}

void drawMIDIScreenData(int freq1, int freq2, int powerPercent){
    ssd1306_clearScreen();

}

void drawMIDIScreenBat(int batPercent){
    ssd1306_clearScreen();
    ssd1306_print("MIDI");
    ssd1306_print("Channel: ");
    ssd1306_print("Note: ");
    ssd1306_print("Velocity: ");
    ssd1306_print("CC: ");
    ssd1306_print("Value: ");
}