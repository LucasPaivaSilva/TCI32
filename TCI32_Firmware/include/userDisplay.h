#ifndef USERDISPLAY_H
#define USERDISPLAY_H
#include "ssd1306_hal/io.h"
#include "ssd1306.h"
#include "userDefs.h"

#define I2C_NUM_0              (0) /*!< I2C port 0 */

void setupSSD1306();
void bootScreen();
void drawMIDIScreen();
void drawMIDIScreenData(int freq1, int freq2, int powerPercent);
void drawMIDIScreenBat(int batPercent);


#endif