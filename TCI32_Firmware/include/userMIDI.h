#ifndef USERMIDI_H
#define USERMIDI_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "uartmidi.h"
#include "userDefs.h"
#include "globalVar.h"
#include "esp_log.h"


void processReceivedMidiPacket(uint8_t *data, uint8_t size);
void uartmidi_receive_message_callback(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos);
int GetOnTime(int freq);
int PitchToFreq(uint8_t pitch);

#endif