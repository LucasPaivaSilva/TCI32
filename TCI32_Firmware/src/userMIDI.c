#include "userMIDI.h"

int GetOnTime(int freq)
{
	return 2 * masterDuty;
}

int PitchToFreq(uint8_t pitch)
{	
  int freq = pitchTable[pitch];
  if (freq>700)
  {
    return 700;
  }
  else
  {
    return freq;
  }
}

void processReceivedMidiPacket(uint8_t *data, uint8_t size)
{
    // printf("Received data : ");
    // for(uint8_t i=0; i<size; i++)
    //     printf("%x ", data[i]);
    // printf(" - %d ", data[0]);
    // printf("-");
    ESP_LOGI(TAG_MIDI, "Recived Data - Mode: %d \n", data[0]);
    uint8_t *ptr = &data[1];
    uint8_t note = ptr[0];
    uint8_t velocity = ptr[1];
    switch (data[0])
    {
    case 128: ;
      // Note Off
      if (note == note1Note){
        note1On = 0;
      }
      if (note == note2Note){
        note2On = 0;
      }
      break;
    case 144: ;
      if (velocity != 0)
      {
        // Note On
        if (note1On == 0)
        {
          note1On = 1;
          note1Note = note;
          note1Freq = PitchToFreq(note);
          //note1DutyMaster = GetOnTime(note1Freq);
        }
        else if (note2On == 0)
        {
          note2On = 1;
          note2Note = note;
          note2Freq = PitchToFreq(note);
          //note2DutyMaster = GetOnTime(note2Freq);
        }
      }
      else
      {
        // Note Off
        if (note == note1Note)
        {
          note1On = 0;
        }
        if (note == note2Note)
        {
          note2On = 0;
        }
      }
      break;
    
    default:
      break;
    }
}

void uartmidi_receive_message_callback(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  ESP_LOGI(TAG_MIDI, "receive_message CALLBACK uartmidi_port=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d \n", uartmidi_port, midi_status, len, continued_sysex_pos);
  {
    if( midi_status == 0xf0 && continued_sysex_pos > 0 ) 
    {
      processReceivedMidiPacket(remaining_message, len);
    } 
    else 
    {
      size_t loopback_packet_len = 1 + len; // includes MIDI status and remaining bytes
      uint8_t *loopback_packet = (uint8_t *)malloc(loopback_packet_len * sizeof(uint8_t));
      if( loopback_packet == NULL ) 
      {
        // no memory...
      } else 
      {
        loopback_packet[0] = midi_status;
        memcpy(&loopback_packet[1], remaining_message, len);
        processReceivedMidiPacket(loopback_packet, loopback_packet_len);
        free(loopback_packet);
      }
    }
  }
  
  
}