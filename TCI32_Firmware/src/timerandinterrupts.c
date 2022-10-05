#include "timerandinterrupts.h"

void IRAM_ATTR timer_group0_isr(void *para){// timer group 0, ISR
  // This function is called every 10us
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
        gpio_set_level(fiberTransmitterPIN, 1);
      }
      else
      {
        gpio_set_level(fiberTransmitterPIN, 0);
      } 

  }
}


void setupTimer(){
    int timer_group = TIMER_GROUP;
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
    timer_start(timer_group, timer_idx);
}

void startTimer(){
    int timer_group = TIMER_GROUP;
    int timer_idx = TIMER_0;
    timer_start(timer_group, timer_idx);
}