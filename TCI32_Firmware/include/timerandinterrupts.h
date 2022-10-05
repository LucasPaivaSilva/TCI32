#ifndef TIMERANDINTERRUPTS_H
#define TIMERANDINTERRUPTS_H

#include "userDefs.h"
#include "globalVar.h"
#include "driver/timer.h"
#include "driver/gpio.h"

// Timer defines
#define TIMER_INTR_SEL TIMER_INTR_LEVEL                                 /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0                                    /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80                                              /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)                 /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000)   /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (0.00001)                                 /*!< test interval for timer 0 */

#define bresFreq 100000 // 1/timer_interval0_sec 

void setupTimer();

void startTimer();

#endif