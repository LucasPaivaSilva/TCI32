#ifndef USERDEFS_H
#define USERDEFS_H

// General defines and related stuff //
#define TAG_TCI32 "TCI32"
#define TAG_MIDI "MIDI"
#define TAG_SYS "SYS"
#define TAG_DISPLAY "DISPLAY"

// Pin defines // 
#define fiberTransmitterPIN 2
#define i2cSDAPIN 21
#define i2cSCLPIN 22

// Display defines //
#define displayHZ 20
// Do not change the values bellow unless you know what you are doing
#define I2C_NUM_0 (0)
#define displayPeriod (1000/displayHZ)

// BMS defines //
#define bmsAquireRateHZ 1
#define bmsAquirePeriod (1000/bmsAquireRateHZ)

#endif   