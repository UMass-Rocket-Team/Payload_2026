#ifndef RFM9X_DRIVER_H
#define RFM9X_DRIVER_H
#include <RH_RF95.h>

#include "kalman.h"


// initialize radio to freq = RFM95_FREQ txPower = POWER  Bw  = 125KHz  Cr = 4/5     SF = SF7 = 128  chips/symbol
bool rfm_init();  

// send data over radio
bool rfm_send(const uint8_t* data, size_t len);

// build packet as an array of bytes to be sent by the radio
size_t build_packet(uint8_t* buf,  float* imu, vehicle_state_t state, float temp);

#endif