#include "RFM9x_driver.h"
#include <RH_RF95.h>
#include <cstring>


#define RFM95_CS 7 // CS/SS pin
#define RFM95_INT 6   // MSP interrupt pin (MSP catches interrupts from RFM and forwards them to pico)
#define RFM95_FREQ  433.0  // MHz
#define POWER 20 // dbm


#define FLAG_LAUNCHED   (1 << 0)
#define FLAG_LANDED     (1 << 1)
#define FLAG_APOGEE     (1 << 2)


RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint16_t counter = 0;

bool rfm_init()
{
    if (!rf95.init()) {  //Bw  = 125KHz  Cr = 4/5     SF = SF7 = 128  chips/symbol
        return false;
    }

    if (!rf95.setFrequency(RFM95_FREQ)) {
        return false;
    }

    rf95.setTxPower(POWER, false);

    return true;
}

bool rfm_send(const uint8_t* data, size_t len)
{
    if (len == 0 || len > RH_RF95_MAX_MESSAGE_LEN) {
        return false;
    }

    if (!rf95.send(data, len)) {
        return false;
    }

    return true;
}

inline int16_t clamp(float value) {
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return -32768;
    return (int16_t)value;
}

size_t build_packet(uint8_t* buf,  float* imu, vehicle_state_t state, float temp){
    uint8_t* p = buf;

    uint16_t ctr = counter++;  // incrementing packet counter  
    uint32_t timestamp = time_us_32();// elapsed time since boot
    
    int16_t  accel_x = clamp(imu[3]*400);  // m/s^2 scaled by 400 -> [-81.920. 81.9175] m/s^2
    int16_t  accel_y = clamp(imu[4]*400);  // m/s^2 scaled by 400
    int16_t  accel_z = clamp(imu[5]*400);  // m/s^@ scaled by 400 
    int16_t  gyro_x = clamp(imu[0]*900);  // rad/s scaled by 900  -> [-36.4089, 36.4078]
    int16_t  gyro_y = clamp(imu[1]*900);  // rad/s scaled by 900
    int16_t  gyro_z = clamp(imu[2]*900);  // rad/s scaled by 900
    
   uint32_t altitude = uint32_t(state.altitude *1000); // mm

   int16_t speed = (int16_t)(state.speed*100); // m/s scaled by 100 [-327.68, 327.67]
 
    
   //maybe switch to smalled three compression if needed - pack index bits into altitude MSBs
    int16_t q_w = clamp(state.q_w*32767); // quaternion w scaled by 32767 [-1, 1] -> [-32767, 32767]   maybe change to 16384
    int16_t q_x = clamp(state.q_x*32767); // quaternion x scaled by 32767 
    int16_t q_y = clamp(state.q_y*32767); // quaternion y scaled by 32767
    int16_t q_z = clamp(state.q_z*32767); // quaternion z scaled by 32767

    int8_t temperature = (int8_t)(temp*2); // temp in C scaled by 2 [-64.0, 63.5]

    int8_t flags = 0;//TODO add

    #define PUSH(v) do { memcpy(p, &(v), sizeof(v)); p += sizeof(v); } while(0)

    PUSH(ctr);
    PUSH(timestamp);
    PUSH(accel_x);
    PUSH(accel_y);
    PUSH(accel_z);
    PUSH(gyro_x);
    PUSH(gyro_y);
    PUSH(gyro_z);
    PUSH(altitude);
    PUSH(speed);
    PUSH(q_w);
    PUSH(q_x);
    PUSH(q_y);
    PUSH(q_z);
    PUSH(temperature);
    PUSH(flags);

    #undef PUSH

    return p - buf;
}
