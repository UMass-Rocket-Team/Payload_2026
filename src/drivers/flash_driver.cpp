#include "flash_driver.h"
#include <SPIMemory.h>
#include <SD.h>
#include <kalman.h>
#include "state_management.h"

#define FLASH_CS 22
#define SD_CS 21
#define FLASH_PAGE_SIZE 256 //bytes

SPIFlash flash(FLASH_CS);

static uint32_t currentFlashAddr = 0;
static uint8_t ramBuffer[FLASH_PAGE_SIZE];
static uint16_t bufferIndex = 0;

bool initFlash() {
    if(!flash.begin()) return false;
    return flash.eraseChip();
}

bool eraseFlashSector(uint32_t addr) {
    return flash.eraseSector(addr);
}
// writes LOG_PACKET_SIZE bit long packet into 512 byte buffer. Once buffer fills up flushes buffer
void writepacket(uint8_t* packet) {
    //check if next packet will over flow 512 byte buffer
    //if it fits, copy it in, if not write buffer to flash and then clear it
    if (bufferIndex + LOG_PACKET_SIZE > FLASH_PAGE_SIZE) flushBuffer();

    //copy packet to ram buffer
    memcpy(&ramBuffer[bufferIndex], packet, LOG_PACKET_SIZE);
    bufferIndex += LOG_PACKET_SIZE;

    if (bufferIndex == FLASH_PAGE_SIZE) flushBuffer();

}

void flushBuffer(){
    if(bufferIndex == 0) return; //nothing to write

    //pad remainder of 512  byte block with 0xFF
    if(bufferIndex < FLASH_PAGE_SIZE){
        memset(&ramBuffer[bufferIndex], 0xFF, FLASH_PAGE_SIZE - bufferIndex);
    }


    //write full buffer to flash
    if(flash.writeByteArray(currentFlashAddr, ramBuffer, FLASH_PAGE_SIZE)){
        currentFlashAddr +=  FLASH_PAGE_SIZE;
        bufferIndex = 0; //reset buffer
        memset(ramBuffer, 0xFF, FLASH_PAGE_SIZE);
    }
}

inline int16_t clamp(float value) {
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return -32768;
    return (int16_t)value;
    
}

size_t build_log_packet(uint8_t* buf ,  float* imu, vehicle_state_t state, float temp){
    uint8_t* p = buf;

    uint32_t timestamp = time_us_32(); // elapsed time since boot
    
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

    static int8_t flags = 0;
    // bit #0 = apogee detected
    if(flight_state = DESCENT) flags |= 1;
    //  bit #1 = landing detected
    if(flight_state = LANDED) flags |= (1<<1);
    //TODO maybe burnout, chute


    #define PUSH(v) do { memcpy(p, &(v), sizeof(v)); p += sizeof(v); } while(0)

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


bool readFlashArray(uint32_t addr, uint8_t* buffer, uint16_t len) {
    return flash.readByteArray(addr, buffer, len);
}

void dumpToSD(){
    //make sure no data is left in RAM
    flushBuffer();

    if(!SD.begin(SD_CS)){
        Serial.println("SD initialization failed!"); 
        return; 
    } 
    //open file
    File dataFile = SD.open("LOG.BIN",  O_CREAT | O_TRUNC | O_WRITE);
    if(!dataFile){
        Serial.println("Failed to open LOG.BIN on SD");
        return;
    }

    uint8_t transferBuffer[FLASH_PAGE_SIZE];
    uint32_t readAddr = 0;

    while(readAddr < currentFlashAddr){
        //read block from flash
        if(flash.readByteArray(readAddr, transferBuffer, FLASH_PAGE_SIZE)){
            //write block to SD
            dataFile.write(transferBuffer, FLASH_PAGE_SIZE);
            readAddr += FLASH_PAGE_SIZE;
        } else {
            Serial.println("Flash read error");
            break;
        }
    }
    
    dataFile.close();

}

