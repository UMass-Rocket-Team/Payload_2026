#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <Arduino.h>

#define LOG_PACKET_SIZE 32 //bytes


bool initFlash();

// writes packet into 512 bit buffer. Once buffer fills up flushes buffer
void writepacket(uint8_t* packet);

size_t build_log_packet(uint8_t* buf,  float* imu, vehicle_state_t state, float temp);

bool readFlashArray(uint32_t addr, uint8_t* buffer, uint16_t len);

#endif