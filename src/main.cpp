#include "SPI.h"
#include "scheduler.h"

#define SPI_SCK   14
#define SPI_MISO  16
#define SPI_MOSI  15


/*
STANDBY: Sensors + Kalman running, no logging or telemetry. detect launch -> Ascent
ASCENT: Sensors + Kalman + logging + telemetry. detect apogee -> descent
DESCENT: Sensors + Kalman + logging + telemetry.  detect landing  -> landed
Landed: radio transmit to help location of rocket
*/


int32_t last_tx_us = 0;


void setup(){

    
    SPI.setSCK(SPI_SCK);
    SPI.setRX(SPI_MISO);
    SPI.setTX(SPI_MOSI);
    SPI.begin();

    while(!imu_init()) Serial.println("IMU initialization failed, trying again");
    while(!rfm_init()) Serial.println("RFM initialization failed, trying again");
    kalman_init();
    while(!bmp585_init()) Serial.println("BMP585 initialization failed, trying again");
    while(!initFlash()) Serial.println("Flash initialization failed, trying again");
    initFlash();

}

void loop(){
    scheduler_run();

}



