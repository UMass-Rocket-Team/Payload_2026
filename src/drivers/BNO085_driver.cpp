#include "BNO085_driver.h"
#include "Adafruit_BNO08x.h"
#include <Arduino.h>

#define IMU_STALE_THRESHOLD_US 5000

Adafruit_BNO08x bno1;
Adafruit_BNO08x bno2;



typedef struct {
    imu_measurements_t imu1;
    imu_measurements_t imu2;
} imu_driver_state_t;

imu_driver_state_t imu_state;

// interrupt
volatile bool imu1_data_ready = false;
volatile bool imu2_data_ready = false;
void imu1_isr() { imu1_data_ready = true; }
void imu2_isr() { imu2_data_ready = true; }


bool imu_init() {

    if (!bno1.begin_SPI(bno_cs1, bno1_int)) return false;
    if (!bno2.begin_SPI(bno_cs2, bno2_int)) return false;

    // Enable reports 
   
    bno1.enableReport(SH2_ACCELEROMETER, interval);
    bno1.enableReport(SH2_GYROSCOPE_CALIBRATED, interval);

    bno2.enableReport(SH2_ACCELEROMETER, interval);
    bno2.enableReport(SH2_GYROSCOPE_CALIBRATED, interval);

    pinMode(bno1_int, INPUT);
    pinMode(bno2_int, INPUT);

    attachInterrupt(digitalPinToInterrupt(bno1_int), imu1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(bno2_int), imu2_isr, RISING);

    memset(&imu_state, 0, sizeof(imu_state));

    return true;


}

bool drain_bno(Adafruit_BNO08x& bno, imu_measurements_t* data){
    bool got_any = false;
    sh2_SensorValue_t ev;
    uint32_t now;

    while (bno.getSensorEvent(&ev)) {
        now = time_us_32();

        if (ev.sensorId == SH2_ACCELEROMETER) {
            data->accel.x = ev.un.accelerometer.x;
            data->accel.y = ev.un.accelerometer.y;
            data->accel.z = ev.un.accelerometer.z;
            data->accel.t_us = now;
            data->accel.valid = true;
        }
        else if (ev.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            data->gyro.x = ev.un.gyroscope.x;
            data->gyro.y = ev.un.gyroscope.y;
            data->gyro.z = ev.un.gyroscope.z;
            data->gyro.t_us = now;
            data->gyro.valid = true;

        }
        got_any = true;
    }

    return got_any;
}

bool imu_poll(void){
    bool got_any = false;

    if (imu1_data_ready) {
        imu1_data_ready = false;
        while(drain_bno(bno1, &imu_state.imu1)) got_any = true;
    }
    if (imu2_data_ready) {
        imu2_data_ready = false;
        while(drain_bno(bno2, &imu_state.imu2)) got_any = true;
    }

    return got_any;
}

bool get_accel(float* ax, float* ay, float* az){
    uint32_t now = time_us_32();
    
    // Check if data exists AND if it was received recently
    bool imu1_fresh = imu_state.imu1.accel.valid && (now - imu_state.imu1.accel.t_us < IMU_STALE_THRESHOLD_US);
    bool imu2_fresh = imu_state.imu2.accel.valid && (now - imu_state.imu2.accel.t_us < IMU_STALE_THRESHOLD_US);

    if (!imu1_fresh && !imu2_fresh) return false;

    if (imu1_fresh && imu2_fresh) {
        // Both are fresh: Average them (Best Case)
        *ax = 0.5f * (imu_state.imu1.accel.x + imu_state.imu2.accel.x);
        *ay = 0.5f * (imu_state.imu1.accel.y + imu_state.imu2.accel.y);
        *az = 0.5f * (imu_state.imu1.accel.z + imu_state.imu2.accel.z);
    } 
    else if (imu1_fresh) {
        // Only IMU1 is fresh: Use it alone
        *ax = imu_state.imu1.accel.x;
        *ay = imu_state.imu1.accel.y;
        *az = imu_state.imu1.accel.z;
    } 
    else {
        // Only IMU2 is fresh: Use it alone
        *ax = imu_state.imu2.accel.x;
        *ay = imu_state.imu2.accel.y;
        *az = imu_state.imu2.accel.z;
    }
    return true;
}

bool get_gyro(float* gx, float* gy, float* gz){
    uint32_t now = time_us_32();
    
    // Check if data exists AND if it was received recently
    bool imu1_fresh = imu_state.imu1.gyro.valid && (now - imu_state.imu1.gyro.t_us < IMU_STALE_THRESHOLD_US);
    bool imu2_fresh = imu_state.imu2.gyro.valid && (now - imu_state.imu2.gyro.t_us < IMU_STALE_THRESHOLD_US);

    if (!imu1_fresh && !imu2_fresh) return false;

    if (imu1_fresh && imu2_fresh) {
        // Both are fresh: Average them (Best Case)
        *gx = 0.5f * (imu_state.imu1.gyro.x + imu_state.imu2.gyro.x);
        *gy = 0.5f * (imu_state.imu1.gyro.y + imu_state.imu2.gyro.y);
        *gz = 0.5f * (imu_state.imu1.gyro.z + imu_state.imu2.gyro.z);
    } 
    else if (imu1_fresh) {
        // Only IMU1 is fresh: Use it alone
        *gx = imu_state.imu1.gyro.x;
        *gy = imu_state.imu1.gyro.y;
        *gz = imu_state.imu1.gyro.z;
    } 
    else {
        // Only IMU2 is fresh: Use it alone
        *gx = imu_state.imu2.gyro.x;
        *gy = imu_state.imu2.gyro.y;
        *gz = imu_state.imu2.gyro.z;
    }
    return true;
}

