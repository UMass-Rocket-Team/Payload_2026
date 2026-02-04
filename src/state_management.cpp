#include "state_management.h"
#include <stdint.h>
#include "scheduler.h"

#define TICKS_REQUIRED 5
#define ACCEL_THRESHOLD 20 // m/s^2
#define SPEED_THRESHOLD 5 // m/s
#define APOGEE_SPEED_THRESHOLD -0.5f //m/s
#define LANDING_SPEED_THRESHOLD   0.3f   // m/s
#define LANDING_ALT_DELTA         0.2f   // meters per tick
#define LANDING_TICKS_REQUIRED   40





void state_machine_update(vehicle_state_t vehicle_state, float az)
{
    static float last_altitude = 0;
    static uint32_t landed_timer = 0;

    switch (flight_state) {

        // standby: sensors and kalman running
    case STANDBY:
        scheduler_enable(TASK_TELEM, false);
        scheduler_enable(TASK_LOG, false);
        if (launch_detected(vehicle_state, az)) {
            flight_state = ASCENT;
            scheduler_enable(TASK_TELEM, true);
            scheduler_enable(TASK_LOG, true);
        }
        break;

        //ascent: sensors, kalman, telemetry, logging
    case ASCENT:
        if (apogee_detected(vehicle_state)) {
            flight_state = DESCENT;
        }
        break;


        //descent: sensors, kalman, telemetry, logging
    case DESCENT:
        if (landing_detected(vehicle_state)) {
            flight_state = LANDED;
            scheduler_enable(TASK_LOG, false);
            scheduler_enable(TASK_BARO, false);
            scheduler_enable(TASK_IMU, false);
            scheduler_enable(TASK_KF, false);
        }
        break;
        //nothing
    case LANDED:
        // final state
        break;
    }
}




bool launch_detected(vehicle_state_t state, float az){
    static uint8_t tick_count = 0;
    static float last_alt = 0;

    if(az >= ACCEL_THRESHOLD && (state.speed >= SPEED_THRESHOLD || state.altitude > last_alt)) tick_count++;
    else tick_count = 0;

    last_alt = state.altitude;

    return tick_count > TICKS_REQUIRED;
}

bool apogee_detected(vehicle_state_t state){
    static uint8_t count  = 0; 
    static float last_alt = 0;
    if(state.speed <=  APOGEE_SPEED_THRESHOLD && state.altitude < last_alt) count++;
    else count = 0;

    last_alt = state.altitude;

    return count > TICKS_REQUIRED;

}

bool landing_detected(vehicle_state_t state){
    static uint8_t count  = 0; 
    static float last_alt = 0;

    if(fabsf(state.speed) < LANDING_SPEED_THRESHOLD && fabsf(state.altitude - last_alt) < LANDING_ALT_DELTA) count++;
    last_alt = state.altitude;
    
    return count >= LANDING_TICKS_REQUIRED;
}
