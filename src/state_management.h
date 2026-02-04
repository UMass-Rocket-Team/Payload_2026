#ifndef STATE_MANAGEMENT_H
#define STATE_MANAGEMENT_H
#include "kalman.h"

enum flight_state_t{
    STANDBY, 
    ASCENT,
    DESCENT,
    LANDED
};



flight_state_t flight_state = STANDBY;

void state_machine_update(vehicle_state_t vehicle_state, float az);

#endif