#include "kalman.h"
#include <ArduinoEigen.h>

using namespace Eigen;

//state vector: qw, qx, qy, qz, speed, altitude, xbias, ybias, zbias
Matrix<float, 9, 1> x = Matrix<float, 9, 1>::Zero(); //zero for now

// initialize kalman filter
void kalman_init(){

}

// kalman filter predict step
void kalman_predict(float gx, float gy, float dt){

}

// kalman filter update step
void kalman_update(float ax, float ay, float az){

}

// returns vehicle_state_t  struct
vehicle_state_t kalman_get_state(){
    vehicle_state_t state = {x(0), x(1), x(2), x(3), x(4), x(5)};
    return state;
} 