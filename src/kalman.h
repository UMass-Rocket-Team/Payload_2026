#ifndef KALMAN_H
#define KALMAN_H

#define KALMAN_RATE 400 //Hz

struct vehicle_state_t{
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    float speed; // m/s
    float altitude; //m
};

// initialize kalman filter
void kalman_init();

// kalman filter predict step
void kalman_predict(float gx, float gy, float dt);

// kalman filter update step
void kalman_update(float ax, float ay, float az);

// returns vehicle_state_t  struct
vehicle_state_t kalman_get_state();

#endif