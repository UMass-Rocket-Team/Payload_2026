#include "scheduler.h"

typedef struct {
    void (*run)(void);
    uint32_t period_us;
    uint32_t last_run;
    bool enabled;
} task_t;


task_t tasks[TASK_COUNT] = {
    [TASK_IMU]   = { imu_task,   1000000/KALMAN_RATE, 0, true },  
    [TASK_BARO]  = { baro_task, 1000000/240, 0, true },  
    [TASK_KF]    = { kf_task,    1000000/KALMAN_RATE, 0, true },  
    [TASK_STATE] = { state_task, 1000000/KALMAN_RATE, 0, true },  
    [TASK_LOG]   = { log_task,   5000, 0, true },  // 200 Hz
    [TASK_TELEM] = { telem_task,50000, 0, true },  // 20 Hz
};

vehicle_state_t vehicle_state;
float temp_c, pressure_hpa;
float imu_data[6];

void scheduler_run(void) {
    uint32_t now = time_us_32();

    for (int i = 0; i < TASK_COUNT; i++) {
        task_t* t = &tasks[i];
        if (!t->enabled) continue;

        if ((now - t->last_run) >= t->period_us) {
            t->last_run = now;
            t->run();
        }
    }
}


void scheduler_enable(task_id_t id, bool enable){
    tasks[id].enabled = enable;
}


void imu_task(){
    imu_poll();
}

void baro_task(){
    bmp585_read(&temp_c, &pressure_hpa);
}

void kf_task(){
    float dt = 0.025;
    get_gyro(&imu_data[0], &imu_data[1], &imu_data[2]);
    bool has_accel  = get_accel(&imu_data[3], &imu_data[4], &imu_data[5]);

    //always predict
    kalman_predict(imu_data[0], imu_data[1], dt);
    //dont  update if no new accel readings
    if(has_accel) kalman_update(imu_data[3], imu_data[4], imu_data[5]);

    vehicle_state = kalman_get_state();
}

void state_task(){
    state_machine_update(vehicle_state, imu_data[5]);
}

void log_task(){
    uint8_t buf[LOG_PACKET_SIZE];
    build_log_packet(buf, imu_data, vehicle_state, temp_c);
    writepacket(buf);
}

void telem_task(){
    uint8_t buf[34];
    size_t len = build_packet(buf, imu_data, vehicle_state, temp_c);
    rfm_send(buf, len);
}
