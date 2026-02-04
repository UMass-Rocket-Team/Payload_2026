#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include "kalman.h"
#include "drivers/RFM9x_driver.h"
#include "drivers/BNO085_driver.h"
#include "drivers/BMP585_driver.h"
#include "drivers/flash_driver.h"
#include "state_management.h"




enum task_id_t {
    TASK_IMU,
    TASK_BARO,
    TASK_KF,
    TASK_LOG,
    TASK_TELEM,
    TASK_STATE,
    TASK_COUNT
};


void scheduler_run(void);

void scheduler_enable(task_id_t id, bool enable);

#endif