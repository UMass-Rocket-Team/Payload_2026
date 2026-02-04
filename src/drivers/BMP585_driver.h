#ifndef BMP585_DRIVER_H
#define BMP585_DRIVER_H
#include "Adafruit_BNO08x.h"


bool bmp585_init(void);


bool bmp585_read(float *temp_c, float *pressure_hpa);

#endif