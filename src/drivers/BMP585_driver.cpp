#include "BMP585_driver.h"
#include <Adafruit_BMP5xx.h>

#define bmp_cs 18
#define bmp_int 14


static Adafruit_BMP5xx bmp;
static bool bmp_ready = false;


bool bmp585_init(){

    if (!bmp.begin(bmp_cs, &SPI)) {
        bmp_ready = false;
        return false;
    }

    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    bmp.setOutputDataRate(BMP5XX_ODR_240_HZ);  //set to 240 Hz
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    bmp_ready = true;
    return true;
}





bool bmp585_read(float *temp_c, float *pressure_hpa) {
    if (!bmp_ready) return false;

    if (!bmp.performReading()) return false;

    if (temp_c)     *temp_c     = bmp.temperature;
    if (pressure_hpa) *pressure_hpa = bmp.pressure;

    return true;
}
