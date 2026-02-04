#ifndef BNO085_DRIVER_H
#define BNO085_DRIVER_H



#define bno_cs1 19 //gpio 19 
#define bno1_int 9 //gpio 9
#define bno2_int 13 //gpio 13
#define bno_cs2 20 //gpio 20

#define update_rate 400 //hz
int interval = 1000000/update_rate; // us

typedef struct {
    float x;
    float y;
    float z;
    uint32_t t_us;
    bool valid;
} vec3_sample_t;

typedef struct {
    vec3_sample_t accel;
    vec3_sample_t gyro;
} imu_measurements_t;


// initiaze IMUs
bool imu_init();

//poll imu for updates
bool imu_poll(void);

//get accel data
bool get_accel(float* ax, float* ay, float* az);

// get gyro data  
bool get_gyro(float* gx, float* gy, float* gz);

#endif