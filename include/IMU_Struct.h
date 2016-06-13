//---------------------------
// Timothee Habra
//
// Creation : 28-4-2015
//---------------------------

#ifndef IMU_STRUCT_h
#define IMU_STRUCT_h

#include "mbs_data.h"
#include "mbs_sensor.h"

typedef struct IMU_Struct
{
    MbsSensor* robotran_sensor;
    int sensor_id;
    double Orientation[9];
    double Angular_Rate[3];
    double Acceleration[3];
} IMU_Struct;

// ---- Init and free functions: declarations ---- //
IMU_Struct* init_IMU_Struct(void);
void free_IMU_Struct(IMU_Struct* Imu);

#endif
