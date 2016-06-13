//---------------------------
// Timothee Habra
//
// Creation : 28-4-2015
//---------------------------

#ifndef FT_SENSOR_STRUCT_h
#define FT_SENSOR_STRUCT_h

typedef struct FT_sensor_Struct
{
    int id_joint_F[3];  // id of T1,T2,T3 fixed joints
    int id_joint_T[3];  // id of R1,R2,R3 fixed joints
    double force[3]; // x,y,z force components [N]
    double torque[3]; // x,y,z torque components [Nm]
} FT_sensor_Struct;

// ---- Init and free functions: declarations ---- //
FT_sensor_Struct* init_FT_sensor_Struct(void);
void free_FT_sensor_Struct(FT_sensor_Struct* Imu);

#endif
