//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : Sat Nov  7 16:13:17 2015
//---------------------------

#ifndef ControllerStruct_h
#define ControllerStruct_h

#include "ControllerIO_Def.h"

// ---- Structures definitions (typedef) ---- //

// ControllerInputsStruc
typedef struct ControllerInputs
{
    double t;			// time [s]
    double q[31+1];     // link position [rad]
    double qd[31+1];    // link velocity [rad/s]
    double Qq[31+1];    // link torque [Nm]
    double q_mot[31+1]; // motor position [rad]
    double qd_mot[31+1];// motor velocity [rad/s]
    double F_Rfoot[3];
    double F_Lfoot[3];
    double T_Rfoot[3];
    double T_Lfoot[3];
    struct FT_sensor_Struct *FT_sensor_list[2];
    struct IMU_Struct *IMU_list[2];

} ControllerInputs;


// ControllerOutputsStruc
typedef struct ControllerOutputs
{
    double q_ref[31+1];  // joint reference position [rad]
    double qd_ref[31+1];
    double Qq_ref[31+1]; // joint reference torque [Nm]
    int servo_type[31+1]; // servo mode (0 = position control , 1 = torque control)

} ControllerOutputs;


// ControllerStructStruc
typedef struct ControllerStruct
{
    struct ControllerInputs *Inputs;
    struct ControllerOutputs *Outputs;
    double out[20];
    double q_ref_r_sh_sag;
    double q_ref_r_sh_lat;
    double q_ref_r_sh_yaw;
    double q_ref_r_elb;
    double q_ref_l_sh_sag;
    double q_ref_l_sh_lat;
    double q_ref_l_sh_yaw;
    double q_ref_l_elb;

} ControllerStruct;


// ---- Init and free functions: declarations ---- //

ControllerInputs * init_ControllerInputs(void);
void free_ControllerInputs(ControllerInputs *cvs);

ControllerOutputs * init_ControllerOutputs(void);
void free_ControllerOutputs(ControllerOutputs *cvs);

ControllerStruct * init_ControllerStruct(void);
void free_ControllerStruct(ControllerStruct *cvs);

/*--------------------*/
#endif

