#ifndef PIDs_Struct_h
#define PIDs_Struct_h

#include <stdlib.h>

#include "robotran_ActuatorsDefinitions.h"

// Defining PID struct (Joint Specific)
/* PID structures used with/without YARP in Simulation*/
typedef struct ControllerPIDs
{
    double p[COMAN_NB_JOINT_ACTUATED+1];
    double i[COMAN_NB_JOINT_ACTUATED+1];
    double d[COMAN_NB_JOINT_ACTUATED+1];
    double maxInt[COMAN_NB_JOINT_ACTUATED+1];
    double maxOut[COMAN_NB_JOINT_ACTUATED+1];
    double int_err[COMAN_NB_JOINT_ACTUATED+1];  //integral error
} ControllerPIDs ;

ControllerPIDs* init_ControllerPIDs(void);
void free_ControllerPIDs(ControllerPIDs *PIDs);

/*--------------------*/
#endif
