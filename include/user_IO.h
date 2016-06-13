/*===========================================================================*
  *
  *  user_sf_IO.h
  *	
  *  Project:	PendulumSpringC
  * 
  *  Generation date: 14-Nov-2014 18:28:15
  * 
  *  (c) Universite catholique de Louvain
  *      D�partement de M�canique 
  *      Unit� de Production M�canique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#include "mbs_user_interface.h"

//#include "userDef.h"
#include "ControllersStruct.h"

struct UserIO 
{
    double Voltage[31+1];
    double currents[31+1];
    double k_stiff[31+1];
    double k_damp[31+1];
    double last_t_ctrl;
    void *compute_gcm;
    void *contactGestion;
    struct ControllerPIDs *PIDs_pos;
    struct ControllerPIDs *PIDs_torque;
    struct ActuatorsStruct *actuatorsStruct;
    struct ControllerStruct *cvs;

};



/*--------------------*/
#endif
