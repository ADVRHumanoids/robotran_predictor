
#ifndef ActuatorsDef_h
#define ActuatorsDef_h

// total number of actuated joints (= nb of motors)
#define COMAN_NB_JOINT_ACTUATED	 31

// Actuator Options:

#define Act_order 2 // 0th (rigid motor, no meca and no elec), 1st (electric), 2nd (mechanical) or 3rd (electrical, mechanical)

#define Act_type 1  // 1 SEA  2 PEA

// limiting actuator voltages
#define MAX_ACT_VOLTAGE 100.0


// -- Motors id -- //

// right leg
#define R_HIP_LAT_MOT   0
#define R_HIP_TRANS_MOT 1
#define R_HIP_SAG_MOT   2
#define R_KNEE_SAG_MOT  3
#define R_ANK_SAG_MOT   4
#define R_ANK_LAT_MOT   5

// left leg
#define L_HIP_LAT_MOT   6
#define L_HIP_TRANS_MOT 7
#define L_HIP_SAG_MOT   8
#define L_KNEE_SAG_MOT  9
#define L_ANK_SAG_MOT   10
#define L_ANK_LAT_MOT   11

// waist
#define WAIST_LAT_MOT   12
#define WAIST_SAG_MOT   13
#define WAIST_TRANS_MOT 14

// right arm
#define R_SH_SAG_MOT            15
#define R_SH_LAT_MOT            16
#define R_SH_TRANS_MOT          17
#define R_ELB_MOT               18
#define R_FORE_ARM_PLATE_MOT    19
#define R_WRJ1_MOT              20
#define R_WRJ2_MOT              21

// left arm
#define L_SH_SAG_MOT            22
#define L_SH_LAT_MOT            23
#define L_SH_TRANS_MOT          24
#define L_ELB_MOT               25
#define L_FORE_ARM_PLATE_MOT    26
#define L_WRJ1_MOT              27
#define L_WRJ2_MOT              28

// head
#define NECK_YAW_MOT    29
#define NECK_PITCH_MOT  30

/*--------------------*/
#endif
