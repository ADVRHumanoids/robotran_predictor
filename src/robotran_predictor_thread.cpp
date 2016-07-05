#include <yarp/os/all.h>

#include "robotran_predictor_thread.h"
#include "robotran_predictor_constants.h"

#include <mbs_load_xml.h>
#include <mbs_part.h>
#include <mbs_dirdyn.h>

#include "robotran_user_all_id.h"
#include "robotran_ActuatorsDefinitions.h"

#include "user_IO.h"
#include "PIDs_Struct.h"

robotran_predictor_thread::robotran_predictor_thread( std::string module_prefix, 
                                                      yarp::os::ResourceFinder rf, 
                                                      std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    generic_thread( module_prefix, rf, ph ),
    state_input("robotran_predictor", "/state_input:i"),
    request("robotran_predictor", "/request:i"),
    prediction("robotran_predictor", "/prediction")

{

}

bool robotran_predictor_thread::custom_init()
{
    // Get mbs file
    yarp::os::ConstString mbs = get_resource_finder().find("mbs_file").asString();
    std::cout << "mbs file used is " << mbs << std::endl;

    // Load mbs file
    mbs_data = mbs_load(mbs.c_str());

    // coordinate partitioning
    MbsPart *mbs_part;

    mbs_part = mbs_new_part(mbs_data);
    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 1;
    mbs_run_part(mbs_part, mbs_data);
    mbs_delete_part(mbs_part);

    // set direct dynamics options
    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options (see documentations for additional options)
    mbs_dirdyn->options->dt0 = 5e-4;
    mbs_dirdyn->options->tf  = 0.5;
    mbs_dirdyn->options->save2file = 0;
    mbs_dirdyn->options->realtime = 1;

    mbs_dirdyn_init(mbs_dirdyn, mbs_data);

    // set initial state
    int seq_num = 0;
    if(state_input.getCommand(actual_state, seq_num)) {
        reset_model_state();
    }

    // init FSM
    FSM_state = WAITING_REQUEST;
    reset_time = 0.;

    // init prediction
    actual_prediction.is_valid = false;
    actual_prediction.left_knee_angle = 0.;

    return true;
}

void robotran_predictor_thread::custom_release()
{
    mbs_dirdyn_finish(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_data(mbs_data);
    return;
}


void robotran_predictor_thread::run()
{   
    int seq_num = 0;

    if(FSM_state == WAITING_REQUEST){

        // check if new request sent
        if(request.getCommand(actual_request, seq_num)) {

            if(actual_request.process_request) {  // a request is received (with true flag)

                std::cout << std::endl <<"receive new request with request status = " << actual_request.process_request << std::endl<< std::endl<< std::endl;

                FSM_state = RESETING_SIMU_STATE;

            }
        }
    }

    else if(FSM_state == RESETING_SIMU_STATE){

        if(state_input.getCommand(actual_state, seq_num)) {

            // reset the model
            reset_time = mbs_dirdyn->options->tf;
            reset_model_state();

            FSM_state = PREDICTING_REQUEST;
        }

    }

    else if(FSM_state == PREDICTING_REQUEST)
    {
        // run the simulation
        mbs_dirdyn->options->tf += 0.005;  // arbitrary period of computation
        mbs_dirdyn_loop(mbs_dirdyn, mbs_data);

        // request completed
        if((mbs_dirdyn->options->tf - reset_time) > 0.5)  // arbitrary time
        {

            FSM_state = SENDING_PREDICTION;

        }
    }

    else if(FSM_state == SENDING_PREDICTION)
    {
        // update prediction
        actual_prediction.is_valid = true;
        actual_prediction.left_knee_angle = mbs_data->q[LKneeSag_id];

        prediction.sendCommand(actual_prediction);

        std::cout << std::endl <<"request processed, prediction = ..." << std::endl<< std::endl<< std::endl;

        FSM_state = WAITING_REQUEST;
    }

    // TODO: Add a floating base computation based on foot position
    // TODO: Rotate model (mass, ...) of walkman model
    // TODO: Check position shift and zero
    // TODO: trigger reset based on messages

}

void robotran_predictor_thread::reset_model_state()
{

    // floating base
    mbs_data->q[T1_id] = 0.;
    mbs_data->q[T2_id] = 0.;
    mbs_data->q[T3_id] = 1.103324;
    mbs_data->q[R1_id] = 0.;
    mbs_data->q[R2_id] = 0.;
    mbs_data->q[R3_id] = 0.;

    mbs_data->qd[T1_id] = 0.;
    mbs_data->qd[T2_id] = 0.;
    mbs_data->qd[T3_id] = 0.;
    mbs_data->qd[R1_id] = 0.;
    mbs_data->qd[R2_id] = 0.;
    mbs_data->qd[R3_id] = 0.;

    //left leg

    // joint position
    mbs_data->q[LHipLat_id]  = actual_state.link_pos[0];
    mbs_data->q[LHipYaw_id]  = actual_state.link_pos[1];
    mbs_data->q[LHipSag_id]  = actual_state.link_pos[2];
    mbs_data->q[LKneeSag_id] = actual_state.link_pos[3];
    mbs_data->q[LAnkSag_id]  = actual_state.link_pos[4];
    mbs_data->q[LAnkLat_id]  = actual_state.link_pos[5];

    // joint velocity
    mbs_data->qd[LHipLat_id]  = actual_state.link_vel[0];
    mbs_data->qd[LHipYaw_id]  = actual_state.link_vel[1];
    mbs_data->qd[LHipSag_id]  = actual_state.link_vel[2];
    mbs_data->qd[LKneeSag_id] = actual_state.link_vel[3];
    mbs_data->qd[LAnkSag_id]  = actual_state.link_vel[4];
    mbs_data->qd[LAnkLat_id]  = actual_state.link_vel[5];

    // motor position
    mbs_data->ux[L_HIP_LAT_MOT  +1]   = actual_state.motor_pos[0];
    mbs_data->ux[L_HIP_TRANS_MOT+1]   = actual_state.motor_pos[1];
    mbs_data->ux[L_HIP_SAG_MOT  +1]   = actual_state.motor_pos[2];
    mbs_data->ux[L_KNEE_SAG_MOT +1]   = actual_state.motor_pos[3];
    mbs_data->ux[L_ANK_SAG_MOT  +1]   = actual_state.motor_pos[4];
    mbs_data->ux[L_ANK_LAT_MOT  +1]   = actual_state.motor_pos[5];

    // motor velocity
    mbs_data->ux[L_HIP_LAT_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[0];
    mbs_data->ux[L_HIP_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[1];
    mbs_data->ux[L_HIP_SAG_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[2];
    mbs_data->ux[L_KNEE_SAG_MOT +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[3];
    mbs_data->ux[L_ANK_SAG_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[4];
    mbs_data->ux[L_ANK_LAT_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[5];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[L_HIP_LAT_MOT  +1] = actual_state.pos_ref[0];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_HIP_TRANS_MOT+1] = actual_state.pos_ref[1];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_HIP_SAG_MOT  +1] = actual_state.pos_ref[2];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_KNEE_SAG_MOT +1] = actual_state.pos_ref[3];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_ANK_SAG_MOT  +1] = actual_state.pos_ref[4];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_ANK_LAT_MOT  +1] = actual_state.pos_ref[5];


    //right leg

    // joint position
    mbs_data->q[RHipLat_id]  = actual_state.link_pos[6];
    mbs_data->q[RHipYaw_id]  = actual_state.link_pos[7];
    mbs_data->q[RHipSag_id]  = actual_state.link_pos[8];
    mbs_data->q[RKneeSag_id] = actual_state.link_pos[9];
    mbs_data->q[RAnkSag_id]  = actual_state.link_pos[10];
    mbs_data->q[RAnkLat_id]  = actual_state.link_pos[11];

    // joint velocity
    mbs_data->qd[RHipLat_id]  = actual_state.link_vel[6];
    mbs_data->qd[RHipYaw_id]  = actual_state.link_vel[7];
    mbs_data->qd[RHipSag_id]  = actual_state.link_vel[8];
    mbs_data->qd[RKneeSag_id] = actual_state.link_vel[9];
    mbs_data->qd[RAnkSag_id]  = actual_state.link_vel[10];
    mbs_data->qd[RAnkLat_id]  = actual_state.link_vel[11];

    // motor position
    mbs_data->ux[R_HIP_LAT_MOT  +1]   = actual_state.motor_pos[6];
    mbs_data->ux[R_HIP_TRANS_MOT+1]   = actual_state.motor_pos[7];
    mbs_data->ux[R_HIP_SAG_MOT  +1]   = actual_state.motor_pos[8];
    mbs_data->ux[R_KNEE_SAG_MOT +1]   = actual_state.motor_pos[9];
    mbs_data->ux[R_ANK_SAG_MOT  +1]   = actual_state.motor_pos[10];
    mbs_data->ux[R_ANK_LAT_MOT  +1]   = actual_state.motor_pos[11];

    // motor velocity
    mbs_data->ux[R_HIP_LAT_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[6];
    mbs_data->ux[R_HIP_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[7];
    mbs_data->ux[R_HIP_SAG_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[8];
    mbs_data->ux[R_KNEE_SAG_MOT +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[9];
    mbs_data->ux[R_ANK_SAG_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[10];
    mbs_data->ux[R_ANK_LAT_MOT  +COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[11];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[R_HIP_LAT_MOT  +1] = actual_state.pos_ref[6];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_HIP_TRANS_MOT+1] = actual_state.pos_ref[7];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_HIP_SAG_MOT  +1] = actual_state.pos_ref[8];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_KNEE_SAG_MOT +1] = actual_state.pos_ref[9];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_ANK_SAG_MOT  +1] = actual_state.pos_ref[10];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_ANK_LAT_MOT  +1] = actual_state.pos_ref[11];

    //torso

    // joint position
    mbs_data->q[WaistLat_id] = actual_state.link_pos[12];
    mbs_data->q[WaistSag_id] = actual_state.link_pos[13]*(-1) + 0.453786;
    mbs_data->q[WaistYaw_id] = actual_state.link_pos[14];

    // joint velocity
    mbs_data->qd[WaistLat_id] = actual_state.link_vel[12];
    mbs_data->qd[WaistSag_id] = actual_state.link_vel[13]*(1);
    mbs_data->qd[WaistYaw_id] = actual_state.link_vel[14];

    // motor position
    mbs_data->ux[WAIST_LAT_MOT  +1] = actual_state.motor_pos[12];
    mbs_data->ux[WAIST_SAG_MOT  +1] = actual_state.motor_pos[13]*(-1) + 0.453786;
    mbs_data->ux[WAIST_TRANS_MOT+1] = actual_state.motor_pos[14];

    // motor velocity
    mbs_data->ux[WAIST_LAT_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[12];
    mbs_data->ux[WAIST_SAG_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[13]*(-1);
    mbs_data->ux[WAIST_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[14];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[WAIST_LAT_MOT  +1] = actual_state.pos_ref[12];
    mbs_data->user_IO->cvs->Outputs->q_ref[WAIST_SAG_MOT  +1] = actual_state.pos_ref[13]*(-1) + 0.453786;
    mbs_data->user_IO->cvs->Outputs->q_ref[WAIST_TRANS_MOT+1] = actual_state.pos_ref[14];

    // left arm

    // joint position
    mbs_data->q[LShSag_id]          = actual_state.link_pos[15] - 0.349066;
    mbs_data->q[LShLat_id]          = actual_state.link_pos[16] - 0.715585;
    mbs_data->q[LShYaw_id]          = actual_state.link_pos[17];
    mbs_data->q[LElbj_id]           = actual_state.link_pos[18];
    mbs_data->q[LForearmPlate_id]   = actual_state.link_pos[19];
    mbs_data->q[LWrj1_id]           = actual_state.link_pos[20];
    mbs_data->q[LWrj2_id]           = actual_state.link_pos[21];

    // joint velocity
    mbs_data->qd[LShSag_id]         = actual_state.link_vel[15];
    mbs_data->qd[LShLat_id]         = actual_state.link_vel[16];
    mbs_data->qd[LShYaw_id]         = actual_state.link_vel[17];
    mbs_data->qd[LElbj_id]          = actual_state.link_vel[18];
    mbs_data->qd[LForearmPlate_id]  = actual_state.link_vel[19];
    mbs_data->qd[LWrj1_id]          = actual_state.link_vel[20];
    mbs_data->qd[LWrj2_id]          = actual_state.link_vel[21];

    // motor position
    mbs_data->ux[L_SH_SAG_MOT        +1] = actual_state.motor_pos[15] - 0.349066;
    mbs_data->ux[L_SH_LAT_MOT        +1] = actual_state.motor_pos[16] - 0.715585 ;
    mbs_data->ux[L_SH_TRANS_MOT      +1] = actual_state.motor_pos[17];
    mbs_data->ux[L_ELB_MOT           +1] = actual_state.motor_pos[18];
    mbs_data->ux[L_FORE_ARM_PLATE_MOT+1] = actual_state.motor_pos[19];
    mbs_data->ux[L_WRJ1_MOT          +1] = actual_state.motor_pos[20];
    mbs_data->ux[L_WRJ2_MOT          +1] = actual_state.motor_pos[21];

    // motor velocity
    mbs_data->ux[L_SH_SAG_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[15];
    mbs_data->ux[L_SH_LAT_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[16];
    mbs_data->ux[L_SH_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[17];
    mbs_data->ux[L_ELB_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[18];
    mbs_data->ux[L_FORE_ARM_PLATE_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[19];
    mbs_data->ux[L_WRJ1_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[20];
    mbs_data->ux[L_WRJ2_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[21];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[L_SH_SAG_MOT        +1] = actual_state.pos_ref[15] - 0.349066;
    mbs_data->user_IO->cvs->Outputs->q_ref[L_SH_LAT_MOT        +1] = actual_state.pos_ref[16] - 0.715585;
    mbs_data->user_IO->cvs->Outputs->q_ref[L_SH_TRANS_MOT      +1] = actual_state.pos_ref[17];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_ELB_MOT           +1] = actual_state.pos_ref[18];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_FORE_ARM_PLATE_MOT+1] = actual_state.pos_ref[19];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_WRJ1_MOT          +1] = actual_state.pos_ref[20];
    mbs_data->user_IO->cvs->Outputs->q_ref[L_WRJ2_MOT          +1] = actual_state.pos_ref[21];

    // head

    // joint position
    mbs_data->q[NeckYawj_id] = actual_state.link_pos[22];
    mbs_data->q[NeckPitchj_id] = actual_state.link_pos[23];

    // joint velocity
    mbs_data->qd[NeckYawj_id] = actual_state.link_vel[22];
    mbs_data->qd[NeckPitchj_id] = actual_state.link_vel[23];

    // motor position
    mbs_data->ux[NECK_YAW_MOT   +1] = actual_state.motor_pos[22];
    mbs_data->ux[NECK_PITCH_MOT +1] = actual_state.motor_pos[23];

    // motor velocity
    mbs_data->ux[NECK_YAW_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[22];
    mbs_data->ux[NECK_PITCH_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[23];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[NECK_YAW_MOT   +1] = actual_state.pos_ref[22];
    mbs_data->user_IO->cvs->Outputs->q_ref[NECK_PITCH_MOT +1] = actual_state.pos_ref[23];

    // right arm

    // joint position
    mbs_data->q[RShSag_id] = actual_state.link_pos[24] - 0.349066;
    mbs_data->q[RShLat_id] = actual_state.link_pos[25] + 0.715585;
    mbs_data->q[RShYaw_id] = actual_state.link_pos[26];
    mbs_data->q[RElbj_id] = actual_state.link_pos[27];
    mbs_data->q[RForearmPlate_id] = actual_state.link_pos[28];
    mbs_data->q[RWrj1_id] = actual_state.link_pos[29];
    mbs_data->q[RWrj2_id] = actual_state.link_pos[30];

    // joint velocity
    mbs_data->qd[RShSag_id] = actual_state.link_vel[24];
    mbs_data->qd[RShLat_id] = actual_state.link_vel[25];
    mbs_data->qd[RShYaw_id] = actual_state.link_vel[26];
    mbs_data->qd[RElbj_id] = actual_state.link_vel[27];
    mbs_data->qd[RForearmPlate_id] = actual_state.link_vel[28];
    mbs_data->qd[RWrj1_id] = actual_state.link_vel[29];
    mbs_data->qd[RWrj2_id] = actual_state.link_vel[30];

    // motor position
    mbs_data->ux[R_SH_SAG_MOT        +1] = actual_state.motor_pos[24] - 0.349066;
    mbs_data->ux[R_SH_LAT_MOT        +1] = actual_state.motor_pos[25] + 0.715585;
    mbs_data->ux[R_SH_TRANS_MOT      +1] = actual_state.motor_pos[26];
    mbs_data->ux[R_ELB_MOT           +1] = actual_state.motor_pos[27];
    mbs_data->ux[R_FORE_ARM_PLATE_MOT+1] = actual_state.motor_pos[28];
    mbs_data->ux[R_WRJ1_MOT          +1] = actual_state.motor_pos[29];
    mbs_data->ux[R_WRJ2_MOT          +1] = actual_state.motor_pos[30];

    // motor velocity
    mbs_data->ux[R_SH_SAG_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[24];
    mbs_data->ux[R_SH_LAT_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[25];
    mbs_data->ux[R_SH_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[26];
    mbs_data->ux[R_ELB_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[27];
    mbs_data->ux[R_FORE_ARM_PLATE_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[28];
    mbs_data->ux[R_WRJ1_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[29];
    mbs_data->ux[R_WRJ2_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[30];

    // position reference
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_SAG_MOT        +1] = actual_state.pos_ref[24] - 0.349066;
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_LAT_MOT        +1] = actual_state.pos_ref[25] + 0.715585;
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_TRANS_MOT      +1] = actual_state.pos_ref[26];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_ELB_MOT           +1] = actual_state.pos_ref[27];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_FORE_ARM_PLATE_MOT+1] = actual_state.pos_ref[28];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_WRJ1_MOT          +1] = actual_state.pos_ref[29];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_WRJ2_MOT          +1] = actual_state.pos_ref[30];

    //for (int i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
        //mbs_data->user_IO->PIDs_pos->int_err[i+1] + 0.;

    // Update state variables
    for(int i=1;i<=mbs_data->nqu;i++)
    {
        mbs_dirdyn->y[i-1] = mbs_data->q[mbs_data->qu[i]];
        mbs_dirdyn->y[i+mbs_data->nqu-1] = mbs_data->qd[mbs_data->qu[i]];
    }
    for(int i=1;i<=mbs_data->Nux;i++)
    {
        mbs_dirdyn->y[i+2*mbs_data->nqu-1] = mbs_data->ux[i];
    }



}
