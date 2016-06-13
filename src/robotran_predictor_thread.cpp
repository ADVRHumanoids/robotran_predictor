#include <yarp/os/all.h>

#include "robotran_predictor_thread.h"
#include "robotran_predictor_constants.h"

#include <mbs_load_xml.h>
#include <mbs_part.h>
#include <mbs_dirdyn.h>

#include "robotran_user_all_id.h"
#include "robotran_ActuatorsDefinitions.h"

#include "user_IO.h"

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
    //mbs_dirdyn->options->realtime = 1;

    mbs_dirdyn_init(mbs_dirdyn, mbs_data);

    return true;
}

void robotran_predictor_thread::custom_release()
{
    mbs_dirdyn_finish(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_data(mbs_data);
    //return true;
}


void robotran_predictor_thread::run()
{   
    int seq_num = 0;
    if(state_input.getCommand(actual_state, seq_num)) {
        actual_state.print();
    }

    // reset the state
    reset_model_state();

    // run the simulation
    mbs_dirdyn->options->tf += 0.001;  // arbitrary period of computation
    mbs_dirdyn_loop(mbs_dirdyn, mbs_data);

}

void robotran_predictor_thread::reset_model_state()
{

    // right arm

    // TODO: add shift in joint, motor

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
    mbs_data->ux[R_SH_SAG_MOT+1] = actual_state.motor_pos[24] - 0.349066;
    mbs_data->ux[R_SH_LAT_MOT+1] = actual_state.motor_pos[25] + 0.715585 ;
    mbs_data->ux[R_SH_TRANS_MOT+1] = actual_state.motor_pos[26];
    mbs_data->ux[R_ELB_MOT+1] = actual_state.motor_pos[27];
    mbs_data->ux[R_FORE_ARM_PLATE_MOT+1] = actual_state.motor_pos[28];
    mbs_data->ux[R_WRJ1_MOT+1] = actual_state.motor_pos[29];
    mbs_data->ux[R_WRJ2_MOT+1] = actual_state.motor_pos[30];

    // motor velocity
    mbs_data->ux[R_SH_SAG_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[24];
    mbs_data->ux[R_SH_LAT_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[25];
    mbs_data->ux[R_SH_TRANS_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[26];
    mbs_data->ux[R_ELB_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[27];
    mbs_data->ux[R_FORE_ARM_PLATE_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[28];
    mbs_data->ux[R_WRJ1_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[29];
    mbs_data->ux[R_WRJ2_MOT+COMAN_NB_JOINT_ACTUATED+1] = actual_state.motor_vel[30];

    // cheat to cancel joint control effect (should be replaced by an update of the ref from Yarp)
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_SAG_MOT+1] = mbs_data->q[RShSag_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_LAT_MOT+1] = mbs_data->q[RShLat_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_SH_TRANS_MOT+1] = mbs_data->q[RShYaw_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_ELB_MOT+1] = mbs_data->q[RElbj_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_FORE_ARM_PLATE_MOT+1] = mbs_data->q[RForearmPlate_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_WRJ1_MOT+1] = mbs_data->q[RWrj1_id];
    mbs_data->user_IO->cvs->Outputs->q_ref[R_WRJ2_MOT+1] = mbs_data->q[RWrj2_id];

}
