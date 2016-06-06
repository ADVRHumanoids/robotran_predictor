#include <yarp/os/all.h>

#include "robotran_predictor_thread.h"
#include "robotran_predictor_constants.h"

#include <mbs_load_xml.h>
#include <mbs_part.h>
#include <mbs_dirdyn.h>

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
    mbs_dirdyn->options->tf  = 10.0;

    return true;
}

void robotran_predictor_thread::run()
{   
    int seq_num = 0;
    if(state_input.getCommand(actual_state, seq_num)) {
        actual_state.print();
    }

}    
