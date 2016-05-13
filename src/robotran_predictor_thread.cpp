#include <yarp/os/all.h>

#include "robotran_predictor_thread.h"
#include "robotran_predictor_constants.h"

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
    return true;
}

void robotran_predictor_thread::run()
{   
    int seq_num = 0;
    if(state_input.getCommand(actual_state, seq_num)) {
        actual_state.print();
    }

}    
