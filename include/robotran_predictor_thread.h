#ifndef robotran_predictor_THREAD_H_
#define robotran_predictor_THREAD_H_

#include <GYM/generic_thread.hpp>
#include <GYM/internal_yarp_command_interface.hpp>

#include <drc_shared/yarp_msgs/robotran_predictor_msg.h>

#include <mbs_data.h>

/**
 * @brief robotran_predictor control thread
 * 
 **/
class robotran_predictor_thread : public generic_thread
{
private:   
    walkman::internal_yarp_command_interface<robotran_predictor::state_input> state_input;
    walkman::internal_yarp_command_interface<robotran_predictor::request> request;
    walkman::internal_yarp_command_sender_interface<robotran_predictor::prediction> prediction;
    
    robotran_predictor::state_input actual_state;

    MbsData* mbs_data;
    MbsDirdyn *mbs_dirdyn;
    
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     robotran_predictor_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief robotran_predictor control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief robotran_predictor control thread close function
     * 
     * @return true on succes, false otherwise
     */
    void custom_release();
    
    /**
     * @brief robotran_predictor control thread main loop
     * 
     */
    virtual void run();
    
};

#endif
