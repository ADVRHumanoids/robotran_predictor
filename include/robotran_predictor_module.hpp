#ifndef robotran_predictor_MODULE_HPP_
#define robotran_predictor_MODULE_HPP_

#include <GYM/generic_module.hpp>

#include "robotran_predictor_thread.h"
#include "robotran_predictor_constants.h"

/**
 * @brief robotran_predictor module derived from generic_module
 * 
 * @author 
 */
class robotran_predictor_module : public generic_module<robotran_predictor_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    robotran_predictor_module(   int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : generic_module<robotran_predictor_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
