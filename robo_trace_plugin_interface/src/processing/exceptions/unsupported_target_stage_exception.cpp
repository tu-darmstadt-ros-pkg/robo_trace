// Base
#include "robo_trace_plugin_interface/processing/exceptions/unsupported_target_stage_exception.hpp"


namespace robo_trace {

UnsupportedTargetStageException::UnsupportedTargetStageException() 
: UnsupportedTargetStageException("") {
    //
};

UnsupportedTargetStageException::UnsupportedTargetStageException(const std::string& what) 
: ros::Exception(what) {
    //
};

// I like making stuff explicit.
UnsupportedTargetStageException::~UnsupportedTargetStageException() = default;

}