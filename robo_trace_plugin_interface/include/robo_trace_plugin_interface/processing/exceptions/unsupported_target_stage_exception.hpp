#pragma once
// Ros
#include <ros/exception.h>


namespace robo_trace {

class UnsupportedTargetStageException : public ros::Exception {

public:

    UnsupportedTargetStageException();

    UnsupportedTargetStageException(const std::string& what);

    ~UnsupportedTargetStageException();

};   

}