#pragma once
// Ros
#include "ros/ros.h"
// Nodelet
#include "nodelet/nodelet.h"


namespace robo_trace {

class RoboTraceNodelet final : public nodelet::Nodelet {

public:

    /**
     * 
     */
    RoboTraceNodelet();

    /**
     * 
     */
    ~RoboTraceNodelet();

private:

    /**
     * 
     */
    virtual void onInit() final override; 

};

}

PLUGINLIB_EXPORT_CLASS(robo_trace::RoboTraceNodelet, nodelet::Nodelet);