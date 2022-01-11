#pragma once
// Std
#include <memory>
#include <vector>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/babel_fish.h>
// Std service definitions
#include <std_srvs/SetBool.h>
// Project Msg definitions
#include <robo_trace_msgs/SetFloat.h>
#include <robo_trace_msgs/SetConfiguration.h>
// Project
#include "robo_trace/processing/constructor.hpp"
#include "robo_trace/modes/replay/publisher.hpp"
#include "robo_trace/modes/replay/time.hpp"
#include "robo_trace/modes/replay/player.hpp"


namespace robo_trace::replay {

class RoboTraceReplayService final : public PlayerBase {

public:

    typedef std::shared_ptr<RoboTraceReplayService> Ptr;
    typedef std::shared_ptr<const RoboTraceReplayService> ConstPtr;

public:

    /**
     *
     */
    RoboTraceReplayService(ros::NodeHandle& node_handle);

    /**
     *
     */
    ~RoboTraceReplayService();
    
    /**
     * 
     */
    void run();

protected:

    /**
     * 
     */
    virtual void initialize() final override;
    
private:

    /**
     * 
     */
    bool onConfigure(robo_trace_msgs::SetConfiguration::Request& request, robo_trace_msgs::SetConfiguration::Response& response);

    /**
     *
     */
    bool onPlaybackToggle(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    /**
     *
     */
    bool onSetPlaybackTime(robo_trace_msgs::SetFloat::Request& request, robo_trace_msgs::SetFloat::Response& response);

    /**
     *
     */
    bool onSetPlaybackSpeed(robo_trace_msgs::SetFloat::Request& request, robo_trace_msgs::SetFloat::Response& response);

private:

    /** */
    ros::ServiceServer m_service_configure;
    /** */
    ros::ServiceServer m_service_playback_toggle;
    /** */
    ros::ServiceServer m_service_set_playback_time;
    /** */
    ros::ServiceServer m_service_set_playback_speed;

    /** */
    bool m_configured;
    /** */
    bool m_flushed;
    /** */
    bool m_paused;
    
    /** */
    double m_time_start;
    /** */
    double m_time_end;

};

}