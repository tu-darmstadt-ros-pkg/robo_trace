// Base
#include "robo_trace_ui_adaptor/ui_adaptor.hpp"
// Std
#include <sstream>
#include <iostream>
#include <boost/date_time.hpp>
// Std srvs
#include <std_srvs/SetBool.h>
// RoboTrace
#include <robo_trace_msgs/SetFloat.h>
#include <robo_trace_msgs/SetConfiguration.h>


namespace robo_trace_ui_adaptor {


RoboTraceUiAdaptor::RoboTraceUiAdaptor(ros::NodeHandle& node_handle) {

    ROS_INFO_STREAM("Starting the RoboTrace ui adaptor...");

    m_time_formats = {
        //std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%S")),
        //std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%M:%S")),
        std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%H:%M:%S"))
        
    };

    /*
        Load parameters from the parameter server.
    */
    ROS_INFO_STREAM(" - Loading parameters.");

    node_handle.param<std::string>("ui_topic_name_playback_progression", m_ui_topic_name_playback_progression, "playback_progression");
    node_handle.param<std::string>("ui_topic_name_playback_meta", m_ui_topic_name_playback_meta, "meta");
    node_handle.param<std::string>("ui_topic_name_signal_ui_init_completed", m_ui_topic_name_signal_ui_init_completed, "signal_ui_initialized");
    node_handle.param<std::string>("ui_topic_name_signal_rt_init_completed", m_ui_topic_name_signal_rt_init_completed, "signal_rt_initialized");
    node_handle.param<std::string>("ui_topic_name_signal_error_accepted", m_ui_topic_name_signal_error_accepted, "signal_error_accepted");
    node_handle.param<std::string>("ui_topic_name_signal_open_configure", m_ui_topic_name_signal_open_configure, "signal_open_configure");
    node_handle.param<std::string>("ui_service_name_set_playback_state", m_ui_service_name_set_playback_state, "set_playback_state");
    node_handle.param<std::string>("ui_service_name_set_playback_time", m_ui_service_name_set_playback_time, "set_playback_time");
    node_handle.param<std::string>("ui_service_name_set_playback_configuration", m_ui_service_name_set_playback_configuration, "set_playback_configuration");
    
    node_handle.param<std::string>("rt_topic_name_playback_clock", m_rt_topic_name_clock, "/robo_trace/clock");    
    node_handle.param<std::string>("rt_service_name_playback_toggle", m_rt_service_name_playback_toggle, "/robo_trace/toggle");    
    node_handle.param<std::string>("rt_service_name_playback_set_time", m_rt_service_name_playback_set_time, "/robo_trace/set_time");    
    node_handle.param<std::string>("rt_service_name_playback_configure", m_rt_service_name_playback_configure, "/robo_trace/configure");    
    
    /*
        Setup ui facing services.
    */
    ROS_INFO_STREAM(" - Starting ui facing services.");

    m_ui_publisher_playback_progression = node_handle.advertise<robo_trace_ui_adaptor::PlaybackProgression>(m_ui_topic_name_playback_progression, 1);
    m_ui_publisher_playback_meta = node_handle.advertise<robo_trace_ui_adaptor::PlaybackMeta>(m_ui_topic_name_playback_meta, 1);
    m_ui_publisher_signal_open_configure = node_handle.advertise<std_msgs::Empty>(m_ui_topic_name_signal_open_configure, 1);
    m_ui_publisher_signal_rt_init_completed = node_handle.advertise<std_msgs::Empty>(m_ui_topic_name_signal_rt_init_completed, 1, true);

    m_ui_subscriber_signal_ui_init_completed = node_handle.subscribe(m_ui_topic_name_signal_ui_init_completed.c_str(), 1, &RoboTraceUiAdaptor::onUiFinishedInitialization, this);
    m_ui_subscriber_signal_error_accepted = node_handle.subscribe(m_ui_topic_name_signal_error_accepted.c_str(), 1, &RoboTraceUiAdaptor::onUiErrorErrorAccepted, this);

    m_ui_service_set_playback_state = node_handle.advertiseService(m_ui_service_name_set_playback_state.c_str(), &RoboTraceUiAdaptor::onUiSetPlaybackState, this);
    m_ui_service_set_playback_time = node_handle.advertiseService(m_ui_service_name_set_playback_time.c_str(), &RoboTraceUiAdaptor::onUiSetPlaybackTime, this);
    m_ui_service_set_playback_configuration = node_handle.advertiseService(m_ui_service_name_set_playback_configuration.c_str(), &RoboTraceUiAdaptor::onUiSetPlaybackConfiguration, this);
    
    /*
        Setup robo trace facing services.
    */
    ROS_INFO_STREAM(" - Starting RoboTrace facing services.");

    m_rt_subscriber_clock = node_handle.subscribe(m_rt_topic_name_clock.c_str(), 1, &RoboTraceUiAdaptor::onPlaybackTime, this);

    m_rt_service_playback_toggle = node_handle.serviceClient<std_srvs::SetBool>(m_rt_service_name_playback_toggle.c_str());
    m_rt_service_playback_set_time = node_handle.serviceClient<robo_trace_msgs::SetFloat>(m_rt_service_name_playback_set_time.c_str());
    m_rt_service_playback_configure = node_handle.serviceClient<robo_trace_msgs::SetConfiguration>(m_rt_service_name_playback_configure.c_str());

    ROS_INFO_STREAM("Running.");

    /*
        Signal completion to the Ui.
    */

    std_msgs::Empty empty_msg;
    m_ui_publisher_signal_rt_init_completed.publish(empty_msg);

}

RoboTraceUiAdaptor::~RoboTraceUiAdaptor() {

}

std::string RoboTraceUiAdaptor::getTimeFormated(const double time) const {

    std::ostringstream progression_string;

    const int played_days = ((int) time) / 3600.0;
    const int time_current_day = ((int) time) % 3600;

    if (played_days > 0) {
        progression_string << std::setw(2) << std::setfill('0') << played_days << ":";
    }

    const int played_hours = time_current_day / 24.0;
    const int time_current_hour = time_current_day % 24;

    progression_string << std::setw(2) << std::setfill('0') << played_hours << ":";

    const int played_minutes = time_current_hour / 60.0;
    const int played_seconds =  time_current_hour % 60;

    progression_string << std::setw(2) << std::setfill('0') << played_seconds;

    return progression_string.str();
}

std::optional<boost::posix_time::time_duration> RoboTraceUiAdaptor::getPosixTime(const std::string& string) const {

    boost::posix_time::time_duration time_p;

    for (size_t i = 0; i < m_time_formats.size(); ++i) {
        
        std::istringstream is(string);
        is.imbue(m_time_formats[i]);
        is >> time_p;

        if (time_p != boost::posix_time::time_duration()) {
            ROS_INFO_STREAM("Converted " << string << " to " << boost::posix_time::to_simple_string(time_p));
            return time_p;
        }

    }

    return {};

}

void RoboTraceUiAdaptor::onPlaybackTime(const rosgraph_msgs::Clock::ConstPtr& message) {

    if (!m_playback_time_end || !m_playback_time_start) {
        return;
    }

    const ros::Time& time_ros = message->clock;  
    const double time = time_ros.toSec();

    const double time_played = time - m_playback_time_start.value();
    const double progression_share = time_played / (m_playback_time_end.value() - m_playback_time_start.value());

    // ROS_INFO_STREAM("Progression: " << progression_share << " " << played_minutes << ":" << played_seconds);

    robo_trace_ui_adaptor::PlaybackProgression update;
    update.progression = progression_share;
    update.formated = getTimeFormated(time_played);

    m_ui_publisher_playback_progression.publish(update);

}

void RoboTraceUiAdaptor::onUiFinishedInitialization(const std_msgs::Empty::Ptr& message) {
    ROS_INFO_STREAM("Ui acknowledged initialization signal.");

    std_msgs::Empty empty_msg;
    m_ui_publisher_signal_open_configure.publish(empty_msg);

}

void RoboTraceUiAdaptor::onUiErrorErrorAccepted(const std_msgs::Empty::Ptr& message) {

}

bool RoboTraceUiAdaptor::onUiSetPlaybackState(robo_trace_ui_adaptor::SetPlaybackState::Request& request, robo_trace_ui_adaptor::SetPlaybackState::Response& response) {
    ROS_INFO_STREAM("Set playback state.");

    std_srvs::SetBool rt_request;
    // TODO: Get rid of this inversed definition.
    rt_request.request.data = !request.run;    

    const bool success = m_rt_service_playback_toggle.call(rt_request);
 
    if (!success) {
        response.status.message = "Service call for toggling the playback state returned unsuccessful.";
    }

    response.status.success = success;
    return true;
}

bool RoboTraceUiAdaptor::onUiSetPlaybackTime(robo_trace_ui_adaptor::SetPlaybackTime::Request& request, robo_trace_ui_adaptor::SetPlaybackTime::Response& response) {
   
    if (!m_playback_time_end || !m_playback_time_start) {
        return false;
    }

    const double offset = m_playback_time_constrain_start.value() - m_playback_time_start.value();
    const double seconds = offset + request.progression * (m_playback_time_constrain_end.value() - m_playback_time_constrain_start.value());
    ROS_INFO_STREAM("Set playback time (Seconds: " << seconds << " Fraction: " << request.progression << ").");

    robo_trace_msgs::SetFloat rt_request;
    rt_request.request.data = seconds;

    const bool success = m_rt_service_playback_set_time.call(rt_request);
    
    if (!success) {
        response.status.message = "Service call for setting the playback time returned unsuccessful.";
    }

    response.status.success = success;
    return true;
}

bool RoboTraceUiAdaptor::onUiSetPlaybackConfiguration(robo_trace_ui_adaptor::SetPlaybackConfiguration::Request& request, robo_trace_ui_adaptor::SetPlaybackConfiguration::Response& response) {
    ROS_INFO_STREAM("Configure request received: " << request.name);

    robo_trace_msgs::SetConfiguration rt_request;
    rt_request.request.name = request.name;

    bool success = m_rt_service_playback_configure.call(rt_request);
    success = success && rt_request.response.success;

    if (!success) {
        response.status.message = "Failed loading record: " + rt_request.response.message;
        return true;
    }

    m_playback_time_start = rt_request.response.start;
    m_playback_time_end = rt_request.response.end;


    if (!request.to.empty()) {

       std::optional<boost::posix_time::time_duration> time = getPosixTime(request.to);

        if (time) {
            m_playback_time_constrain_start = ros::Time::fromBoost(time.value()).toSec();
        } else {
            response.status.success = false;
            response.status.message = "The start time is misformated.";
            return true;
        }
    }

    if (!request.from.empty()) {

        std::optional<boost::posix_time::time_duration> time = getPosixTime(request.from);

        if (time) {
            m_playback_time_constrain_end = ros::Time::fromBoost(time.value()).toSec();
        } else {
            response.status.success = false;
            response.status.message = "The end time is misformated.";
            return true;
        }
    }

    const double record_duration = m_playback_time_end.value() - m_playback_time_start.value();
    ROS_INFO_STREAM("Duration: " << record_duration);

    if (m_playback_time_constrain_start && m_playback_time_constrain_end) {
        
        ROS_INFO_STREAM("Start: " << m_playback_time_constrain_start.value() << " End: " << m_playback_time_constrain_end.value());

        if (m_playback_time_constrain_end.value() < m_playback_time_constrain_start.value()) {
            response.status.success = false;
            response.status.message = "The start time is greater than the end time.";
            return true; 
        }
    } 

    if (m_playback_time_constrain_start) {

        if (record_duration < m_playback_time_constrain_start.value()) {
            response.status.success = false;
            response.status.message = "The start time is beyond the record end time.";
            return true;
        }

        m_playback_time_constrain_start = m_playback_time_constrain_start.value() + m_playback_time_start.value();

    } else {
        m_playback_time_constrain_start = m_playback_time_start;
    }


    if (m_playback_time_constrain_end) {

        if (record_duration < m_playback_time_constrain_end.value()) {
            response.status.success = false;
            response.status.message = "The end time is beyond the record end time.";
            return true;
        }

        m_playback_time_constrain_end = m_playback_time_constrain_end.value() + m_playback_time_start.value();

    } else {
        m_playback_time_constrain_end = m_playback_time_end.value();
    }

    const double end_time = m_playback_time_constrain_end.value() - m_playback_time_start.value();

    robo_trace_ui_adaptor::PlaybackMeta meta_update;
    meta_update.length_formated = getTimeFormated(end_time);
    
    m_ui_publisher_playback_meta.publish(meta_update);

    response.status.success = success;
    return true;

}


}