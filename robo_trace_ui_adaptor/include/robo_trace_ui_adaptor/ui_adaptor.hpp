// Std
#include <string>
#include <optional>
#include <vector>
// Ros
#include <ros/ros.h>
// Std msgs
#include <std_msgs/Empty.h>
// RosGraph
#include "rosgraph_msgs/Clock.h"
// Project
#include "robo_trace_ui_adaptor/PlaybackProgression.h"
#include "robo_trace_ui_adaptor/SetPlaybackState.h"
#include "robo_trace_ui_adaptor/SetPlaybackTime.h"
#include "robo_trace_ui_adaptor/SetPlaybackConfiguration.h"
#include <robo_trace_ui_adaptor/PlaybackMeta.h>


namespace robo_trace_ui_adaptor {

class RoboTraceUiAdaptor {

public:

    RoboTraceUiAdaptor(ros::NodeHandle& node_handle);

    ~RoboTraceUiAdaptor();

protected:

    void onPlaybackTime(const rosgraph_msgs::Clock::ConstPtr& message);


    void onUiFinishedInitialization(const std_msgs::Empty::Ptr& message);

    void onUiErrorErrorAccepted(const std_msgs::Empty::Ptr& message);

    bool onUiSetPlaybackState(robo_trace_ui_adaptor::SetPlaybackState::Request& request, robo_trace_ui_adaptor::SetPlaybackState::Response& response);

    bool onUiSetPlaybackTime(robo_trace_ui_adaptor::SetPlaybackTime::Request& request, robo_trace_ui_adaptor::SetPlaybackTime::Response& response);

    bool onUiSetPlaybackConfiguration(robo_trace_ui_adaptor::SetPlaybackConfiguration::Request& request, robo_trace_ui_adaptor::SetPlaybackConfiguration::Response& response);
   
private:

    std::string getTimeFormated(const double time) const;

    std::optional<boost::posix_time::time_duration> getPosixTime(const std::string& string) const;

private:

    std::vector<std::locale> m_time_formats;

    std::optional<double> m_playback_time_start;
    std::optional<double> m_playback_time_end;

    std::optional<double> m_playback_time_constrain_start;
    std::optional<double> m_playback_time_constrain_end;

private:

    std::string m_ui_topic_name_playback_progression;
    std::string m_ui_topic_name_playback_meta;
    std::string m_ui_topic_name_signal_ui_init_completed;
    std::string m_ui_topic_name_signal_rt_init_completed;
    std::string m_ui_topic_name_signal_error_accepted;
    std::string m_ui_topic_name_signal_open_configure;
    std::string m_ui_service_name_set_playback_state;
    std::string m_ui_service_name_set_playback_time;
    std::string m_ui_service_name_set_playback_configuration;
    
    ros::Publisher m_ui_publisher_playback_progression;
    ros::Publisher m_ui_publisher_playback_meta;
    ros::Publisher m_ui_publisher_signal_open_configure;
    ros::Publisher m_ui_publisher_signal_rt_init_completed;
    
    ros::Subscriber m_ui_subscriber_signal_ui_init_completed;
    ros::Subscriber m_ui_subscriber_signal_error_accepted;

    ros::ServiceServer m_ui_service_set_playback_state;
    ros::ServiceServer m_ui_service_set_playback_time;
    ros::ServiceServer m_ui_service_set_playback_configuration;

    std::string m_rt_topic_name_clock;
    std::string m_rt_service_name_playback_toggle;
    std::string m_rt_service_name_playback_set_time;
    std::string m_rt_service_name_playback_configure;
    
    ros::Subscriber m_rt_subscriber_clock;

    ros::ServiceClient m_rt_service_playback_toggle;
    ros::ServiceClient m_rt_service_playback_set_time;
    ros::ServiceClient m_rt_service_playback_configure;

};
    
}