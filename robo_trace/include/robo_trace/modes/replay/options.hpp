#pragma once

// Std
#include <string>
#include <memory>
#include <optional>
// Boost
#include <boost/program_options.hpp>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/util/options.hpp"


namespace po = boost::program_options;

namespace robo_trace {

/**
 * The container holding all the information for how the player shoud realize the playback.
 * Priarily inspired by rosbag. Shout outs!
 *  -> https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/player.cpp#L77
 */
class PlayerOptions final : public OptionsContainer {

public:

    typedef std::shared_ptr<PlayerOptions> Ptr;
    typedef std::shared_ptr<PlayerOptions> ConstPtr;

public:

    /**
     *
     */
    PlayerOptions();

    /**
     *
     */
    virtual ~PlayerOptions();

    /**
     *
     */
    virtual void load(const ros::NodeHandle& node_handle) final override;

    /**
     *
     */
    virtual void setup(po::options_description& description) final override;

    /**
     *
     */
    virtual void load(po::variables_map& options) final override;
    
    /**
     *
     */
    virtual void validate() final override;

public:

    /** */
    int m_topic_queue_size;
     /** */
    std::string m_topic_prefix;

    /** */
    std::optional<float> m_time_start;
    /** */
    std::optional<float> m_time_duration;

    /** */
    std::vector<std::string> m_replay_topics;

    /** */
    bool m_wait_for_all_topics_subscribed;
    /** */
    ros::WallDuration m_wait_duration_after_advertise;

    /** */
    std::optional<double> m_publish_clock_frequency;

    /** */
    bool m_all_at_once;

    

}; 

}