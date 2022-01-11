#pragma once

// Std
#include <memory>
// Boost
#include <boost/regex.hpp>
#include <boost/program_options.hpp>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/util/options.hpp"


namespace po = boost::program_options;

namespace robo_trace::capture {

class Options final : public robo_trace::util::Options {

public:

    typedef std::shared_ptr<Options> Ptr;
    typedef std::shared_ptr<const Options> ConstPtr;

public:

    /**
     *
     */
    Options();

    /**
     *
     */
    ~Options(); 

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
    bool m_capture_all;

    /** */
    bool m_capture_node;
    /** */
    std::string m_capture_node_name;
    
    /** */
    bool m_capture_topics_by_regex;
    /** */
    std::vector<std::string> m_capture_topics;
    
    /** */
    bool m_capture_exclude_by_regex;
    /** */
    boost::regex m_capture_exclude_regex;

    /** */
    double m_capture_topic_check_period;

    /** */
    int m_capture_limit_per_topic;
    /** */
    ros::Duration m_capture_duration;
    
    /** */
    int m_capture_subscriber_queue_size;
    /** */
    ros::TransportHints m_capture_subscriber_transport_hints;
   
};

}