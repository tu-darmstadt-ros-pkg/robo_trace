#pragma once

// Std
#include <vector>
// Boost
#include "boost/program_options.hpp"
// Ros
#include <ros/ros.h> 


namespace po = boost::program_options;

namespace robo_trace {

class OptionsContainer {

public:

    typedef std::shared_ptr<OptionsContainer> Ptr;
    typedef std::shared_ptr<const OptionsContainer> ConstPtr;

public:

    static void load(const std::vector<OptionsContainer::Ptr>& containers, ros::NodeHandle& node_handle, int argc, char** argv);

public:

    /**
     *
     */
    OptionsContainer();

    /**
     *
     */
    virtual ~OptionsContainer(); 


    /**
     *
     */
    virtual void load(const ros::NodeHandle& node_handle) = 0;

    /**
     *
     */
    virtual void setup(po::options_description& description) = 0;

    /**
     *
     */
    virtual void load(po::variables_map& options) = 0;

    /**
     *
     */
    virtual void validate();

};

}