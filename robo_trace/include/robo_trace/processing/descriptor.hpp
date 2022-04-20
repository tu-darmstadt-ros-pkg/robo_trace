/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// Std
#include <memory>
#include <string>
#include <optional>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/mode.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class Descriptor {

public:

    typedef std::shared_ptr<Descriptor> Ptr;
    typedef std::shared_ptr<const Descriptor> ConstPtr;

public:

    /**
     * TODO
     */
    Descriptor(const ros::NodeHandle& stage_namespace, const std::string& name);

    /**
     * TODO
     */
    virtual ~Descriptor();

    /**
     * Provides the name of the underlying processing stage. This name should be
     * short and informative. For example: 'open_ssh_full_encryption'
     * 
     * @return the name of the underlying processing stage.
     */
    const std::string& getName() const;

    /**
     * Provides a ROS node handle registered within a unique namespace for the
     * underlying processing stages. This may be used for fetching configuration
     * parameters.
     * 
     * @return the ROS node handle for the underlying processing stage.
     */
    ros::NodeHandle& getNodeHandle();

    /**
     * 
     */
    virtual bool isModeSupported(const Mode mode) const = 0;
   
    /**
     * Provides a pointer to a processing stage that is to be used for 
     * processing messages belonging to the provided topic.
     * 
     * @param type 
     * @param topic the topic the processing stage will process messages from.
     * 
     * @return a shared pointer to the processing stage.
     */
    virtual std::optional<Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& summary, const Mode mode) = 0;

protected:

    /** */
    const std::string m_name;

    /** */
    ros::NodeHandle m_handle;

};

}