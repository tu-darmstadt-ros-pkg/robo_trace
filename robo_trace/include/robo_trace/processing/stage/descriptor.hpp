#pragma once

// Std
#include <memory>
#include <string>
#include <optional>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/modes/modes.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class ProcessingStageDescriptor {

public:

    typedef std::shared_ptr<ProcessingStageDescriptor> Ptr;
    typedef std::shared_ptr<const ProcessingStageDescriptor> ConstPtr;

public:

    /**
     * TODO
     */
    ProcessingStageDescriptor(const ros::NodeHandle& stage_namespace, const std::string name);

    /**
     * TODO
     */
    virtual ~ProcessingStageDescriptor();

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
    ros::NodeHandle& getHandle();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const = 0;
   
    /**
     * Provides a pointer to a processing stage that is to be used for 
     * processing messages belonging to the provided topic.
     * 
     * @param type 
     * @param topic the topic the processing stage will process messages from.
     * 
     * @return a shared pointer to the processing stage.
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) = 0;

protected:

    /** */
    const std::string m_name;

    /** */
    ros::NodeHandle m_handle;

};

}