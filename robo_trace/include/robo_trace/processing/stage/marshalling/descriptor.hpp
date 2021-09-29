#pragma once

// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/stage/descriptor.hpp"


namespace robo_trace {

class BasicMessageMarshallingStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * 
     */
    virtual ~BasicMessageMarshallingStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const final override;

    /**
     * 
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) final override;

private:

    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

};

}