#pragma once

// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

class BasicMarshallingModuleDescriptor final : public Descriptor {

public:

    /**
     * 
     */
    BasicMarshallingModuleDescriptor(const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    BasicMarshallingModuleDescriptor(const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * 
     */
    virtual ~BasicMarshallingModuleDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const Mode mode) const final override;

    /**
     * 
     */
    virtual std::optional<Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& chain_metadata, const Mode mode) final override;

private:

    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

};

}