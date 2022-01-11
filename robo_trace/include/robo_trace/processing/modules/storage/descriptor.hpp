#pragma once

// Std
#include <string>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

class StorageModuleDescriptor final : public Descriptor {

public:

    /**
     * 
     */
    StorageModuleDescriptor(const robo_trace::store::Options::ConstPtr& options, const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    virtual ~StorageModuleDescriptor();

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
    const robo_trace::store::Options::ConstPtr m_connector_options;

};

}