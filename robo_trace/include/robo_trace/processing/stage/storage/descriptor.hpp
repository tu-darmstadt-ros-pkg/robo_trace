#pragma once

// Std
#include <string>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace/storage/options.hpp"


namespace robo_trace {

class StorageStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    StorageStageDescriptor(const ConnectionOptions::ConstPtr& options, const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    virtual ~StorageStageDescriptor();

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
    const std::string m_summary_collection_path; 
    /** */
    const ConnectionOptions::ConstPtr m_connector_options;

};

}