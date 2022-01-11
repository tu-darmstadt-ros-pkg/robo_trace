#pragma once

// Std
#include <string>
#include <vector>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace/processing/modules/downsampling/configuration.hpp"


namespace robo_trace::processing {

class DownsamplingModuleDescriptor final : public Descriptor {

public:

    /**
     * 
     */
    DownsamplingModuleDescriptor(const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    virtual ~DownsamplingModuleDescriptor();

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
    std::vector<DownsamplingMode::Ptr> m_downsampling_modes;

};

}