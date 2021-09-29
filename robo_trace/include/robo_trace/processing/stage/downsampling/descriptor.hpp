#pragma once

// Std
#include <string>
#include <vector>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/stage/downsampling/configuration.hpp"


namespace robo_trace {

class DownsamplingStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    DownsamplingStageDescriptor(const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    virtual ~DownsamplingStageDescriptor();

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
    std::vector<DownsamplingMode::Ptr> m_downsampling_modes;

};

}