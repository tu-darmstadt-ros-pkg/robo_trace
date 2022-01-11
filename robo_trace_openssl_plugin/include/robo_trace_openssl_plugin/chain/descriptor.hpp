#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace::plugin::open_ssl { 

class HashChainStageModuleDescriptor final : public robo_trace::processing::Descriptor {

public:

    /**
     * TODO
     */
    HashChainStageModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     * TODO
     */
    virtual ~HashChainStageModuleDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const robo_trace::processing::Mode mode) const final override;

    /**
     * 
     */
    virtual std::optional<robo_trace::processing::Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& summary, const robo_trace::processing::Mode mode) final override;

  
protected:

    /** */
    KeyManager::Ptr m_key_manager;
    /** */
    HashChainModuleConfiguration::Ptr m_configuration;
    
};

}   