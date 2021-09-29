#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace { 

class OpenSSLHashChainStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * TODO
     */
    OpenSSLHashChainStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     * TODO
     */
    virtual ~OpenSSLHashChainStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const final override;

    /**
     * 
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) final override;

  
protected:

    /** */
    OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    OpenSSLHashChainConfiguration::Ptr m_configuration;
    
};

}   