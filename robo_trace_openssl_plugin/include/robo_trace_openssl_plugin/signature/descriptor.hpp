#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/signature/configuration.hpp"


namespace robo_trace::plugin::open_ssl { 

class SignatureModuleDescriptor final : public robo_trace::processing::Descriptor {

public:

    /**
     * 
     */
    SignatureModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    virtual ~SignatureModuleDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const robo_trace::processing::Mode mode) const final override;
    
    /**
     * TODO
     */
    virtual std::optional<robo_trace::processing::Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& summary, const robo_trace::processing::Mode mode) final override;

  
protected:

    /** */
    KeyManager::Ptr m_key_manager;
    /** */
    SignatureModuleConfiguration::Ptr m_configuration;

};

}