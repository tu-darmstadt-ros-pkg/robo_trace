#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/signature/configuration.hpp"


namespace robo_trace { 

class OpenSSLSignatureStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    OpenSSLSignatureStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    virtual ~OpenSSLSignatureStageDescriptor();


    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingStage::Mode mode) const final override;

    /**
     * 
     */
    virtual bool isExclusiveToTopic(const ProcessingStage::Mode mode) const final override;

    /**
     * 
     */
    virtual bool isConcurrentlyExecutable(const ProcessingStage::Mode mode) const final override;


    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr& getKeyManager() const;

    /**
     * 
     */
    const OpenSSLSignatureStageConfiguration::Ptr& getConfiguration() const;
    
    
    /**
     * TODO
     */
    virtual ProcessingStage::Ptr getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) final override;

  
protected:

    /** */
    OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    OpenSSLSignatureStageConfiguration::Ptr m_configuration;

};

}