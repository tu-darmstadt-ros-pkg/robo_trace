#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/encryption/full/configuration.hpp"


namespace robo_trace { 

class OpenSSLFullEncryptionStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     *
     */
    OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~OpenSSLFullEncryptionStageDescriptor();


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
    const OpenSSLFullEncryptionConfiguration::Ptr& getConfiguration() const;
    
    /**
     * TODO
     */
    virtual ProcessingStage::Ptr getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) final override;

  
protected:

    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    const OpenSSLFullEncryptionConfiguration::Ptr m_configuration;

};  


}   