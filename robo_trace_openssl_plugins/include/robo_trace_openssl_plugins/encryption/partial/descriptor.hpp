#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/encryption/partial/configuration.hpp"


namespace robo_trace { 

class OpenSSLPartialEncryptionStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * TODO
     */
    OpenSSLPartialEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace);

    /**
     *
     */
    OpenSSLPartialEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~OpenSSLPartialEncryptionStageDescriptor();


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
    const OpenSSLPartialEncryptionConfiguration::Ptr& getConfiguration() const;
    
    /**
     * 
     */
    virtual ProcessingStage::Ptr getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) final override;

protected:

    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
     /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    OpenSSLPartialEncryptionConfiguration::Ptr m_configuration;


};

}   