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
#include <robo_trace/processing/stage/descriptor.hpp>
#include <robo_trace_openssl_plugin/key_manager.hpp>
#include <robo_trace_openssl_plugin/encryption/partial/configuration.hpp>


namespace robo_trace { 

class OpenSSLPartialEncryptionStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * TODO
     */
    OpenSSLPartialEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     *
     */
    OpenSSLPartialEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~OpenSSLPartialEncryptionStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const final override;

    /**
     * 
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) final override;

protected:

    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
     /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    OpenSSLPartialEncryptionConfiguration::Ptr m_configuration;


};

}   