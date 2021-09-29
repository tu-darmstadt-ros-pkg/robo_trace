#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace { 

class OpenSSLFullEncryptionStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     *
     */
    OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~OpenSSLFullEncryptionStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const final override;

    /**
     * TODO
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) final override;


protected:

    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    const OpenSSLFullEncryptionConfiguration::Ptr m_configuration;

};  


}   