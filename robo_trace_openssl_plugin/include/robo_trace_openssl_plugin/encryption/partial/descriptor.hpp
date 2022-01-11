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
#include <robo_trace/processing/descriptor.hpp>
#include <robo_trace_openssl_plugin/key_manager.hpp>
#include <robo_trace_openssl_plugin/encryption/partial/configuration.hpp>


namespace robo_trace::plugin::open_ssl { 

class PartialEncryptionModuleDescriptor final : public robo_trace::processing::Descriptor {

public:

    /**
     * TODO
     */
    PartialEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     *
     */
    PartialEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~PartialEncryptionModuleDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const robo_trace::processing::Mode mode) const final override;

    /**
     * 
     */
    virtual std::optional<robo_trace::processing::Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& chain_metadata, const robo_trace::processing::Mode mode) final override;

protected:

    /** */
    const KeyManager::Ptr m_key_manager;
     /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    PartialEncryptionModuleConfiguration::Ptr m_configuration;


};

}   