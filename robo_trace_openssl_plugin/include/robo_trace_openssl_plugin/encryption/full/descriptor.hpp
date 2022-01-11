#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace::plugin::open_ssl { 

class FullEncryptionModuleDescriptor final : public robo_trace::processing::Descriptor {

public:

    /**
     *
     */
    FullEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    FullEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * TODO
     */
    virtual ~FullEncryptionModuleDescriptor();

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
    const KeyManager::Ptr m_key_manager;
    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

    /** */
    const FullEncryptionModuleConfiguration::Ptr m_configuration;

};  


}   