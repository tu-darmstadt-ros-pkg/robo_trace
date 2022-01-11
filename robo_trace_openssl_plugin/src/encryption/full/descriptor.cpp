// Base
#include "robo_trace_openssl_plugin/encryption/full/descriptor.hpp"
// Std
#include <memory>
#include <exception>
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/encryption/full/forward.hpp"
#include "robo_trace_openssl_plugin/encryption/full/backward.hpp"


namespace robo_trace::plugin::open_ssl {

FullEncryptionModuleDescriptor::FullEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace)
: FullEncryptionModuleDescriptor(key_manager, stages_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

FullEncryptionModuleDescriptor::FullEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: robo_trace::processing::Descriptor(stages_namespace, "openssl_full_encryption"), m_key_manager(key_manager), m_message_description_provider(message_description_provider), 
  m_configuration(std::make_shared<FullEncryptionModuleConfiguration>()) {
    
    /*
        Load the configuration.
    */

    m_configuration->setEnryptionMethod(m_handle.param<std::string>(
        ROS_PARAM_ENCRYPTION_METHOD_NAME, 
        ROS_PARAM_ENCRYPTION_METHOD_DEFAULT)
    );
    
}

FullEncryptionModuleDescriptor::~FullEncryptionModuleDescriptor() = default;


bool FullEncryptionModuleDescriptor::isModeSupported(const robo_trace::processing::Mode mode) const {
    // Currently only FORWARD is implemented.
    return mode == robo_trace::processing::Mode::CAPTURE || mode == robo_trace::processing::Mode::REPLAY;
}

std::optional<robo_trace::processing::Processor::Ptr> FullEncryptionModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& chain_metadata, const robo_trace::processing::Mode mode) {
    switch(mode) {

        case robo_trace::processing::Mode::CAPTURE : 
            return std::make_shared<FullEncryptionForwardProcessor>(m_configuration, m_key_manager, chain_metadata);
        
        case robo_trace::processing::Mode::REPLAY :
            return std::make_shared<FullEncryptionBackwardProcessor>(m_configuration, m_key_manager, m_message_description_provider, chain_metadata);
    
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}