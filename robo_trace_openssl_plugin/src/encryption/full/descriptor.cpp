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


namespace robo_trace {

OpenSSLFullEncryptionStageDescriptor::OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace)
: OpenSSLFullEncryptionStageDescriptor(key_manager, stages_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

OpenSSLFullEncryptionStageDescriptor::OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: ProcessingStageDescriptor(stages_namespace, "openssl_full_encryption"), m_key_manager(key_manager), m_message_description_provider(message_description_provider), 
  m_configuration(std::make_shared<OpenSSLFullEncryptionConfiguration>()) {
    
    /*
        Load the configuration.
    */

    m_configuration->setEnryptionMethod(m_handle.param<std::string>(
        ROS_PARAM_ENCRYPTION_METHOD_NAME, 
        ROS_PARAM_ENCRYPTION_METHOD_DEFAULT)
    );
    
}

OpenSSLFullEncryptionStageDescriptor::~OpenSSLFullEncryptionStageDescriptor() = default;


bool OpenSSLFullEncryptionStageDescriptor::isModeSupported(const ProcessingMode mode) const {
    // Currently only FORWARD is implemented.
    return mode == ProcessingMode::CAPTURE || mode == ProcessingMode::REPLAY;
}

std::optional<ProcessingStage::Ptr> OpenSSLFullEncryptionStageDescriptor::getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) {
    switch(mode) {

        case ProcessingMode::CAPTURE : 
            return std::make_shared<OpenSSLFullEncryptionForwardStage>(m_configuration, m_key_manager, chain_metadata);
        
        case ProcessingMode::REPLAY :
            return std::make_shared<OpenSSLFullEncryptionBackwardStage>(m_configuration, m_key_manager, m_message_description_provider, chain_metadata);
    
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}