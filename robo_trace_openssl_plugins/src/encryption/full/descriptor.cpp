// Base
#include "robo_trace_openssl_plugins/encryption/full/descriptor.hpp"
// Std
#include <memory>
#include <exception>
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace_openssl_plugins/definitions.hpp"
#include "robo_trace_openssl_plugins/encryption/full/forward.hpp"
#include "robo_trace_openssl_plugins/encryption/full/backward.hpp"


namespace robo_trace {

OpenSSLFullEncryptionStageDescriptor::OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace)
: OpenSSLFullEncryptionStageDescriptor(key_manager, plugin_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

OpenSSLFullEncryptionStageDescriptor::OpenSSLFullEncryptionStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: ProcessingStageDescriptor(plugin_namespace, "openssl_full_encryption"), m_key_manager(key_manager), m_message_description_provider(message_description_provider), 
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


bool OpenSSLFullEncryptionStageDescriptor::isModeSupported(const ProcessingStage::Mode mode) const {
    // Currently only FORWARD is implemented.
    return mode == ProcessingStage::Mode::FORWARD;
}

bool OpenSSLFullEncryptionStageDescriptor::isExclusiveToTopic(const ProcessingStage::Mode mode) const {
    return true;
}

bool OpenSSLFullEncryptionStageDescriptor::isConcurrentlyExecutable(const ProcessingStage::Mode mode) const {
    return false;
}


const OpenSSLPluginKeyManager::Ptr& OpenSSLFullEncryptionStageDescriptor::getKeyManager() const {
    return m_key_manager;
}

const OpenSSLFullEncryptionConfiguration::Ptr& OpenSSLFullEncryptionStageDescriptor::getConfiguration() const {
    return m_configuration;
}


ProcessingStage::Ptr OpenSSLFullEncryptionStageDescriptor::getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) {
    switch(mode) {

        case ProcessingStage::Mode::FORWARD : 
            return std::make_shared<OpenSSLFullEncryptionForwardStage>(m_configuration, m_key_manager);
        
        case ProcessingStage::Mode::BACKWARD :
            return std::make_shared<OpenSSLFullEncryptionBackwardStage>(m_configuration, m_key_manager, m_message_description_provider->getMessageDescription(message_type));
    
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}