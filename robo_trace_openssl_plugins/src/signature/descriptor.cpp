// Base
#include "robo_trace_openssl_plugins/signature/descriptor.hpp"
// Std
#include <memory>
// Project
#include "robo_trace_openssl_plugins/definitions.hpp"
#include "robo_trace_openssl_plugins/signature/forward.hpp"


namespace robo_trace {

/**
 * TODO: Pass in private key
 */
OpenSSLSignatureStageDescriptor::OpenSSLSignatureStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace) 
: ProcessingStageDescriptor(plugin_namespace, "openssl_signature"), m_key_manager(key_manager) {
  
    /*
        Create and load the configuration.
    */

    m_configuration = std::make_shared<OpenSSLSignatureStageConfiguration>();


    m_configuration->setHashingMethodName(m_handle.param<std::string>(
        ROS_PARAM_SIGNATURE_HASH_METHOD_NAME, 
        ROS_PARAM_SIGNATURE_HASH_METHOD_DEFAULT
    ));
    m_configuration->setInputStorageKey(m_handle.param<std::string>(
        ROS_PARAM_SIGNATURE_INPUT_STORAGE_KEY_NAME, 
        ROS_PARAM_SIGNATURE_INPUT_STORAGE_KEY_DEFAULT
    ));
    m_configuration->setResultStorageKey(m_handle.param<std::string>(
        ROS_PARAM_SIGNATURE_RESULT_STORAGE_KEY_NAME,
        ROS_PARAM_SIGNATURE_RESULT_STORAGE_KEY_DEFAULT
    ));
    
}

OpenSSLSignatureStageDescriptor::~OpenSSLSignatureStageDescriptor() = default;


bool OpenSSLSignatureStageDescriptor::isModeSupported(const ProcessingStage::Mode mode) const {
    // Currently only FORWARD is implemented.
    return mode == ProcessingStage::Mode::FORWARD;
}

bool OpenSSLSignatureStageDescriptor::isExclusiveToTopic(const ProcessingStage::Mode mode) const {
    return true;
}

bool OpenSSLSignatureStageDescriptor::isConcurrentlyExecutable(const ProcessingStage::Mode mode) const {
    return false;
}


const OpenSSLPluginKeyManager::Ptr& OpenSSLSignatureStageDescriptor::getKeyManager() const {
    return m_key_manager;
}

const OpenSSLSignatureStageConfiguration::Ptr& OpenSSLSignatureStageDescriptor::getConfiguration() const {
    return m_configuration;
}


ProcessingStage::Ptr OpenSSLSignatureStageDescriptor::getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) {
    switch(mode) {

        case ProcessingStage::Mode::FORWARD : 
            return std::make_shared<OpenSSLSignatureProcessingStage>(m_configuration, m_key_manager);
        
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}


}