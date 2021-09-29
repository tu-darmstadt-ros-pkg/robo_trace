// Base
#include "robo_trace_openssl_plugin/signature/descriptor.hpp"
// Std
#include <memory>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/signature/forward.hpp"


namespace robo_trace {

/**
 * TODO: Pass in private key
 */
OpenSSLSignatureStageDescriptor::OpenSSLSignatureStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace) 
: ProcessingStageDescriptor(stages_namespace, "openssl_signature"), m_key_manager(key_manager) {
  
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

bool OpenSSLSignatureStageDescriptor::isModeSupported(const ProcessingMode mode) const {
    // Currently only FORWARD is implemented.
    return mode == ProcessingMode::CAPTURE;
}

std::optional<ProcessingStage::Ptr> OpenSSLSignatureStageDescriptor::getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) {
    switch(mode) {

        case ProcessingMode::CAPTURE : 
            return std::make_shared<OpenSSLSignatureForwardStage>(m_configuration, m_key_manager);
        
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}


}