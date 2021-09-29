// Base
#include "robo_trace_openssl_plugin/chain/descriptor.hpp"
// Std
#include <memory>
#include <exception>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/chain/forward.hpp"
#include "robo_trace_openssl_plugin/chain/validate.hpp"


namespace robo_trace {


OpenSSLHashChainStageDescriptor::OpenSSLHashChainStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace) 
: ProcessingStageDescriptor(stages_namespace, "openssl_hash_chain"), m_key_manager(key_manager) {
    
    /*
        Create and load the configuration.
    */

    m_configuration = std::make_shared<OpenSSLHashChainConfiguration>();

    m_configuration->setHashingMethodName(m_handle.param<std::string>(
        ROS_PARAM_HASH_CHAIN_METHOD_NAME, 
        ROS_PARAM_HASH_CHAIN_METHOD_DEFAULT)
    );
    
    m_configuration->setHashingResultStorageKey(m_handle.param<std::string>(
        ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_NAME, 
        ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_DEFAULT)
    );

}

OpenSSLHashChainStageDescriptor::~OpenSSLHashChainStageDescriptor() = default;

bool OpenSSLHashChainStageDescriptor::isModeSupported(const ProcessingMode mode) const {
    // Currently only FORWARD is implemented.
    return ProcessingMode::CAPTURE == mode && ProcessingMode::VALIDATE == mode;
}

std::optional<ProcessingStage::Ptr> OpenSSLHashChainStageDescriptor::getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) {
    switch(mode) {

        case ProcessingMode::CAPTURE : 
            return std::make_shared<OpenSSLHashChainForwardStage>(m_configuration, m_key_manager, chain_metadata);
        
        case ProcessingMode::VALIDATE:
            return std::make_shared<OpenSSLHashChainValidationStage>(m_configuration, m_key_manager, chain_metadata);

        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}