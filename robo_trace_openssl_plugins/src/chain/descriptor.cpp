// Base
#include "robo_trace_openssl_plugins/chain/descriptor.hpp"
// Std
#include <memory>
#include <exception>
// Project
#include "robo_trace_openssl_plugins/definitions.hpp"
#include "robo_trace_openssl_plugins/chain/forward.hpp"


namespace robo_trace {


OpenSSLHashChainStageDescriptor::OpenSSLHashChainStageDescriptor(const OpenSSLPluginKeyManager::Ptr& key_manager, const ros::NodeHandle& plugin_namespace) 
: ProcessingStageDescriptor(plugin_namespace, "openssl_hash_chain"), m_key_manager(key_manager) {
    
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


bool OpenSSLHashChainStageDescriptor::isModeSupported(const ProcessingStage::Mode mode) const {
    // Currently only FORWARD is implemented.
    return mode == ProcessingStage::Mode::FORWARD;
}

bool OpenSSLHashChainStageDescriptor::isExclusiveToTopic(const ProcessingStage::Mode mode) const {
    return true;
}

bool OpenSSLHashChainStageDescriptor::isConcurrentlyExecutable(const ProcessingStage::Mode mode) const {
    return false;
}


const OpenSSLPluginKeyManager::Ptr& OpenSSLHashChainStageDescriptor::getKeyManager() const {
    return m_key_manager;
}

const OpenSSLHashChainConfiguration::Ptr& OpenSSLHashChainStageDescriptor::getConfiguration() const {
    return m_configuration;
}


ProcessingStage::Ptr OpenSSLHashChainStageDescriptor::getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) {
    switch(mode) {

        case ProcessingStage::Mode::FORWARD : 
            return std::make_shared<OpenSSLHashChainProcessingStage>(m_configuration, m_key_manager);
        
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}