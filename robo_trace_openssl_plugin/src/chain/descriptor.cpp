// Base
#include "robo_trace_openssl_plugin/chain/descriptor.hpp"
// Std
#include <memory>
#include <exception>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/chain/forward.hpp"
#include "robo_trace_openssl_plugin/chain/validate.hpp"


namespace robo_trace::plugin::open_ssl {

HashChainStageModuleDescriptor::HashChainStageModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace) 
: robo_trace::processing::Descriptor(stages_namespace, "openssl_hash_chain"), m_key_manager(key_manager) {
    
    /*
        Create and load the configuration.
    */

    m_configuration = std::make_shared<HashChainModuleConfiguration>();

    m_configuration->setHashingMethodName(m_handle.param<std::string>(
        ROS_PARAM_HASH_CHAIN_METHOD_NAME, 
        ROS_PARAM_HASH_CHAIN_METHOD_DEFAULT)
    );
    
    m_configuration->setHashingResultStorageKey(m_handle.param<std::string>(
        ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_NAME, 
        ROS_PARAM_HASH_CHAIN_RESULT_STORAGE_KEY_DEFAULT)
    );

}

HashChainStageModuleDescriptor::~HashChainStageModuleDescriptor() = default;

bool HashChainStageModuleDescriptor::isModeSupported(const robo_trace::processing::Mode mode) const {
    return robo_trace::processing::Mode::CAPTURE == mode || robo_trace::processing::Mode::VALIDATE == mode;
}

std::optional<robo_trace::processing::Processor::Ptr> HashChainStageModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& chain_metadata, const robo_trace::processing::Mode mode) {
    switch(mode) {

        case robo_trace::processing::Mode::CAPTURE : 
            return std::make_shared<HashChainForwardProcessor>(m_configuration, m_key_manager, chain_metadata);
        
        case robo_trace::processing::Mode::VALIDATE:
            return std::make_shared<HashChainValidationProcessor>(m_configuration, m_key_manager, chain_metadata);

        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}

}