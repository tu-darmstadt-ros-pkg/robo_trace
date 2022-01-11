// Base
#include "robo_trace_openssl_plugin/signature/descriptor.hpp"
// Std
#include <memory>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/signature/forward.hpp"


namespace robo_trace::plugin::open_ssl {

/**
 * TODO: Pass in private key
 */
SignatureModuleDescriptor::SignatureModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace) 
: robo_trace::processing::Descriptor(stages_namespace, "openssl_signature"), m_key_manager(key_manager) {
  
    /*
        Create and load the configuration.
    */

    m_configuration = std::make_shared<SignatureModuleConfiguration>();


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

SignatureModuleDescriptor::~SignatureModuleDescriptor() = default;

bool SignatureModuleDescriptor::isModeSupported(const robo_trace::processing::Mode mode) const {
    // Currently only FORWARD is implemented.
    return mode == robo_trace::processing::Mode::CAPTURE;
}

std::optional<robo_trace::processing::Processor::Ptr> SignatureModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& summary, const robo_trace::processing::Mode mode) {
    switch(mode) {

        case robo_trace::processing::Mode::CAPTURE : 
            return std::make_shared<SignatureForwardProcessor>(m_configuration, m_key_manager);
        
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}


}