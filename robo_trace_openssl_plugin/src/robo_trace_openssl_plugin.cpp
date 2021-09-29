// Base
#include "robo_trace_openssl_plugin/robo_trace_openssl_plugin.hpp"
// PluginLib
#include <pluginlib/class_list_macros.h>
// Project
#include "robo_trace_openssl_plugin/chain/descriptor.hpp"
#include "robo_trace_openssl_plugin/signature/descriptor.hpp"
#include "robo_trace_openssl_plugin/encryption/full/descriptor.hpp"
#include "robo_trace_openssl_plugin/encryption/partial/descriptor.hpp"


namespace robo_trace {

RoboTraceOpenSSLPlugin::RoboTraceOpenSSLPlugin() 
: RoboTraceProcessingPlugin("RoboTraceOpenSSLPlugin") {
    //
}

RoboTraceOpenSSLPlugin::~RoboTraceOpenSSLPlugin() = default;

const OpenSSLPluginKeyManager::Ptr& RoboTraceOpenSSLPlugin::getKeyManager() const {
    return m_key_manager;
}

std::vector<ProcessingStageDescriptor::Ptr> RoboTraceOpenSSLPlugin::setup() {

    /*
        Load and generate keys.
    */

    m_key_manager = std::make_shared<OpenSSLPluginKeyManager>(m_plugin_node_handle);

    /*
        Setup processing stages
    */

    std::vector<ProcessingStageDescriptor::Ptr> descriptors;

    std::shared_ptr<OpenSSLHashChainStageDescriptor> descriptor_hash_chain = std::make_shared<OpenSSLHashChainStageDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_hash_chain);
    
    std::shared_ptr<OpenSSLSignatureStageDescriptor> descriptor_signature = std::make_shared<OpenSSLSignatureStageDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_signature);
    
    std::shared_ptr<OpenSSLFullEncryptionStageDescriptor> descriptor_full_encryption = std::make_shared<OpenSSLFullEncryptionStageDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_full_encryption);
    
    std::shared_ptr<OpenSSLPartialEncryptionStageDescriptor> descriptor_partial_encryption =  std::make_shared<OpenSSLPartialEncryptionStageDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_partial_encryption);

    return descriptors;

}

}

PLUGINLIB_EXPORT_CLASS(robo_trace::RoboTraceOpenSSLPlugin, robo_trace::RoboTraceProcessingPlugin)