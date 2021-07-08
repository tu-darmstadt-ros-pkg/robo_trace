// Base
#include "robo_trace_openssl_plugins/robo_trace_openssl_plugins.hpp"
// PluginLib
#include <pluginlib/class_list_macros.h>
// Project
#include "robo_trace_openssl_plugins/chain/descriptor.hpp"
#include "robo_trace_openssl_plugins/signature/descriptor.hpp"
#include "robo_trace_openssl_plugins/encryption/full/descriptor.hpp"
#include "robo_trace_openssl_plugins/encryption/partial/descriptor.hpp"


namespace robo_trace {

RoboTraceOpenSSLPlugins::RoboTraceOpenSSLPlugins() 
: RoboTraceProcessingPlugin("openssl") {
    //
}

RoboTraceOpenSSLPlugins::~RoboTraceOpenSSLPlugins() = default;


const OpenSSLPluginKeyManager::Ptr& RoboTraceOpenSSLPlugins::getKeyManager() const {
    return m_key_manager;
}

const std::vector<ProcessingStageDescriptor::Ptr>& RoboTraceOpenSSLPlugins::getDescriptors() const {
    return m_descriptors;
}


void RoboTraceOpenSSLPlugins::setup() {

    /*
        Load and generate keys.
    */

    m_key_manager = std::make_shared<OpenSSLPluginKeyManager>(m_plugin_node_handle);

    /*
        Setup processing stages
    */

    std::shared_ptr<OpenSSLHashChainStageDescriptor> descriptor_hash_chain = std::make_shared<OpenSSLHashChainStageDescriptor>(m_key_manager, m_plugin_node_handle);
    m_descriptors.push_back(descriptor_hash_chain);

    std::shared_ptr<OpenSSLSignatureStageDescriptor> descriptor_signature = std::make_shared<OpenSSLSignatureStageDescriptor>(m_key_manager, m_plugin_node_handle);
    m_descriptors.push_back(descriptor_signature);

    std::shared_ptr<OpenSSLFullEncryptionStageDescriptor> descriptor_full_encryption = std::make_shared<OpenSSLFullEncryptionStageDescriptor>(m_key_manager, m_plugin_node_handle);
    m_descriptors.push_back(descriptor_full_encryption);

    std::shared_ptr<OpenSSLPartialEncryptionStageDescriptor> descriptor_partial_encryption =  std::make_shared<OpenSSLPartialEncryptionStageDescriptor>(m_key_manager, m_plugin_node_handle);
    m_descriptors.push_back(descriptor_partial_encryption);

}

}

PLUGINLIB_EXPORT_CLASS(robo_trace::RoboTraceOpenSSLPlugins, robo_trace::RoboTraceProcessingPlugin)