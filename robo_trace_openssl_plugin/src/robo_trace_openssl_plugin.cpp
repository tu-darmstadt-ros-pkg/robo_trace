// Base
#include "robo_trace_openssl_plugin/robo_trace_openssl_plugin.hpp"
// PluginLib
#include <pluginlib/class_list_macros.h>
// Project
#include "robo_trace_openssl_plugin/chain/descriptor.hpp" 
#include "robo_trace_openssl_plugin/signature/descriptor.hpp"
#include "robo_trace_openssl_plugin/encryption/full/descriptor.hpp" 
#include "robo_trace_openssl_plugin/encryption/partial/descriptor.hpp" 


namespace robo_trace::plugin::open_ssl {

RoboTraceOpenSSLPlugin::RoboTraceOpenSSLPlugin() 
: robo_trace::processing::Plugin("RoboTraceOpenSSLPlugin") {
    //
}

RoboTraceOpenSSLPlugin::~RoboTraceOpenSSLPlugin() = default;

const KeyManager::Ptr& RoboTraceOpenSSLPlugin::getKeyManager() const {
    return m_key_manager;
}

std::vector<robo_trace::processing::Descriptor::Ptr> RoboTraceOpenSSLPlugin::setup() {

    /*
        Load and generate keys.
    */

    m_key_manager = std::make_shared<KeyManager>(m_plugin_node_handle);

    /*
        Setup processing stages
    */

    std::vector<robo_trace::processing::Descriptor::Ptr> descriptors;

    std::shared_ptr<HashChainStageModuleDescriptor> descriptor_hash_chain = std::make_shared<HashChainStageModuleDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_hash_chain);

    std::shared_ptr<SignatureModuleDescriptor> descriptor_signature = std::make_shared<SignatureModuleDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_signature);

    std::shared_ptr<FullEncryptionModuleDescriptor> descriptor_full_encryption = std::make_shared<FullEncryptionModuleDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_full_encryption);
      
    std::shared_ptr<PartialEncryptionModuleDescriptor> descriptor_partial_encryption =  std::make_shared<PartialEncryptionModuleDescriptor>(m_key_manager, m_stage_node_handle);
    descriptors.push_back(descriptor_partial_encryption);

    return descriptors;

}

}

PLUGINLIB_EXPORT_CLASS(robo_trace::plugin::open_ssl::RoboTraceOpenSSLPlugin, robo_trace::processing::Plugin)