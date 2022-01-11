// Base
#include "robo_trace_openssl_plugin/encryption/partial/descriptor.hpp"
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/encryption/partial/forward.hpp"
#include "robo_trace_openssl_plugin/encryption/partial/backward.hpp"
 

namespace robo_trace::plugin::open_ssl {

PartialEncryptionModuleDescriptor::PartialEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace)
: PartialEncryptionModuleDescriptor(key_manager, stages_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

PartialEncryptionModuleDescriptor::PartialEncryptionModuleDescriptor(const KeyManager::Ptr& key_manager, const ros::NodeHandle& stages_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: robo_trace::processing::Descriptor(stages_namespace, "openssl_partial_encryption"), m_key_manager(key_manager), m_message_description_provider(message_description_provider) {
    
    /*
        Create and load basic parameters of the configuration.
    */

    m_configuration = std::make_shared<PartialEncryptionModuleConfiguration>();

    m_configuration->setEnryptionMethod(m_handle.param<std::string>(ROS_PARAM_ENCRYPTION_METHOD_NAME, ROS_PARAM_ENCRYPTION_METHOD_DEFAULT));
    
    /*
        Load the encryption tree.
    */

    std::string delimiter = ".";

    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>& encryption_target_tree = m_configuration->getEncryptionTargetsTree();

    XmlRpc::XmlRpcValue target_message_type_list;
    m_handle.getParam("encryption_targets", target_message_type_list);

    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it_message_type = target_message_type_list.begin(); it_message_type != target_message_type_list.end(); ++it_message_type) { 

        const std::string& target_message_type = it_message_type->first;
        const XmlRpc::XmlRpcValue& target_encryption_fields =  it_message_type->second;
        
        if (target_encryption_fields.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) {
             // TODO: Unexpected format.
            continue;
        }

        if (target_encryption_fields.size() == 0) {
            // TODO: No targets specified.
            continue;
        }
        
        const PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr& target_tree_root = std::make_shared<PartialEncryptionModuleConfiguration::EncryptionTarget>();
        
        std::pair<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr> target_tree_root_record(target_message_type, target_tree_root);
        encryption_target_tree.insert(target_tree_root_record);

        // No need to copy here
        // const OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr& target_tree_root = encryption_target_tree.emplace(target_message_type).first->second;
       
        for(size_t idx = 0; idx < target_encryption_fields.size(); ++idx) { 
            
            const XmlRpc::XmlRpcValue& xml_rpc_value = target_encryption_fields[idx];

            if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::Type::TypeString) {
                // TODO: Unexpected type.
                continue;
            }
            
            // Expected to be "key1.key2.key3"
            std::string enryption_target = static_cast<std::string>(xml_rpc_value);
            std::vector<std::string> path;
            size_t pos = 0;
            
            while ((pos = enryption_target.find(delimiter)) != std::string::npos) {
                path.push_back(enryption_target.substr(0, pos));
                enryption_target.erase(0, pos + delimiter.length());
            }
            path.push_back(enryption_target);
            
            PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr current_tree_node = target_tree_root;

            for (size_t target_idx = 0; target_idx < path.size(); target_idx++) {
                
                const std::string& current_element = path[target_idx]; 
               
                // Last level
                if (target_idx + 1 == path.size()) {
                    current_tree_node->targets.push_back(current_element);
                // Nest down
                } else {

                    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator target = current_tree_node->children.find(current_element);

                    if (target == current_tree_node->children.end()) {
                        
                        //  // current_tree_node = encryption_targets.emplace(current_element)->first->second;
                        PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr new_tree_node = std::make_shared<PartialEncryptionModuleConfiguration::EncryptionTarget>();
        
                        std::pair<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr> new_tree_node_record(current_element, new_tree_node);
                        current_tree_node->children.insert(new_tree_node_record);

                        current_tree_node = new_tree_node;

                    } else {
                        current_tree_node = target->second; 
                    }

                }
            }


        }
    }

    
}

PartialEncryptionModuleDescriptor::~PartialEncryptionModuleDescriptor() = default;

bool PartialEncryptionModuleDescriptor::isModeSupported(const robo_trace::processing::Mode mode) const {
    return mode == robo_trace::processing::Mode::CAPTURE || mode == robo_trace::processing::Mode::REPLAY;
}

std::optional<robo_trace::processing::Processor::Ptr> PartialEncryptionModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& summary, const robo_trace::processing::Mode mode) {
    switch(mode) {

        case robo_trace::processing::Mode::CAPTURE : 
            return std::make_shared<PartialEncryptionForwardProcessor>(m_configuration, m_key_manager, m_message_description_provider, summary);

        case robo_trace::processing::Mode::REPLAY : 
            return std::make_shared<PartialEncryptionBackwardProcessor>(m_configuration, m_key_manager, m_message_description_provider, summary);
        
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
    
}


}