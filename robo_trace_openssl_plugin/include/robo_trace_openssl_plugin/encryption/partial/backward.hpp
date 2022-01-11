#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// MongoCXX
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/document/element.hpp>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
#include <ros_babel_fish/messages/compound_message.h>
// Project
#include <robo_trace/processing/processor.hpp>
#include <robo_trace_openssl_plugin/key_manager.hpp>
#include <robo_trace_openssl_plugin/encryption/partial/configuration.hpp>


namespace robo_trace::plugin::open_ssl {

class PartialEncryptionBackwardProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * TODO
     */
    PartialEncryptionBackwardProcessor(const PartialEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~PartialEncryptionBackwardProcessor();

    /**
     *
     */
    virtual robo_trace::processing::Mode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const robo_trace::processing::Context::Ptr& context) final override;

private:

    /**
     *
     */
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree);

    /**
     *
     */
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, const bsoncxx::document::element& serialized_element, ros_babel_fish::CompoundMessage& deserialized, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree);

private:

    /** */
    const PartialEncryptionModuleConfiguration::Ptr m_configuration;
    /** */
    const KeyManager::Ptr m_key_manager;
    
    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;
    /** */
    PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr m_encryption_tree;

    /** */
    std::vector<unsigned char> m_key;
    /** */
    std::vector<unsigned char> m_iv;

    /** */
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_decryption_context; 

    /** */
    std::vector<unsigned char> m_decryption_buffer;
    
};

}