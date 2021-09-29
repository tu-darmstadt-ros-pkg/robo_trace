#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
#include <ros_babel_fish/messages/compound_message.h>
// Project
#include <robo_trace/processing/stage/stage.hpp>
#include <robo_trace_openssl_plugin/key_manager.hpp>
#include <robo_trace_openssl_plugin/encryption/partial/configuration.hpp>


namespace robo_trace {

class OpenSSLPartialEncryptionBackwardStage final : public ProcessingStage {

public:

    /**
     * TODO
     */
    OpenSSLPartialEncryptionBackwardStage(const OpenSSLPartialEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const DataContainer::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~OpenSSLPartialEncryptionBackwardStage();

    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /**
     *
     */
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const mongo::BSONObj& serialized, ros_babel_fish::CompoundMessage& deserialized, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree);

    /**
     *
     */
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, const mongo::BSONElement& serialized_element, ros_babel_fish::CompoundMessage& deserialized, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree);

private:

    /** */
    const OpenSSLPartialEncryptionConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    
    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;
    /** */
    OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr m_encryption_tree;

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