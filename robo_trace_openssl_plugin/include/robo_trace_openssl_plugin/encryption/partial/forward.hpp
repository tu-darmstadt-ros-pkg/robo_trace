#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include <robo_trace/processing/stage/stage.hpp>
#include <robo_trace_openssl_plugin/key_manager.hpp>
#include <robo_trace_openssl_plugin/encryption/partial/configuration.hpp>


namespace robo_trace {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class OpenSSLPartialEncryptionForwardStage final : public ProcessingStage {


public:

    /**
     * TODO
     */
    OpenSSLPartialEncryptionForwardStage(const OpenSSLPartialEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const DataContainer::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~OpenSSLPartialEncryptionForwardStage();

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
    template<bool DoSerialization>
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree);

    template<bool DoSerialization>
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree);


private:

    /** */
    const OpenSSLPartialEncryptionConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    
    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_msg_template;
    /** */
    OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr m_encryption_tree;

    /** */
    std::vector<unsigned char> m_key;
    /** */
    std::vector<unsigned char> m_iv;

    /** */
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_encryption_context; 

    /** */
    std::vector<unsigned char> m_cipher_buffer;
    
};

} 