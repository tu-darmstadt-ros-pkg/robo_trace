#pragma once

// OpenSSL
#include <openssl/evp.h>
#include <openssl/rsa.h>
#include <openssl/aes.h>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/encryption/partial/configuration.hpp"


namespace robo_trace {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class OpenSSLPartialEncryptionProcessingStage final : public ProcessingStage {


public:

    /**
     * TODO
     */
    OpenSSLPartialEncryptionProcessingStage(const OpenSSLPartialEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const std::string& data_type);

    /**
     * TODO
     */
    virtual ~OpenSSLPartialEncryptionProcessingStage();

    /**
     * 
     */
    const OpenSSLPartialEncryptionConfiguration::Ptr getConfiguration() const;

    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr getKeyManager() const;

    /**
     * TODO
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    /**
     *
     */
    template<bool DoSerialization>
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr encryption_tree);

private:

    /** */
    const OpenSSLPartialEncryptionConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;

    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_msg_template;
    /** */
    OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr m_encryption_tree;

    // No idea who's the owner of that pointer tbh.
    /** */
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_encryption_context;

    /** */
    std::string m_key;
    /** */
    std::string m_iv;

};

} 