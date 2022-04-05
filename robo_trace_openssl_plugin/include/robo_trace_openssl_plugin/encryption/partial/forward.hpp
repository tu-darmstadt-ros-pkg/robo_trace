#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// MongoCXX
#include <bsoncxx/builder/basic/document.hpp>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/storage/stream.hpp"
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/parameters.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/partial/configuration.hpp"

#define COMMA ,

#ifdef MODULE_PARTIAL_ENCRYPTION_OFFLOAD_ENCRYPTED_BLOBS
#define MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(insert) insert
#else
#define MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(insert)
#endif

namespace robo_trace::plugin::open_ssl {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class PartialEncryptionForwardProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * TODO
     */
    PartialEncryptionForwardProcessor(const PartialEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~PartialEncryptionForwardProcessor();

    /**
     *
     */
    virtual robo_trace::processing::Mode getMode() const final override;
  
    /**
     * 
     */
    virtual void process(const robo_trace::processing::Context::Ptr& context) final override;

private:

    /**
     * 
     */
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA const robo_trace::store::StreamHandler::Ptr& stream_handler));

    /**
     * 
     */
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree MODULE_PARTIAL_ENCRYPTION_INSERT_ON_OFFLOADING(COMMA const robo_trace::store::StreamHandler::Ptr& stream_handler));

private:

    /** */
    const PartialEncryptionModuleConfiguration::Ptr m_configuration;
    /** */
    const KeyManager::Ptr m_key_manager;
    
    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_msg_template;
    /** */
    PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr m_encryption_tree;

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