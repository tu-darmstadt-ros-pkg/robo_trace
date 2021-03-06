#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

/**
 * Implements a hash chaining stage based on the SHA256 algorithm.
 * The hash for some message is generated by piping the concatenation
 * of the previouses message hash with the current message into the
 * hashing algorithm. 
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class FullEncryptionForwardProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * TODO
     */
    FullEncryptionForwardProcessor(const FullEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~FullEncryptionForwardProcessor();
  
    /**
     *
     */
    virtual robo_trace::processing::Mode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const robo_trace::processing::Context::Ptr& context) final override;

private:

    /** */
    const FullEncryptionModuleConfiguration::Ptr m_configuration;
    /** */
    const KeyManager::Ptr m_key_manager;

    /** */
    std::vector<unsigned char> m_key;
    /** */
    std::vector<unsigned char> m_iv;

    // No idea who's the owner of that pointer tbh.
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_encryption_context;

    /** */
    std::vector<unsigned char> m_cipher_buffer;

};

} 