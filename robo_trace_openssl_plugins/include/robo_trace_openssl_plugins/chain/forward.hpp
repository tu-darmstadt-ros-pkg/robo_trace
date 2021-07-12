#pragma once

// Std
#include <memory>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/chain/configuration.hpp"


namespace robo_trace {

/**
 * Implements a hash chaining stage based on the SHA256 algorithm.
 * The hash for some message is generated by piping the concatenation
 * of the previouses message hash with the current message into the
 * hashing algorithm. 
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class OpenSSLHashChainForwardStage final : public ProcessingStage {

public:

    /**
     * TODO
     */
    OpenSSLHashChainForwardStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager);

    /**
     * TODO
     */
    virtual ~OpenSSLHashChainForwardStage();

    /**
     * 
     */
    const OpenSSLHashChainConfiguration::Ptr& getConfiguration() const;

    /**
     * TODO
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    const OpenSSLHashChainConfiguration::Ptr m_configuration;

    const EVP_MD* hashing_method;
    EVP_MD_CTX* hashing_context;
    
    unsigned char hash_buffer[EVP_MAX_MD_SIZE];
    unsigned int hash_buffer_length = 0;

};

} 