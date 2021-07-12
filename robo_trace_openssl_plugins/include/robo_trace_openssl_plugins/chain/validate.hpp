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

class OpenSSLHashChainValidationStage final : public ProcessingStage {

public:

    /**
     * 
     */
    OpenSSLHashChainValidationStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager);

    /**
     * 
     */
    virtual ~OpenSSLHashChainValidationStage();

    /**
     * 
     */
    const OpenSSLHashChainConfiguration::Ptr& getConfiguration() const;

    /**
     * 
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    /** */
    const OpenSSLHashChainConfiguration::Ptr m_configuration;

    /** */
    const EVP_MD* hashing_method;
    /** */
    EVP_MD_CTX* hashing_context;
    
    /** */
    unsigned char hash_buffer[EVP_MAX_MD_SIZE];
    /** */
    unsigned int hash_buffer_length = 0;

}; 

}