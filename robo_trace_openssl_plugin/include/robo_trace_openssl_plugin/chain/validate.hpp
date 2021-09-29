#pragma once
// Std
#include <memory>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/stage/stage.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace {

class OpenSSLHashChainValidationStage final : public ProcessingStage {

public:

    /**
     * 
     */
    OpenSSLHashChainValidationStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const DataContainer::Ptr& metadata);

    /**
     * 
     */
    virtual ~OpenSSLHashChainValidationStage();

    /**
     *
     */
    virtual ProcessingMode getMode() const final override;

    /**
     * 
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const OpenSSLHashChainConfiguration::Ptr m_configuration;
  
    /** */
    const EVP_MD* m_hashing_method;
    /** */
    EVP_MD_CTX* m_hashing_context;
    
    /** */
    unsigned char m_hash_buffer[EVP_MAX_MD_SIZE];
    /** */
    unsigned int m_hash_buffer_length = 0;

}; 

}