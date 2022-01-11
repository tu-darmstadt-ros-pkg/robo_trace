#pragma once
// Std
#include <memory>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

class HashChainValidationProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * 
     */
    HashChainValidationProcessor(const HashChainModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& metadata);

    /**
     * 
     */
    virtual ~HashChainValidationProcessor();

    /**
     *
     */
    virtual robo_trace::processing::Mode getMode() const final override;

    /**
     * 
     */
    virtual void process(const robo_trace::processing::Context::Ptr& context) final override;

private:

    /** */
    const HashChainModuleConfiguration::Ptr m_configuration;
  
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