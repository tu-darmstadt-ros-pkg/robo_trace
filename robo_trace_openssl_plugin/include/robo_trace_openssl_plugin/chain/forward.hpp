#pragma once

// Project
#include "robo_trace_openssl_plugin/parameters.hpp"
// Std
#include <memory>
#ifdef MODULE_HASH_CHAIN_FORWARD_SYNCHRONIZE
#include <mutex>
#endif
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

/**
 * Implements a hash chaining stage based on the SHA256 algorithm.
 * The hash for some message is generated by piping the concatenation
 * of the previouses message hash with the current message into the
 * hashing algorithm. 
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class HashChainForwardProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * 
     */
    HashChainForwardProcessor(const HashChainModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& metadata);

    /**
     * 
     */
    virtual ~HashChainForwardProcessor();

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

#ifdef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    /** */
    EVP_MD_CTX* m_hashing_context;
#endif

#ifdef MODULE_HASH_CHAIN_FORWARD_SYNCHRONIZE
    /** */
    std::mutex m_hashing_mutex;
#endif

#ifdef MODULE_HASH_CHAIN_SEQUENCE_NUMBER_ENABLE
    int64_t m_sequence_number;
#endif

    /** */
    unsigned char m_hash_buffer[EVP_MAX_MD_SIZE];
    /** */
    unsigned int m_hash_buffer_length = 0;

};

} 