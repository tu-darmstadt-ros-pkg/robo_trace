#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/signature/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class SignatureForwardProcessor final : public robo_trace::processing::Processor {


public:

    /**
     * TODO
     */
    SignatureForwardProcessor(const SignatureModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager);

    /**
     * TODO
     */
    virtual ~SignatureForwardProcessor();
  
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
    const SignatureModuleConfiguration::Ptr m_configuration;
    /** */
    const KeyManager::Ptr m_key_manager;

    /** */
    const EVP_MD* m_signing_hashing_method;
    /** */
    EVP_MD_CTX* m_signing_context;

    /** */
    std::vector<unsigned char> m_signature_buffer;

};

} 