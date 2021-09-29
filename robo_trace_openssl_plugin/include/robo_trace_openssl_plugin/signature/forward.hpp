#pragma once

// Std
#include <vector>
// OpenSSL
#include <openssl/evp.h>
// Project
#include "robo_trace/processing/stage/stage.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/signature/configuration.hpp"


namespace robo_trace {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class OpenSSLSignatureForwardStage final : public ProcessingStage {


public:

    /**
     * TODO
     */
    OpenSSLSignatureForwardStage(const OpenSSLSignatureStageConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager);

    /**
     * TODO
     */
    virtual ~OpenSSLSignatureForwardStage();
  
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const OpenSSLSignatureStageConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;

    /** */
    const EVP_MD* m_signing_hashing_method;
    /** */
    EVP_MD_CTX* m_signing_context;

    /** */
    std::vector<unsigned char> m_signature_buffer;

};

} 