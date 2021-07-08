#pragma once

// OpenSSL
#include <openssl/evp.h>
#include <openssl/rsa.h>
#include <openssl/aes.h>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/signature/configuration.hpp"


namespace robo_trace {

/**
 * Implements a signature stage over the hash of the message to
 * be processed.
 * 
 * This is stage must be executed in a blocking fashion! Only one 
 */
class OpenSSLSignatureProcessingStage final : public ProcessingStage {


public:

    /**
     * TODO
     */
    OpenSSLSignatureProcessingStage(const OpenSSLSignatureStageConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager);

    /**
     * TODO
     */
    virtual ~OpenSSLSignatureProcessingStage();

    /**
     * 
     */
    const OpenSSLSignatureStageConfiguration::Ptr getConfiguration() const;

    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr getKeyManager() const;

    /**
     * TODO
     */
    virtual void process(MessageProcessingContext::Ptr &context) final override;

private:

    /** */
    const OpenSSLSignatureStageConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;

    /** */
    const EVP_MD* m_signing_hashing_method;
    /** */
    EVP_MD_CTX* m_signing_context;

};

} 