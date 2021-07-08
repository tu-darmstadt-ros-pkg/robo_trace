// Base
#include "robo_trace_openssl_plugins/signature/forward.hpp"
// Std
#include <sstream>
#include <iomanip>
#include <string_view>
// OpenSSL
#include <openssl/err.h>
#include <openssl/pem.h>

namespace robo_trace {


OpenSSLSignatureProcessingStage::OpenSSLSignatureProcessingStage(const OpenSSLSignatureStageConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager) 
: ProcessingStage(ProcessingStage::Mode::FORWARD, "openssl_signature"), m_configuration(configuration), m_key_manager(key_manager) {

    m_signing_hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());
    m_signing_context = EVP_MD_CTX_new();
}

OpenSSLSignatureProcessingStage::~OpenSSLSignatureProcessingStage() {
    EVP_MD_CTX_free(m_signing_context);
}


const OpenSSLSignatureStageConfiguration::Ptr OpenSSLSignatureProcessingStage::getConfiguration() const {
    return m_configuration;
}

const OpenSSLPluginKeyManager::Ptr OpenSSLSignatureProcessingStage::getKeyManager() const {
    return m_key_manager;
}


void OpenSSLSignatureProcessingStage::process(MessageProcessingContext::Ptr &context) {
    /*
        
        A good overview on signatures with OpenSSL can be found in 
        the official documentation:
         -> https://wiki.openssl.org/index.php/EVP_Signing_and_Verifying

    */

    if (!EVP_DigestSignInit(m_signing_context, NULL, m_signing_hashing_method, NULL, m_key_manager->getRecorderKey())) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed initializing signature context!");
        return;
    }

    size_t message_hash_length;
    const char* message_hash = (const char*) context->getMetadata()->getBinData(m_configuration->getInputStorageKey(), message_hash_length);
    //std::string message_hash = context->getMetadata()->getString(m_configuration->getInputStorageKey());

    // Feed without NULL termination!
    // if (!EVP_DigestSignUpdate(m_signing_context, message_hash.c_str(), message_hash.length())) {
    if (!EVP_DigestSignUpdate(m_signing_context, message_hash, message_hash_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed feeding in message hash!");
        return;
    }

    size_t m_signature_buffer_length = 0;

    if (!EVP_DigestSignFinal(m_signing_context, NULL, &m_signature_buffer_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed retrieving final signature size!");
        return;
    }

    unsigned char* m_signature_buffer = (unsigned char*) OPENSSL_malloc(m_signature_buffer_length);

    if (!EVP_DigestSignFinal(m_signing_context, m_signature_buffer, &m_signature_buffer_length)) {
        OPENSSL_free(m_signature_buffer);
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed finalizing signature creation!");
        return;
    }

    context->getMetadata()->append(m_configuration->getResultStorageKey(), m_signature_buffer, m_signature_buffer_length);

    /*
    std::stringstream ss;

    for(unsigned int i = 0; i < m_signature_buffer_length; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int) m_signature_buffer[i];
    }

    // TODO: The storage field should be configurable. 
    context->getMetadata()->append(m_configuration->getResultStorageKey(), ss.str());
    */
    OPENSSL_free(m_signature_buffer);

}

}