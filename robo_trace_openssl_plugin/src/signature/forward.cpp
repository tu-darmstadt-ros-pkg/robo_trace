// Base
#include "robo_trace_openssl_plugin/signature/forward.hpp"
// Std
#include <memory>
#include <stdexcept>


namespace robo_trace::plugin::open_ssl {

SignatureForwardProcessor::SignatureForwardProcessor(const SignatureModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager) 
: m_configuration(configuration), m_key_manager(key_manager) {

    m_signing_context = EVP_MD_CTX_new();

    if (m_signing_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    m_signing_hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());

    if (m_signing_hashing_method == nullptr) {
        throw std::runtime_error("Failed retrieving signature hashing method.");
    }

}

SignatureForwardProcessor::~SignatureForwardProcessor() {
    EVP_MD_CTX_free(m_signing_context);
}

robo_trace::processing::Mode SignatureForwardProcessor::getMode() const {
    return robo_trace::processing::Mode::CAPTURE;
}

void SignatureForwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {
    /*
        
        A good overview on signatures with OpenSSL can be found in 
        the official documentation:
         -> https://wiki.openssl.org/index.php/EVP_Signing_and_Verifying

    */

    if (!EVP_DigestSignInit(m_signing_context, NULL, m_signing_hashing_method, NULL, m_key_manager->getRecorderKey())) {
        throw std::runtime_error("Failed initializing EVP context.");
    }

    // No copy is done here.
    size_t message_hash_length;
    const char* message_hash = (const char*) context->getMetadata()->getBinData(m_configuration->getInputStorageKey(), message_hash_length);
    
    if (!EVP_DigestSignUpdate(m_signing_context, message_hash, message_hash_length)) {
        throw std::runtime_error("Failed feeding in message hash to signature.");
    }

    size_t signature_buffer_length = 0;

    if (!EVP_DigestSignFinal(m_signing_context, NULL, &signature_buffer_length)) {
        throw std::runtime_error("Failed fetching final signature length.");
    }

    // Adjust buffer to required size.
    m_signature_buffer.resize(signature_buffer_length);
    
    if (!EVP_DigestSignFinal(m_signing_context, &m_signature_buffer[0], &signature_buffer_length)) {
        throw std::runtime_error("Failed finalizing signature creation.");
    }

    context->getMetadata()->append(m_configuration->getResultStorageKey(), &m_signature_buffer[0], signature_buffer_length);

}

}