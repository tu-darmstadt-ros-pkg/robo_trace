// Base
#include "robo_trace_openssl_plugin/chain/forward.hpp"
// Std
#include <cassert>
#include <stdexcept>
// Ros
#include <ros/console.h>


#define STAGE_LOGGER_NAME "robo_trace_openssl_chain_forward"


namespace robo_trace {

OpenSSLHashChainForwardStage::OpenSSLHashChainForwardStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const DataContainer::Ptr& metadata) 
: m_configuration(configuration) {
    
    m_hashing_context = EVP_MD_CTX_new();

    if (m_hashing_context == nullptr) {
        throw std::runtime_error("Failed instantiating new EVP context.");
    }

    m_hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());
    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Using hashing method: " << m_configuration->getHashingMehtodName());

    if(!EVP_DigestInit_ex(m_hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Could not initialize EVP context.");
    }

    const std::string& public_key = key_manager->getRecorderPublicKeyString();

    if(!EVP_DigestUpdate(m_hashing_context, public_key.c_str(), public_key.length())) {
        throw std::runtime_error("Failed feeding in the public key for initial hash.");
    }   

    m_hash_buffer_length = 0;

    if(!EVP_DigestFinal(m_hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing initial hash.");
    }

    // We'll persist the initial hash (i.e. the hash of the public key).
    metadata->append("initial_hash", m_hash_buffer, m_hash_buffer_length);

}

OpenSSLHashChainForwardStage::~OpenSSLHashChainForwardStage() {
    EVP_MD_CTX_free(m_hashing_context);
}


ProcessingMode OpenSSLHashChainForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void OpenSSLHashChainForwardStage::process(const ProcessingContext::Ptr& context) {
    /*
        
        The official documentation on OpenSSL's EVP can be found here:
          -> https://www.openssl.org/docs/manmaster/man3/EVP_DigestInit.html
        
        This implementation follows partially an example from StackOverflow:
          -> https://stackoverflow.com/a/40155962
        Shout outs!

    */

    if(!EVP_DigestInit_ex(m_hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Failed to initialize hashing context!");
    }

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getUnserializedMessage(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* const message_stream = o_message_stream.value();

    // Feed in the current message
    if(!EVP_DigestUpdate(m_hashing_context, message_stream, message_stream_length)) {
        throw std::runtime_error("Failed hashing in current message!");
    }  

    // Feed in the hash of the previous message
    if(!EVP_DigestUpdate(m_hashing_context, m_hash_buffer, m_hash_buffer_length)) {
        throw std::runtime_error("Failed hashing in previous message hash!");
    }

    m_hash_buffer_length = 0;

    if(!EVP_DigestFinal(m_hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing hash!");
    }

    assert(m_hash_buffer_length < EVP_MAX_MD_SIZE);
    context->getMetadata()->append(m_configuration->getHashingResultStorageKey(), m_hash_buffer, m_hash_buffer_length);

}

}