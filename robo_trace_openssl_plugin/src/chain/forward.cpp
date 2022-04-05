// Base
#include "robo_trace_openssl_plugin/chain/forward.hpp"
// Std
#include <cassert>
#include <stdexcept>
// Ros
#include <ros/console.h>


#define STAGE_LOGGER_NAME "robo_trace_openssl_chain_forward"


namespace robo_trace::plugin::open_ssl {

HashChainForwardProcessor::HashChainForwardProcessor(const HashChainModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& metadata) 
: m_configuration(configuration) {

#ifdef MODULE_HASH_CHAIN_SEQUENCE_NUMBER_ENABLE
    m_sequence_number = 0;
#endif

    EVP_MD_CTX* hashing_context = EVP_MD_CTX_new();
    
    if (hashing_context == nullptr) {
        throw std::runtime_error("Failed instantiating new EVP context.");
    }

#ifdef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    m_hashing_context = hashing_context;
#endif

    m_hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());
    
    if(!EVP_DigestInit_ex(hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Could not initialize EVP context.");
    }

    const std::string& public_key = key_manager->getRecorderPublicKeyString();

    if(!EVP_DigestUpdate(hashing_context, public_key.c_str(), public_key.length())) {
        throw std::runtime_error("Failed feeding in the public key for initial hash.");
    }   

    m_hash_buffer_length = 0;

    if(!EVP_DigestFinal(hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing initial hash.");
    }

#ifndef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    EVP_MD_CTX_free(hashing_context);
#endif

    // We'll persist the initial hash (i.e. the hash of the public key).
    metadata->append("initial_hash", m_hash_buffer, m_hash_buffer_length);

}

HashChainForwardProcessor::~HashChainForwardProcessor() {
#ifdef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    EVP_MD_CTX_free(m_hashing_context);
#endif
}


robo_trace::processing::Mode HashChainForwardProcessor::getMode() const {
    return robo_trace::processing::Mode::CAPTURE;
}

void HashChainForwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {
    /*
        
        The official documentation on OpenSSL's EVP can be found here:
          -> https://www.openssl.org/docs/manmaster/man3/EVP_DigestInit.html
        
        This implementation follows partially an example from StackOverflow:
          -> https://stackoverflow.com/a/40155962
        Shout outs!

    */

#ifdef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    EVP_MD_CTX* hashing_context = m_hashing_context;

// We have only a single context and must restrict concurrent access. 
#ifdef MODULE_HASH_CHAIN_FORWARD_SYNCHRONIZE
    std::lock_guard<std::mutex> guard(m_hashing_mutex);
#endif
    
#else

    EVP_MD_CTX* hashing_context = EVP_MD_CTX_new();
    
    if (hashing_context == nullptr) {
        throw std::runtime_error("Failed instantiating new EVP context.");
    }

#endif

    if(!EVP_DigestInit_ex(hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Failed to initialize hashing context!");
    }

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getRosMessageStream(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* const message_stream = o_message_stream.value();

    // Feed in the current message
    if(!EVP_DigestUpdate(hashing_context, message_stream, message_stream_length)) {
        throw std::runtime_error("Failed hashing in current message!");
    }  

#if defined(MODULE_HASH_CHAIN_FORWARD_SYNCHRONIZE) and !defined(MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT)
    std::lock_guard<std::mutex> guard(m_hashing_mutex);
#endif

    // Feed in the hash of the previous message
    if(!EVP_DigestUpdate(hashing_context, m_hash_buffer, m_hash_buffer_length)) {
        throw std::runtime_error("Failed hashing in previous message hash!");
    }

    m_hash_buffer_length = 0;

    if(!EVP_DigestFinal(hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing hash!");
    }

    context->getMetadata()->append(m_configuration->getHashingResultStorageKey(), m_hash_buffer, m_hash_buffer_length);

#ifdef MODULE_HASH_CHAIN_SEQUENCE_NUMBER_ENABLE
    context->getMetadata()->append(MODULE_HASH_CHAIN_SEQUENCE_NUMBER_KEY, m_sequence_number++);
#endif

#ifndef MODULE_HASH_CHAIN_FORWARD_REUSE_CONTEXT
    EVP_MD_CTX_free(hashing_context);
#endif

}

}