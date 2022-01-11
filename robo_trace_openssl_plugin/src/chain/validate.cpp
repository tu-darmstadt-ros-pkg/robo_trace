// Base
#include "robo_trace_openssl_plugin/chain/validate.hpp"
// Std
#include <algorithm> 
#include <iterator>
#include <stdexcept>
// Ros
#include <ros/console.h>


namespace robo_trace::plugin::open_ssl {

HashChainValidationProcessor::HashChainValidationProcessor(const HashChainModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& data) 
: m_configuration(configuration) {
    
    m_hashing_context = EVP_MD_CTX_new();
    
    if (m_hashing_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    m_hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());

    if(!EVP_DigestInit_ex(m_hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Could not initialize EVP context.");
    }

    const std::string& public_key = key_manager->getRecorderPublicKeyString();

    if(!EVP_DigestUpdate(m_hashing_context, public_key.c_str(), public_key.length())) {
        throw std::runtime_error("Failed feeding in the public key for the initial hash.");
    }   

    m_hash_buffer_length = 0;

    if(!EVP_DigestFinal(m_hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing the initial hash.");
    }

    size_t expected_hash_length = 0;
    const uint8_t* expected_hash_data = (const uint8_t*) data->getBinData("initial_hash", expected_hash_length); 

    if (m_hash_buffer_length != expected_hash_length) {
        throw std::runtime_error("Initial hash length missmatch.");
    }

    for (size_t idx = 0; idx < expected_hash_length; ++idx) {
        if (expected_hash_data[idx] != m_hash_buffer[idx]) {
            throw std::runtime_error("Initial hash missmatch!");
        }
    }

}

HashChainValidationProcessor::~HashChainValidationProcessor() {
    // Can not be a nullptr as constructor terminated with except if so.
    EVP_MD_CTX_free(m_hashing_context);
}

robo_trace::processing::Mode HashChainValidationProcessor::getMode() const {
    return robo_trace::processing::Mode::VALIDATE;
}

void HashChainValidationProcessor::process(const robo_trace::processing::Context::Ptr& context) {

    if (!EVP_DigestInit_ex(m_hashing_context, m_hashing_method, nullptr)) {
        throw std::runtime_error("Failed to initialize hashing context!");
    }

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getUnserializedMessage(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* const message_stream = o_message_stream.value();


    // Feed in the current message
    if (!EVP_DigestUpdate(m_hashing_context, message_stream, message_stream_length)) {
        throw std::runtime_error("Failed hashing in current message!");
    }  

    // Feed in the hash of the previous message
    if (!EVP_DigestUpdate(m_hashing_context, m_hash_buffer, m_hash_buffer_length)) {
        throw std::runtime_error("Failed hashing in previous message hash!");
    }

    m_hash_buffer_length = 0;

    if (!EVP_DigestFinal(m_hashing_context, m_hash_buffer, &m_hash_buffer_length)) {
        throw std::runtime_error("Failed finalizing hash!");
    }

    size_t expected_hash_length = 0;
    const uint8_t* expected_hash_data = (const uint8_t*) context->getMetadata()->getBinData(m_configuration->getHashingResultStorageKey(), expected_hash_length); 

    // TODO:
    if (m_hash_buffer_length != expected_hash_length) {
        //context->setStatus(MessageProcessingContext::Status::INVALID, "Message hash missmatch.");
    } else {

        for (size_t idx = 0; idx < expected_hash_length; ++idx) {
            if (expected_hash_data[idx] != m_hash_buffer[idx]) {
                //context->setStatus(MessageProcessingContext::Status::INVALID, "Message hash missmatch.");
                break;
            }
        }
        
    }
    

}


}