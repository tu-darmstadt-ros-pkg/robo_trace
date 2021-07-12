// Base
#include "robo_trace_openssl_plugins/chain/validate.hpp"
// Std
#include <algorithm> 
#include <iterator>


namespace robo_trace {

OpenSSLHashChainValidationStage::OpenSSLHashChainValidationStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager) 
: ProcessingStage(ProcessingStage::Mode::VALIDATE, "openssl_hash_chain_validate"), m_configuration(configuration) {
    
    hashing_context = EVP_MD_CTX_new();
    hashing_method = EVP_get_digestbyname(m_configuration->getHashingMehtodName().c_str());

    if (hashing_context == nullptr) {
        return;
    }

    if(!EVP_DigestInit_ex(hashing_context, hashing_method, nullptr)) {
        return;
    }

    const std::string& public_key = key_manager->getRecorderPublicKey();

    if(!EVP_DigestUpdate(hashing_context, public_key.c_str(), public_key.length())) {
        return;
    }   

    hash_buffer_length = 0;

    if(!EVP_DigestFinal(hashing_context, hash_buffer, &hash_buffer_length)) {
        return;
    }

}

 
OpenSSLHashChainValidationStage::~OpenSSLHashChainValidationStage() {
    // 
}

 
const OpenSSLHashChainConfiguration::Ptr& OpenSSLHashChainValidationStage::getConfiguration() const {
    return m_configuration;
}


void OpenSSLHashChainValidationStage::process(MessageProcessingContext::Ptr& context) {

    if (hashing_context == nullptr) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Hashing context not allocated!");
        return;
    }

    if (!EVP_DigestInit_ex(hashing_context, hashing_method, nullptr)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed to initialize hashing context!");
        return;
    }

    // Feed in the current message
    if (!EVP_DigestUpdate(hashing_context, context->getMessage()->getStreamData(), context->getMessage()->getStreamLength())) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed hashing in current message!");
        return;
    }  

    // Feed in the hash of the previous message
    if (!EVP_DigestUpdate(hashing_context, hash_buffer, hash_buffer_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed hashing in previous message hash!");
        return;
    }

    hash_buffer_length = 0;

    if (!EVP_DigestFinal(hashing_context, hash_buffer, &hash_buffer_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed finalizing hash!");
        return;
    }

    size_t expected_hash_length = 0;
    const uint8_t* expected_hash_data = (const uint8_t*) context->getMetadata()->getBinData(m_configuration->getHashingResultStorageKey(), expected_hash_length); 

    if (hash_buffer_length != expected_hash_length) {
        context->setStatus(MessageProcessingContext::Status::INVALID, "Message hash length missmatch.");
        return;
    }

    for (size_t idx = 0; idx < expected_hash_length; ++idx) {
        if (expected_hash_data[idx] != hash_buffer[idx]) {
            context->setStatus(MessageProcessingContext::Status::INVALID, "Message hash missmatch.");
            break;
        }
    }
    

}


}