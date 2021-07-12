// Base
#include "robo_trace_openssl_plugins/chain/forward.hpp"
// Std
#include <sstream>
#include <iomanip>
// Ros
#include "ros/console.h"


namespace robo_trace {

OpenSSLHashChainForwardStage::OpenSSLHashChainForwardStage(const OpenSSLHashChainConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager) 
: ProcessingStage(ProcessingStage::Mode::FORWARD, "openssl_hash_chain"), m_configuration(configuration) {
 
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

OpenSSLHashChainForwardStage::~OpenSSLHashChainForwardStage() {
    EVP_MD_CTX_free(hashing_context);
}

const OpenSSLHashChainConfiguration::Ptr& OpenSSLHashChainForwardStage::getConfiguration() const {
    return m_configuration;
}

void OpenSSLHashChainForwardStage::process(MessageProcessingContext::Ptr& context) {
    /*
        
        The official documentation on OpenSSL's EVP can be found here:
          -> https://www.openssl.org/docs/manmaster/man3/EVP_DigestInit.html
        
        This implementation follows partially an example from StackOverflow:
          -> https://stackoverflow.com/a/40155962
        Shout outs!

    */

    if (hashing_context == nullptr) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Hashing context not allocated!");
        return;
    }

    if(!EVP_DigestInit_ex(hashing_context, hashing_method, nullptr)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed to initialize hashing context!");
        return;
    }

    // Feed in the current message
    if(!EVP_DigestUpdate(hashing_context, context->getMessage()->getSerializedStreamData(), context->getMessage()->getSerializedStreamLength())) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed hashing in current message!");
        return;
    }  

    // Feed in the hash of the previous message
    if(!EVP_DigestUpdate(hashing_context, hash_buffer, hash_buffer_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed hashing in previous message hash!");
        return;
    }

    hash_buffer_length = 0;

    if(!EVP_DigestFinal(hashing_context, hash_buffer, &hash_buffer_length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed finalizing hash!");
        return;
    }

    context->getMetadata()->append(m_configuration->getHashingResultStorageKey(), hash_buffer, hash_buffer_length);

    /*
    std::stringstream ss;
    
    for(unsigned int i = 0; i < hash_buffer_length; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int) hash_buffer[i];
    }

    // TODO: The storage field should be configurable. 
    context->getMetadata()->append(m_configuration->getHashingResultStorageKey(), ss.str());
    */
}

}