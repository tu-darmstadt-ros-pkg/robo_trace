// Base
#include "robo_trace_openssl_plugins/encryption/full/forward.hpp"
// OpenSSL
#include <openssl/rand.h>
// MongoDB
#include "robo_trace_plugin_interface/config.h"
#include <mongo/bson/bsonobjbuilder.h>

#include <ros/console.h>

namespace robo_trace {

 
OpenSSLFullEncryptionProcessingStage::OpenSSLFullEncryptionProcessingStage(const OpenSSLFullEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager) 
: ProcessingStage(ProcessingStage::Mode::FORWARD, "openssl_full_encryption"), m_configuration(configuration), m_key_manager(key_manager) {

    /*
        Initialize EVP related members.
    */

    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    m_encryption_context = EVP_CIPHER_CTX_new();

    /*
        Create random symetric encryption key.
    */
    
    // 256bit key. Maybe use less if to much overhead.
    m_key.resize(32);
    
    if (!RAND_bytes((unsigned char*) &m_key[0], 32)) {
        // TODO: Error
        return;
    }

    // TODO: Write back Key for decryption.

    /*
        Setup the IV vector.
    */

    // For AES 256 CBC the IV size is 16 bytes (128 bits).
    // TODO: Make this dynamic
    m_iv.resize(16);

}
  
OpenSSLFullEncryptionProcessingStage::~OpenSSLFullEncryptionProcessingStage() {
    EVP_CIPHER_CTX_free(m_encryption_context);
};


const OpenSSLFullEncryptionConfiguration::Ptr OpenSSLFullEncryptionProcessingStage::getConfiguration() const {
    return m_configuration;
}

const OpenSSLPluginKeyManager::Ptr OpenSSLFullEncryptionProcessingStage::getKeyManager() const {
    return m_key_manager;
}


void OpenSSLFullEncryptionProcessingStage::process(MessageProcessingContext::Ptr& context) {

    if (!RAND_bytes((unsigned char*) &m_iv[0], 16)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Could not sample random IV for message encryption.");
        return;
    }

    context->getMetadata()->append("iv", m_iv);

    // Currently only non authenticated methods. GCM and CCM probably not relevant anyways.
    if(!EVP_EncryptInit_ex(m_encryption_context, EVP_aes_256_cbc(), NULL, (unsigned char*) &m_key[0], (unsigned char*) &m_iv[0])) {
        // TODO: Error
        return;
    }

    const int block_size = EVP_CIPHER_CTX_block_size(m_encryption_context);
    const int stream_length = context->getMessage()->getStreamLength();
    // The ciper text may be longer than the original text due to possible padding.
    const int cipher_max_legth = stream_length + block_size;

    // No way arround allocating a new chunk on the heap here. :/
    std::unique_ptr<uint8_t[]> cipher_text(static_cast<uint8_t*>(malloc(cipher_max_legth)));
    int cipher_text_length = 0;

    if(!EVP_EncryptUpdate(m_encryption_context, cipher_text.get(), &cipher_text_length, context->getMessage()->getStreamData(), stream_length)) {
        // TODO: Error
        return;
    }

    int length = 0;
    // Usually does not add anything to the cipher text, but might i.e. for padding.
    if(!EVP_EncryptFinal_ex(m_encryption_context, cipher_text.get() + cipher_text_length, &length)) {
        // TODO: Error
        return;
    }

    mongo::BSONObjBuilder builder;
    builder.appendBinData("encrypted", cipher_max_legth, mongo::BinDataType::BinDataGeneral, cipher_text.get());
    context->getMessage()->setSerialized(builder.obj());

    // Update stream. This will also set the messages status to 'StreamModified'.
    // TODO: Flag messages as non decodable.
    // context->getMessage()->setStream(std::move(cipher_text), cipher_text_length + length);
    
}


}