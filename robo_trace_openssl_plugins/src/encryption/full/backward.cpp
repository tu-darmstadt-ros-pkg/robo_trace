// Base
#include "robo_trace_openssl_plugins/encryption/full/backward.hpp"


namespace robo_trace {

OpenSSLFullEncryptionBackwardStage::OpenSSLFullEncryptionBackwardStage(const OpenSSLFullEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::MessageDescription::ConstPtr& message_description) 
 : ProcessingStage(ProcessingStage::Mode::BACKWARD, "openssl_full_encryption_backward"), m_configuration(configuration), m_key_manager(key_manager), m_message_description(message_description) {


    /*
        Initialize EVP related members.
    */

    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    m_decryption_context = EVP_CIPHER_CTX_new();

 
    /*
        TODO: Load  the enrcyption key.
    */

    /*
        Setup the IV vector.
    */

    // For AES 256 CBC the IV size is 16 bytes (128 bits).
    // TODO: Make this dynamic
    m_iv.resize(16);

}

OpenSSLFullEncryptionBackwardStage::~OpenSSLFullEncryptionBackwardStage() {
    //
}

const OpenSSLFullEncryptionConfiguration::Ptr OpenSSLFullEncryptionBackwardStage::getConfiguration() const {
    return m_configuration;
}

const OpenSSLPluginKeyManager::Ptr OpenSSLFullEncryptionBackwardStage::getKeyManager() const {
    return m_key_manager;
}

void OpenSSLFullEncryptionBackwardStage::process(MessageProcessingContext::Ptr& context) {

    size_t iv_length = 0;
    const unsigned char* iv_data = (const unsigned char*) context->getMetadata()->getBinData("iv", iv_length); 

    const mongo::BSONObj& serialized = context->getMessage()->getSerialized();

    if (!serialized.hasField("encrypted")) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Message is not encrypted.");
        return;
    }

    // Currently only non authenticated methods. GCM and CCM probably not relevant anyways.
    if(!EVP_DecryptInit_ex(m_decryption_context, EVP_aes_256_cbc(), NULL, (unsigned char*) &m_key[0], (unsigned char*) &m_iv[0])) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Failed initializing decryption context with key and iv.");
        return;
    }

    int encrypted_legth;
    const unsigned char* encrypted_data = (const unsigned char*) serialized["encrypted"].binData(encrypted_legth);

    ros_babel_fish::BabelFishMessage::Ptr message = boost::make_shared<ros_babel_fish::BabelFishMessage>(); 
    message->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");    
    message->allocate(encrypted_legth);

    int plain_text_length = 0;

    if(!EVP_DecryptUpdate(m_decryption_context, message->buffer(), &plain_text_length, encrypted_data, encrypted_legth)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Feeding in message for decryption failed.");
        return;
    }

    int length = 0;

    // Usually does not add anything to the cipher text, but might i.e. for padding.
    if(!EVP_DecryptFinal_ex(m_decryption_context, message->buffer() + plain_text_length, &length)) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Finalizing decryption failed.");
        return;
    }
  
    // assert(encrypted_legth <= plain_text_length + length, "Plain text may not be longer than cipher text.");
    // As we'll allocate less memory than with the first allocate invocation, the buffer is not reallocated.
    message->allocate(plain_text_length + length);

}


}