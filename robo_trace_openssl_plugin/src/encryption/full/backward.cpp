// Base
#include "robo_trace_openssl_plugin/encryption/full/backward.hpp"
// Std
#include <stdexcept>
 #include <openssl/err.h>

namespace robo_trace::plugin::open_ssl {

FullEncryptionBackwardProcessor::FullEncryptionBackwardProcessor(const FullEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata) 
 : m_configuration(configuration), m_key_manager(key_manager) {

    /*
        Load the message template.
    */

    std::string message_type = metadata->getString("message_type");
    m_message_description = message_description_provider->getMessageDescription(message_type);

    if (m_message_description == nullptr) {
        throw std::runtime_error("Failed finding message description.");
    }

    /*
        Initialize EVP related members.
    */

    m_decryption_context = EVP_CIPHER_CTX_new();

    if (m_decryption_context == nullptr) {
        throw std::runtime_error("Failed instantiating EVP context.");
    }

    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    
    if (m_encryption_method == nullptr) {
        throw std::runtime_error("Failed resolving target cipher.");
    }
    
    /*
        Setup the IV vector.
    */

    int iv_length = EVP_CIPHER_iv_length(m_encryption_method);
    m_iv.resize(iv_length);

    /*
        Load  the enrcyption key.
    */

    evp_pkey_ctx_ptr pkey_context(EVP_PKEY_CTX_new(m_key_manager->getAuthorityPrivateKey(), nullptr));
    
    if (EVP_PKEY_decrypt_init(pkey_context.get()) <= 0) {
        throw std::runtime_error("Failed initializing pkey encryption context.");
    }

    if (EVP_PKEY_CTX_set_rsa_padding(pkey_context.get(), RSA_PKCS1_OAEP_PADDING) <= 0) {
        throw std::runtime_error("Failed seeting RSA padding.");
    }

    size_t encrypted_key_size;
    unsigned char* encrypted_key = (unsigned char*) metadata->getBinData("key", encrypted_key_size); 

    size_t decrypted_key_size;

    if (EVP_PKEY_decrypt(pkey_context.get(), NULL, &decrypted_key_size, encrypted_key, encrypted_key_size) <= 0) {
        throw std::runtime_error("Failed fetching decrypted key size.");
    }
 
    m_key.resize(decrypted_key_size);
    
    if (EVP_PKEY_decrypt(pkey_context.get(), &m_key[0], &decrypted_key_size, encrypted_key, encrypted_key_size) <= 0) {
        throw std::runtime_error("Failed decrypting key.");
    }

    std::string s(m_key.begin(), m_key.end());
   
}

FullEncryptionBackwardProcessor::~FullEncryptionBackwardProcessor() {
    EVP_CIPHER_CTX_free(m_decryption_context);
}


robo_trace::processing::Mode FullEncryptionBackwardProcessor::getMode() const {
    return robo_trace::processing::Mode::REPLAY;
}

void FullEncryptionBackwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {
    
    const std::optional<bsoncxx::document::view>& o_serialized = context->getBsonMessage();

    if (!o_serialized) {
        throw std::runtime_error("Serialized message not present.");
    }

    const bsoncxx::document::view& serialized = o_serialized.value();
    bsoncxx::document::view::const_iterator matches = serialized.find("encrypted");

    if (matches == serialized.end()) {
        throw std::runtime_error("Failed decrypting message! Could not find message.");
    }
 
    size_t iv_length = 0;
    // Trusting that encryption IV size matches IV size of decrypt. 
    const unsigned char* iv_data = (const unsigned char*) context->getMetadata()->getBinData("iv", iv_length); 

    if(!EVP_DecryptInit_ex(m_decryption_context, m_encryption_method, NULL, &m_key[0], iv_data)) {
        throw std::runtime_error("Failed to initialize EVP context!");
    }

    const bsoncxx::types::b_binary wrapper = (*matches).get_binary();
    const size_t encrypted_legth = wrapper.size;
    const unsigned char* encrypted_data = (const unsigned char*) wrapper.bytes;

    ros_babel_fish::BabelFishMessage::Ptr message_wrapper = boost::make_shared<ros_babel_fish::BabelFishMessage>();
    message_wrapper->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");    
    message_wrapper->allocate(encrypted_legth);

    int plain_text_length = 0;

    if(!EVP_DecryptUpdate(m_decryption_context, message_wrapper->buffer(), &plain_text_length, encrypted_data, encrypted_legth)) {
        throw std::runtime_error("Failed feeding message for decryption.");
    }

    int length = 0;

    // Usually does not add anything to the cipher text, but might i.e. for padding.
    if(!EVP_DecryptFinal_ex(m_decryption_context, message_wrapper->buffer() + plain_text_length, &length)) {
        throw std::runtime_error("Failed finalizing decryption.");
    }
  
    // assert(encrypted_legth <= plain_text_length + length, "Plain text may not be longer than cipher text.");
    // As we'll allocate less memory than with the first allocate invocation, the buffer is not reallocated.
    message_wrapper->allocate(plain_text_length + length);
    context->setRosMessage(message_wrapper);

}


}