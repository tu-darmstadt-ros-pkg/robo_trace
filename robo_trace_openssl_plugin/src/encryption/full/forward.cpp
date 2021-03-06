// Base
#include "robo_trace_openssl_plugin/encryption/full/forward.hpp"
// Std
#include <stdexcept>
// OpenSSL
#include <openssl/rand.h>
// MongoCXX
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>


namespace robo_trace::plugin::open_ssl {
 
FullEncryptionForwardProcessor::FullEncryptionForwardProcessor(const FullEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const robo_trace::store::Container::Ptr& summary) 
: m_configuration(configuration), m_key_manager(key_manager) {

    /*
        Initialize basic EVP related members.
    */

    m_encryption_context = EVP_CIPHER_CTX_new();

    if (m_encryption_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());

    if (m_encryption_method == nullptr) {
        throw std::runtime_error("Encryption method could not be resolved.");
    }

    /*
        Setup the IV vector.
    */

    int iv_length = EVP_CIPHER_iv_length(m_encryption_method);
    m_iv.resize(iv_length);

    /*
        Create random symetric encryption key.
    */
    
    int key_length = EVP_CIPHER_key_length(m_encryption_method);
    m_key.resize(key_length);
    
    if (!RAND_bytes(&m_key[0], key_length)) {
        throw std::runtime_error("Failed generating encryption key.");
    }
    
    std::string s(m_key.begin(), m_key.end());
 
    /*
        Save the encrypted key to the container.
    */
    
    evp_pkey_ctx_ptr pkey_context(EVP_PKEY_CTX_new(m_key_manager->getAuthorityPublicKey(), nullptr));
    
    if (EVP_PKEY_encrypt_init(pkey_context.get()) <= 0) {
        throw std::runtime_error("Failed initializing pkey encryption context.");
    }

    if (EVP_PKEY_CTX_set_rsa_padding(pkey_context.get(), RSA_PKCS1_OAEP_PADDING) <= 0) {
        throw std::runtime_error("Failed seeting RSA padding.");
    }

    size_t encrypted_key_size;

    if (EVP_PKEY_encrypt(pkey_context.get(), nullptr, &encrypted_key_size, &m_key[0], m_key.size()) <= 0) {
        throw std::runtime_error("Failed fetching encrypted key size.");
    }

    std::unique_ptr<unsigned char[]> encrypted_key((unsigned char*) malloc(encrypted_key_size));
    
    if (EVP_PKEY_encrypt(pkey_context.get(), &encrypted_key[0], &encrypted_key_size, &m_key[0], m_key.size()) <= 0) {
        throw std::runtime_error("Failed encrypting key.");
    }

    summary->append("key", encrypted_key.get(), encrypted_key_size);

}
  
FullEncryptionForwardProcessor::~FullEncryptionForwardProcessor() {
    EVP_CIPHER_CTX_free(m_encryption_context);
}

robo_trace::processing::Mode FullEncryptionForwardProcessor::getMode() const {
    return robo_trace::processing::Mode::CAPTURE;
}

void FullEncryptionForwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {

    if (!RAND_bytes(&m_iv[0], m_iv.size())) {
        throw std::runtime_error("Failed sampling IV!");
    }
    
    context->getMetadata()->append("iv", &m_iv[0], m_iv.size());
    
    if(!EVP_EncryptInit_ex(m_encryption_context, m_encryption_method, NULL, &m_key[0], &m_iv[0])) {
        throw std::runtime_error("Failed to initialize EVP context!");
    }

    size_t message_stream_length = 0;
    const std::optional<const uint8_t* const> o_message_stream = context->getRosMessageStream(message_stream_length);

    if (!o_message_stream) {
        throw std::runtime_error("Unserialized message not present.");
    }

    const uint8_t* const message_stream = o_message_stream.value();

    const int block_size = EVP_CIPHER_CTX_block_size(m_encryption_context);
    // The ciper text may be longer than the original text due to possible padding.
    const int cipher_max_legth = message_stream_length + block_size;

    m_cipher_buffer.resize(cipher_max_legth);
    int cipher_text_length = 0;

    if(!EVP_EncryptUpdate(m_encryption_context, &m_cipher_buffer[0], &cipher_text_length, message_stream, message_stream_length)) {
        throw std::runtime_error("Failed feeding message for encryption.");
    }

    int length = 0;
    // Account for possible padding.
    if(!EVP_EncryptFinal_ex(m_encryption_context, &m_cipher_buffer[0] + cipher_text_length, &length)) {
        throw std::runtime_error("Failed finalizing encryption.");
    }

    bsoncxx::types::b_binary wrapper;
    wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
    wrapper.size = cipher_text_length + length;
    wrapper.bytes = &m_cipher_buffer[0];

    bsoncxx::builder::basic::document builder{};
    builder.append(bsoncxx::builder::basic::kvp("encrypted", wrapper));
    
    context->setBsonMessage(builder.extract());
 
}


}