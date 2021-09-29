// Base
#include "robo_trace_openssl_plugin/key_manager.hpp"
// Std
#include <stdexcept>
// OpenSSL
#include <openssl/err.h>
#include <openssl/rand.h>
#include <openssl/pem.h>
// Ros
#include <ros/console.h>


namespace robo_trace {

OpenSSLPluginKeyManager::OpenSSLPluginKeyManager(const ros::NodeHandle& plugin_namespace) {
    
    /*
        Setup namespace
    */
    m_handle = ros::NodeHandle(plugin_namespace, "openssl_keys");

    /*
        Load authority public key
    */

    if (!m_handle.getParam("authority_public_key", m_authority_public_key_string)) {
        throw std::runtime_error("Failed loading authority public key.");
    }

    bio_ptr bio_authority_public_key(BIO_new_mem_buf(m_authority_public_key_string.c_str(), -1));

    if (!bio_authority_public_key) {
        throw std::runtime_error("Failed digesting authority key to BIO.");
    }

    EVP_PKEY* authority_public_key = nullptr;
    authority_public_key = PEM_read_bio_PUBKEY(bio_authority_public_key.get(), &authority_public_key, nullptr, nullptr);

    if (authority_public_key == nullptr) {
        throw std::runtime_error("Failed reading RSA key.");
    }

    m_authority_public_key.reset(authority_public_key);


    /*
        Load authority private key if available.

        TODO: If we have the private key, we actually no not need to specify the public 
              key, so we could omit that part if we wanted to.

    */

    if (m_handle.getParam("authority_private_key", m_authority_private_key_string)) {
       
        bio_ptr bio_authority_private_key(BIO_new_mem_buf(m_authority_private_key_string.c_str(), -1));

        if (!bio_authority_private_key) {
            throw std::runtime_error("Failed digesting authority key to BIO.");
        }

        EVP_PKEY* authority_private_key = nullptr;
        authority_private_key = PEM_read_bio_PrivateKey(bio_authority_private_key.get(), &authority_private_key, nullptr, nullptr);

        if (authority_private_key == nullptr) {
            throw std::runtime_error("Failed reading RSA key.");
        }

        m_authority_private_key.reset(authority_private_key);

    }

    /*
        Generate new RSA keypair
    */

    evp_pkey_ctx_ptr keygen_context(EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, NULL));
    
    if (!keygen_context) {
        throw std::runtime_error("Failed creating EVP pkey context!");
    }

    if (EVP_PKEY_keygen_init(keygen_context.get()) <= 0) {
        throw std::runtime_error("Failed initializing EVP pkey context!");
    }

    // Note that the longer the key size, the more time async encryption will take. 
    if (EVP_PKEY_CTX_set_rsa_keygen_bits(keygen_context.get(), 1024) <= 0) {
        throw std::runtime_error("Failed adjusting key size!");
    }

    m_recorder_key.reset(EVP_PKEY_new());
    
    if (!m_recorder_key) {
        throw std::runtime_error("Failed creating key!");
    }

    EVP_PKEY* recorder_key_raw = m_recorder_key.get();

    if (EVP_PKEY_keygen(keygen_context.get(), &recorder_key_raw) != 1) {
        throw std::runtime_error("Failed generating recorder key!");
    }  

    /*
        Store private key to string
    */

    bio_ptr bio_private_key(BIO_new(BIO_s_mem()));
    
    if (!bio_private_key) {
        throw std::runtime_error("Failed creating memory bio for dumping private key.");
    }

    if (!PEM_write_bio_PrivateKey(bio_private_key.get(), m_recorder_key.get(), NULL, NULL, 0, NULL, NULL)) {
        throw std::runtime_error("Failed writing recorder private key to BIO.");
    }

    int rsa_private_key_length = BIO_pending(bio_private_key.get());
    m_recorder_private_key_string.resize(rsa_private_key_length);

    if (BIO_read(bio_private_key.get(), (unsigned char*) &m_recorder_private_key_string[0], rsa_private_key_length) <= 0) {
        throw std::runtime_error("Failed writing recorder private key to std string.");
    }

    ROS_INFO_STREAM("Recorder private key: \n" << m_recorder_private_key_string);

    /*
        Store public key to string
    */

    bio_ptr bio_public_key(BIO_new(BIO_s_mem()));

    if (!PEM_write_bio_PUBKEY(bio_public_key.get(), m_recorder_key.get())) {
        // TODO: Free and Error
        return;
    }

    int rsa_public_key_length = BIO_pending(bio_public_key.get());
    m_recorder_public_key_string.resize(rsa_public_key_length);

    if (BIO_read(bio_public_key.get(), (unsigned char*) &m_recorder_public_key_string[0], rsa_public_key_length) <= 0) {
        // TODO: Free and Error
        return;
    }
    
    ROS_INFO_STREAM("Recorder public key: \n" << m_recorder_public_key_string);

}

OpenSSLPluginKeyManager::~OpenSSLPluginKeyManager() {
    // 
}

ros::NodeHandle& OpenSSLPluginKeyManager::getKeyManagerNodeHandle() {
    return m_handle;
}


EVP_PKEY* OpenSSLPluginKeyManager::getRecorderKey() {
    return m_recorder_key.get();
}

const std::string& OpenSSLPluginKeyManager::getRecorderPublicKeyString() const {
    return m_recorder_public_key_string;
}

const std::string& OpenSSLPluginKeyManager::getRecorderPrivateKeyString() const {
    return m_recorder_private_key_string;
}

   
EVP_PKEY* OpenSSLPluginKeyManager::getAuthorityPublicKey() {
    return m_authority_public_key.get();
}

const std::string& OpenSSLPluginKeyManager::getAuthorityPublicKeyString() const {
    return m_authority_public_key_string;
}

EVP_PKEY* OpenSSLPluginKeyManager::getAuthorityPrivateKey() {
    return m_authority_private_key.get();
} 

std::string OpenSSLPluginKeyManager::getAuthorityPrivateKeyString() const {
    return m_authority_private_key_string;
} 



}