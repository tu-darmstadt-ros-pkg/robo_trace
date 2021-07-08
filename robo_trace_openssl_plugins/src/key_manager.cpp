// Base
#include "robo_trace_openssl_plugins/key_manager.hpp"
// OpenSSL
#include <openssl/err.h>
#include <openssl/rand.h>
// Ros
#include <ros/console.h>


namespace robo_trace {

OpenSSLPluginKeyManager::OpenSSLPluginKeyManager(const ros::NodeHandle& plugin_namespace) {
    
    /*
        Setup namespace
    */
    m_handle = ros::NodeHandle(plugin_namespace, "openssl_keys");

    /*
        Generate new RSA keypair
    */

    //EVP_PKEY_CTX* keygen_context = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, NULL);
    std::unique_ptr<EVP_PKEY_CTX, decltype(&::EVP_PKEY_CTX_free)> keygen_context(EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, NULL), ::EVP_PKEY_CTX_free);
    
    if (!keygen_context) {
        // TODO: Error
        return;
    }

    if (EVP_PKEY_keygen_init(keygen_context.get()) <= 0) {
        // TODO: Free and Error
        return;
    }

    if (EVP_PKEY_CTX_set_rsa_keygen_bits(keygen_context.get(), 2048) <= 0) {
        // TODO: Free and Error
        return;
    }

    m_recorder_key = EVP_PKEY_new();
   
    if (EVP_PKEY_keygen(keygen_context.get(), &m_recorder_key) != 1) {
        // TODO: Free and Error
        return;
    }  


    /*
        Store private key to string
    */

    // TODO: schmart ptr
    BIO* bio_private_key = BIO_new(BIO_s_mem());
    
    if (!PEM_write_bio_PrivateKey(bio_private_key, m_recorder_key, NULL, NULL, 0, NULL, NULL)) {
        // TODO: Free and Error
        return;
    }

    int rsa_private_key_length = BIO_pending(bio_private_key);
    m_recorder_private_key.resize(rsa_private_key_length);

    if (BIO_read(bio_private_key, (unsigned char*) &m_recorder_private_key[0], rsa_private_key_length) <= 0) {
        // TODO: Free and Error
        return;
    }

    ROS_INFO_STREAM("Recorder private key: " << m_recorder_private_key);

    /*
        Store public key to string
    */

    BIO* bio_public_key = BIO_new(BIO_s_mem());

    if (!PEM_write_bio_PUBKEY(bio_public_key, m_recorder_key)) {
        // TODO: Free and Error
        return;
    }

    int rsa_public_key_length = BIO_pending(bio_public_key);
    m_recorder_public_key.resize(rsa_public_key_length);

    if (BIO_read(bio_public_key, (unsigned char*) &m_recorder_public_key[0], rsa_public_key_length) <= 0) {
        // TODO: Free and Error
        return;
    }
    
    ROS_INFO_STREAM("Recorder public key: " << m_recorder_public_key);

    /*
        Load authority key
    */

    // TODO:

}

OpenSSLPluginKeyManager::~OpenSSLPluginKeyManager() {
    EVP_PKEY_free(m_recorder_key);
}

ros::NodeHandle& OpenSSLPluginKeyManager::getKeyManagerNodeHandle() {
    return m_handle;
}

EVP_PKEY* OpenSSLPluginKeyManager::getRecorderKey() {
    return m_recorder_key;
}

const std::string& OpenSSLPluginKeyManager::getRecorderPublicKey() const {
    return m_recorder_public_key;
}

const std::string& OpenSSLPluginKeyManager::getRecorderPrivateKey() const {
    return m_recorder_private_key;
}

}