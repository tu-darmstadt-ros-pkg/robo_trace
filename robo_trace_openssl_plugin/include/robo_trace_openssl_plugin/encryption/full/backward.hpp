#pragma once
// OpenSSL
#include <openssl/evp.h>
// BabelFish
#include <ros_babel_fish/message_description.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/stage/stage.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace {

class OpenSSLFullEncryptionBackwardStage final : public ProcessingStage {

public:

    /**
     * TODO
     */
    OpenSSLFullEncryptionBackwardStage(const OpenSSLFullEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const DataContainer::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~OpenSSLFullEncryptionBackwardStage();
  
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const OpenSSLFullEncryptionConfiguration::Ptr m_configuration;
    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;
    
    /** */
    std::vector<uint8_t> m_key;
    /** */
    std::vector<uint8_t> m_iv;

    // No idea who's the owner of that pointer tbh.
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_decryption_context;


    
};


}