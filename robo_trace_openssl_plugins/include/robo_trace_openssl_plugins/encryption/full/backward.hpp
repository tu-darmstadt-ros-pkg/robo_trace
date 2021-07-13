#pragma once
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"
#include "robo_trace_openssl_plugins/encryption/full/configuration.hpp"
// BabelFish
#include <ros_babel_fish/message_description.h>


namespace robo_trace {

class OpenSSLFullEncryptionBackwardStage final : public ProcessingStage {

public:

    /**
     * TODO
     */
    OpenSSLFullEncryptionBackwardStage(const OpenSSLFullEncryptionConfiguration::Ptr& configuration, const OpenSSLPluginKeyManager::Ptr& key_manager, const ros_babel_fish::MessageDescription::ConstPtr& message_description);

    /**
     * TODO
     */
    virtual ~OpenSSLFullEncryptionBackwardStage();

    /**
     * 
     */
    const OpenSSLFullEncryptionConfiguration::Ptr getConfiguration() const;

    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr getKeyManager() const;

    /**
     * TODO
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    /** */
    const OpenSSLPluginKeyManager::Ptr m_key_manager;
    /** */
    const OpenSSLFullEncryptionConfiguration::Ptr m_configuration;
    /** */
    const ros_babel_fish::MessageDescription::ConstPtr m_message_description;

    /** */
    std::string m_key;
    /** */
    std::string m_iv;

    // No idea who's the owner of that pointer tbh.
    const EVP_CIPHER* m_encryption_method;
    /** */
    EVP_CIPHER_CTX* m_decryption_context;


    
};


}