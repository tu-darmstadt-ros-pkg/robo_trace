#pragma once
// OpenSSL
#include <openssl/evp.h>
// BabelFish
#include <ros_babel_fish/message_description.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

class FullEncryptionBackwardProcessor final : public robo_trace::processing::Processor {

public:

    /**
     * TODO
     */
    FullEncryptionBackwardProcessor(const FullEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata);

    /**
     * TODO
     */
    virtual ~FullEncryptionBackwardProcessor();
  
    /**
     *
     */
    virtual robo_trace::processing::Mode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const robo_trace::processing::Context::Ptr& context) final override;

private:

    /** */
    const FullEncryptionModuleConfiguration::Ptr m_configuration;
    /** */
    const KeyManager::Ptr m_key_manager;
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