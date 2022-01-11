#pragma once

// Std
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>


namespace robo_trace::plugin::open_ssl {

/**
 *
 */
class PartialEncryptionModuleConfiguration {

public:

    typedef std::shared_ptr<PartialEncryptionModuleConfiguration> Ptr;
    typedef std::shared_ptr<const PartialEncryptionModuleConfiguration> ConstPtr;

    struct EncryptionTarget {

        typedef std::shared_ptr<PartialEncryptionModuleConfiguration::EncryptionTarget> Ptr;

        EncryptionTarget() = default;

        std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr> children;
        std::vector<std::string> targets;
    };

public:
    
    /**
     *
     */
    PartialEncryptionModuleConfiguration();

    /**
     *
     */
    ~PartialEncryptionModuleConfiguration();
    
    /**
     *
     */
    const std::string& getEncryptionMethod() const;

    /**
     *
     */
    void setEnryptionMethod(const std::string method);
  
    /**
     *
     */
    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>& getEncryptionTargetsTree();

private:

    /** The OpenSSL encryption method name to use for encrypting messages. */
    std::string m_encryption_method;

    // From message type to encryption targets
    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr> m_encryption_targets;
};

}