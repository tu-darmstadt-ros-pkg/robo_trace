#pragma once

// Std
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>


namespace robo_trace {

/**
 *
 */
class OpenSSLPartialEncryptionConfiguration {

public:

    typedef std::shared_ptr<OpenSSLPartialEncryptionConfiguration> Ptr;
    typedef std::shared_ptr<const OpenSSLPartialEncryptionConfiguration> ConstPtr;

    struct EncryptionTarget {

        typedef std::shared_ptr<OpenSSLPartialEncryptionConfiguration::EncryptionTarget> Ptr;

        EncryptionTarget() = default;

        std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr> children;
        std::vector<std::string> targets;
    };

public:
    
    /**
     *
     */
    OpenSSLPartialEncryptionConfiguration();

    /**
     *
     */
    ~OpenSSLPartialEncryptionConfiguration();
    
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
    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>& getEncryptionTargetsTree();

private:

    /** The OpenSSL encryption method name to use for encrypting messages. */
    std::string m_encryption_method;

    // From message type to encryption targets
    std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr> m_encryption_targets;
};

}