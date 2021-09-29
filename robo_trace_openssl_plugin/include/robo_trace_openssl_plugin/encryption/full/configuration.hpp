#pragma once

// Std
#include <string>
#include <memory>


namespace robo_trace {

class OpenSSLFullEncryptionConfiguration {

public:

    typedef std::shared_ptr<OpenSSLFullEncryptionConfiguration> Ptr;
    typedef std::shared_ptr<const OpenSSLFullEncryptionConfiguration> ConstPtr;

public:
    
    /**
     * 
     */
    OpenSSLFullEncryptionConfiguration();

    /**
     * 
     */
    ~OpenSSLFullEncryptionConfiguration();
    
    /**
     * 
     */
    const std::string& getEncryptionMethod() const;

    /**
     * 
     */
    void setEnryptionMethod(const std::string method);
    
private:

    std::string m_encryption_method;
   
};

}