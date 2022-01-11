#pragma once

// Std
#include <string>
#include <memory>


namespace robo_trace::plugin::open_ssl {

class FullEncryptionModuleConfiguration {

public:

    typedef std::shared_ptr<FullEncryptionModuleConfiguration> Ptr;
    typedef std::shared_ptr<const FullEncryptionModuleConfiguration> ConstPtr;

public:
    
    /**
     * 
     */
    FullEncryptionModuleConfiguration();

    /**
     * 
     */
    ~FullEncryptionModuleConfiguration();
    
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