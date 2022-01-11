#pragma once

// Std
#include <string>
#include <memory>
// OpenSSL
#include <openssl/evp.h>


namespace robo_trace::plugin::open_ssl {

class SignatureModuleConfiguration {

public:

    typedef std::shared_ptr<SignatureModuleConfiguration> Ptr;
    typedef std::shared_ptr<const SignatureModuleConfiguration> ConstPtr;

public:
    
    /**
     * 
     */
    SignatureModuleConfiguration();

    /**
     * 
     */
    ~SignatureModuleConfiguration();
    
    /**
     * 
     */
    const std::string& getHashingMehtodName() const;

    /**
     * 
     */
    void setHashingMethodName(const std::string name);

    /**
     * 
     */
    const std::string& getInputStorageKey() const;

    /**
     * 
     */
    void setInputStorageKey(const std::string key);

     /**
     * 
     */
    const std::string& getResultStorageKey() const;

    /**
     * 
     */
    void setResultStorageKey(const std::string key);

private:

    /** */
    std::string m_hashing_method_name;
    
    /** */
    std::string m_input_storage_key;
    /** */
    std::string m_result_storage_key;

};

}