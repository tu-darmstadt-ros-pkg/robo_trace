#pragma once

// Std
#include <string>
#include <memory>
// OpenSSL
#include <openssl/evp.h>


namespace robo_trace {

class OpenSSLSignatureStageConfiguration {

public:

    typedef std::shared_ptr<OpenSSLSignatureStageConfiguration> Ptr;
    typedef std::shared_ptr<const OpenSSLSignatureStageConfiguration> ConstPtr;

public:
    
    /**
     * 
     */
    OpenSSLSignatureStageConfiguration();

    /**
     * 
     */
    ~OpenSSLSignatureStageConfiguration();
    
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