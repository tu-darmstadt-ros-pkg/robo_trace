#pragma once

// Std
#include <string>
#include <memory>


namespace robo_trace {

class OpenSSLHashChainConfiguration final {

public:

    typedef std::shared_ptr<OpenSSLHashChainConfiguration> Ptr;
    typedef std::shared_ptr<const OpenSSLHashChainConfiguration> ConstPtr;

public:
    
    /**
     * 
     */
    OpenSSLHashChainConfiguration();

    /**
     * 
     */
    ~OpenSSLHashChainConfiguration();
    
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
    const std::string& getHashingResultStorageKey() const;

    /**
     * 
     */
    void setHashingResultStorageKey(const std::string key);

private:

    std::string hashing_method_name;
    std::string hashing_result_storage_key;

};

}