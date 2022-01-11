#pragma once

// Std
#include <string>
#include <memory>
#include <optional>
// OpenSSL
#include <openssl/rsa.h>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace_openssl_plugin/definitions.hpp"


namespace robo_trace::plugin::open_ssl {

class KeyManager {

public:

    typedef std::shared_ptr<KeyManager> Ptr;
    typedef std::shared_ptr<const KeyManager> ConstPtr;

public:

    /**
     * 
     */
    KeyManager(const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    ~KeyManager();

    /**
     * 
     */
    ros::NodeHandle& getKeyManagerNodeHandle();

    /**
     * 
     * 
     * Note: Instead of smart point, a normal pointer is retured as 
     * any instance using his method should NOT (permanently) copy
     * the pointer to its own memory. 
     */
    EVP_PKEY* getRecorderKey();

    /**
     * 
     */
    const std::string& getRecorderPublicKeyString() const;

    /**
     * 
     */
    const std::string& getRecorderPrivateKeyString() const;

    /**
     *
     */
    EVP_PKEY* getAuthorityPublicKey();
    
    /**
     *
     */
    const std::string& getAuthorityPublicKeyString() const;

    /**
     *
     */
    EVP_PKEY* getAuthorityPrivateKey(); 

    /**
     *
     */
    std::string getAuthorityPrivateKeyString() const; 


private:

    /** */
    ros::NodeHandle m_handle;

    /** */
    evp_pkey_ptr m_recorder_key;
    /** */
    std::string m_recorder_public_key_string;
    /** */
    std::string m_recorder_private_key_string;
        
    /** */
    evp_pkey_ptr m_authority_public_key;
    /** */
    std::string m_authority_public_key_string;
   
    /** */
    evp_pkey_ptr m_authority_private_key;
    /** */
    std::string m_authority_private_key_string;


};

}