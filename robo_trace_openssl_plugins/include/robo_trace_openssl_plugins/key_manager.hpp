#pragma once
// Std
#include <string>
#include <memory>
// OpenSSL
#include <openssl/rsa.h>
#include <openssl/pem.h>
// Ros
#include <ros/ros.h>


namespace robo_trace {

class OpenSSLPluginKeyManager {

public:

    typedef std::shared_ptr<OpenSSLPluginKeyManager> Ptr;
    typedef std::shared_ptr<const OpenSSLPluginKeyManager> ConstPtr;

public:

    /**
     * 
     */
    OpenSSLPluginKeyManager(const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    ~OpenSSLPluginKeyManager();

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
    const std::string& getRecorderPublicKey() const;

    /**
     * 
     */
    const std::string& getRecorderPrivateKey() const;

private:

    /** */
    ros::NodeHandle m_handle;

    /** */
    EVP_PKEY* m_recorder_key;

    /** */
    std::string m_recorder_public_key;
    /** */
    std::string m_recorder_private_key;

};

}