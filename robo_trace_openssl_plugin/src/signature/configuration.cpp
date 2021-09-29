// Base
#include "robo_trace_openssl_plugin/signature/configuration.hpp"


namespace robo_trace {


OpenSSLSignatureStageConfiguration::OpenSSLSignatureStageConfiguration() = default;

OpenSSLSignatureStageConfiguration::~OpenSSLSignatureStageConfiguration() = default;


const std::string& OpenSSLSignatureStageConfiguration::getHashingMehtodName() const {
    return m_hashing_method_name;
}

void OpenSSLSignatureStageConfiguration::setHashingMethodName(const std::string name) {
    m_hashing_method_name = name;
}


const std::string& OpenSSLSignatureStageConfiguration::getInputStorageKey() const {
    return m_input_storage_key;
}

void OpenSSLSignatureStageConfiguration::setInputStorageKey(const std::string key) {
    m_input_storage_key = key;
}


const std::string& OpenSSLSignatureStageConfiguration::getResultStorageKey() const {
    return m_result_storage_key;
}

void OpenSSLSignatureStageConfiguration::setResultStorageKey(const std::string key) {
    m_result_storage_key = key;
}   


}