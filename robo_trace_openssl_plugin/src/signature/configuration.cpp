// Base
#include "robo_trace_openssl_plugin/signature/configuration.hpp"


namespace robo_trace::plugin::open_ssl {


SignatureModuleConfiguration::SignatureModuleConfiguration() = default;

SignatureModuleConfiguration::~SignatureModuleConfiguration() = default;


const std::string& SignatureModuleConfiguration::getHashingMehtodName() const {
    return m_hashing_method_name;
}

void SignatureModuleConfiguration::setHashingMethodName(const std::string name) {
    m_hashing_method_name = name;
}


const std::string& SignatureModuleConfiguration::getInputStorageKey() const {
    return m_input_storage_key;
}

void SignatureModuleConfiguration::setInputStorageKey(const std::string key) {
    m_input_storage_key = key;
}


const std::string& SignatureModuleConfiguration::getResultStorageKey() const {
    return m_result_storage_key;
}

void SignatureModuleConfiguration::setResultStorageKey(const std::string key) {
    m_result_storage_key = key;
}   


}