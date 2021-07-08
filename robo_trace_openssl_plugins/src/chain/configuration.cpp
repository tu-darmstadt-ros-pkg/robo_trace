// Base
#include "robo_trace_openssl_plugins/chain/configuration.hpp"


namespace robo_trace {


OpenSSLHashChainConfiguration::OpenSSLHashChainConfiguration() = default;

OpenSSLHashChainConfiguration::~OpenSSLHashChainConfiguration() = default;


const std::string& OpenSSLHashChainConfiguration::getHashingMehtodName() const {
    return hashing_method_name;
}

void OpenSSLHashChainConfiguration::setHashingMethodName(const std::string name) {
    hashing_method_name = name;
}


const std::string& OpenSSLHashChainConfiguration::getHashingResultStorageKey() const {
    return hashing_result_storage_key;
}

void OpenSSLHashChainConfiguration::setHashingResultStorageKey(const std::string key) {
    hashing_result_storage_key = key;
}


}