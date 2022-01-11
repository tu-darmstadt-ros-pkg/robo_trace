// Base
#include "robo_trace_openssl_plugin/chain/configuration.hpp"


namespace robo_trace::plugin::open_ssl {


HashChainModuleConfiguration::HashChainModuleConfiguration() = default;

HashChainModuleConfiguration::~HashChainModuleConfiguration() = default;


const std::string& HashChainModuleConfiguration::getHashingMehtodName() const {
    return hashing_method_name;
}

void HashChainModuleConfiguration::setHashingMethodName(const std::string name) {
    hashing_method_name = name;
}


const std::string& HashChainModuleConfiguration::getHashingResultStorageKey() const {
    return hashing_result_storage_key;
}

void HashChainModuleConfiguration::setHashingResultStorageKey(const std::string key) {
    hashing_result_storage_key = key;
}


}