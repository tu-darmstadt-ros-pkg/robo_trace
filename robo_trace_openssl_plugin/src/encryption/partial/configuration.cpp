// Base
#include "robo_trace_openssl_plugin/encryption/partial/configuration.hpp"

namespace robo_trace::plugin::open_ssl {

PartialEncryptionModuleConfiguration::PartialEncryptionModuleConfiguration() = default;

PartialEncryptionModuleConfiguration::~PartialEncryptionModuleConfiguration() = default;
    

const std::string& PartialEncryptionModuleConfiguration::getEncryptionMethod() const {
    return m_encryption_method;
}

void PartialEncryptionModuleConfiguration::setEnryptionMethod(const std::string method) {
    m_encryption_method = method;
}

std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>& PartialEncryptionModuleConfiguration::getEncryptionTargetsTree() {
    return m_encryption_targets;
}

}