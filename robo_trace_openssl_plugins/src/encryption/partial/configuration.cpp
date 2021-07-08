// Base
#include "robo_trace_openssl_plugins/encryption/partial/configuration.hpp"

namespace robo_trace {

OpenSSLPartialEncryptionConfiguration::OpenSSLPartialEncryptionConfiguration() = default;

OpenSSLPartialEncryptionConfiguration::~OpenSSLPartialEncryptionConfiguration() = default;
    

const std::string& OpenSSLPartialEncryptionConfiguration::getEncryptionMethod() const {
    return m_encryption_method;
}

void OpenSSLPartialEncryptionConfiguration::setEnryptionMethod(const std::string method) {
    m_encryption_method = method;
}
    
std::unordered_map<std::string, OpenSSLPartialEncryptionConfiguration::EncryptionTarget::Ptr>& OpenSSLPartialEncryptionConfiguration::getEncryptionTargetsTree() {
    return m_encryption_targets;
}


}