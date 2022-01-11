// Base
#include "robo_trace_openssl_plugin/encryption/full/configuration.hpp"


namespace robo_trace::plugin::open_ssl {

FullEncryptionModuleConfiguration::FullEncryptionModuleConfiguration() = default;

FullEncryptionModuleConfiguration::~FullEncryptionModuleConfiguration() = default;

const std::string& FullEncryptionModuleConfiguration::getEncryptionMethod() const {
    return m_encryption_method;
}

void FullEncryptionModuleConfiguration::setEnryptionMethod(const std::string method) {
    m_encryption_method = method;
}

}