// Base
#include "robo_trace_openssl_plugins/encryption/full/configuration.hpp"


namespace robo_trace {

OpenSSLFullEncryptionConfiguration::OpenSSLFullEncryptionConfiguration() = default;

OpenSSLFullEncryptionConfiguration::~OpenSSLFullEncryptionConfiguration() = default;


const std::string& OpenSSLFullEncryptionConfiguration::getEncryptionMethod() const {
    return m_encryption_method;
}

void OpenSSLFullEncryptionConfiguration::setEnryptionMethod(const std::string method) {
    m_encryption_method = method;
}

}