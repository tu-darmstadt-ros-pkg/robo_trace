// Base
#include "robo_trace_plugin_interface/processing/context.hpp"


namespace robo_trace {

MessageProcessingContext::MessageProcessingContext(MetadataContainer::Ptr metadata, Message::Ptr message)
: m_message(message), m_metadata(metadata) {
    // 
}

MessageProcessingContext::~MessageProcessingContext() = default;


const MetadataContainer::Ptr& MessageProcessingContext::getMetadata() const {
    return m_metadata;
}

const Message::Ptr& MessageProcessingContext::getMessage() const {
    return m_message; 
}

MessageProcessingContext::Status MessageProcessingContext::getStatusCode() const {
    return m_status_code;
}

const std::string& MessageProcessingContext::getStatusMessage() const {
    return m_status_message;
}

void MessageProcessingContext::setStatus(const MessageProcessingContext::Status status, const std::string& message) {
    m_status_code = status;
    m_status_message = message;
}

}