#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/message.hpp"
#include "robo_trace_plugin_interface/processing/metadata.hpp"


namespace robo_trace {

class MessageProcessingContext {
   
public:

    enum Status {
        /** */
        VALID,
        /** */
        ERROR,
    };

    typedef std::shared_ptr<MessageProcessingContext> Ptr;
    typedef std::shared_ptr<const MessageProcessingContext> ConstPtr;

public:

    /**
     * 
     */
    MessageProcessingContext(MetadataContainer::Ptr metadata, Message::Ptr message);

    /**
     * 
     */
    ~MessageProcessingContext();

    /**
     * Returns the metadata that is associated to the message. This metadata 
     * will also be persisted.
     * 
     * @returns the metadata associated to the message
     */
    const MetadataContainer::Ptr& getMetadata() const;

    /**
     * 
     */
    const Message::Ptr& getMessage() const;

     /**
     * 
     */
    MessageProcessingContext::Status getStatusCode() const;

    /**
     * 
     */
    const std::string& getStatusMessage() const;

    /**
     * 
     */
    void setStatus(const MessageProcessingContext::Status status, const std::string& message = "");

private:

    /** */
    const Message::Ptr m_message;
    /** */
    const MetadataContainer::Ptr m_metadata;

    /** */
    MessageProcessingContext::Status m_status_code;
    /** */
    std::string m_status_message;

};

}