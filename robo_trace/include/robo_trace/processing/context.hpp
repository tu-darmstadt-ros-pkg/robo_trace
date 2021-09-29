#pragma once

// Std
#include <memory>
#include <optional>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/bson/bsonobj.h>
// BabelFish
#include <ros_babel_fish/babel_fish_message.h>
// Project
#include "robo_trace/storage/container.hpp"


namespace robo_trace {

class ProcessingContext {

public:

    typedef std::shared_ptr<ProcessingContext> Ptr;
    typedef std::shared_ptr<const ProcessingContext> ConstPtr;

public:

    /**
     *
     */
    ProcessingContext();

    /**
     *
     */
    ProcessingContext(const DataContainer::Ptr& metadata);

    /**
     *
     */
    ~ProcessingContext();

    /**
     *
     */
    const DataContainer::Ptr& getMetadata() const;
    
    /**
     *
     */
    bool isTerminated() const;

    /**
     *
     */
    void setTerminated();

    /**
     *
     */
    bool isUnserialized() const;

    /**
     * 
     */
    const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& getUnserializedMessage() const;

    /**
     * 
     */
    const std::optional<const uint8_t* const> getUnserializedMessage(size_t& length) const;

    /**
     *
     */
    void setUnserializedMessage(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress);

    /**
     *
     */
    bool isSerialized() const;

    /**
     * 
     */
    const std::optional<mongo::BSONObj>& getSerializedMessage() const;

    /**
     * 
     */
    const std::optional<const uint8_t* const> getSerializedMessage(size_t& length) const;

    /**
     * 
     */
    void setSerializedMessage(const mongo::BSONObj& serialized);

protected:

    /** */
    const DataContainer::Ptr m_metadata;

    /** */
    bool m_terminated;
    
    /** */
    std::optional<mongo::BSONObj> m_serialized_message;
    /**  */
    std::optional<ros_babel_fish::BabelFishMessage::ConstPtr> m_unserialized_message;

};

}