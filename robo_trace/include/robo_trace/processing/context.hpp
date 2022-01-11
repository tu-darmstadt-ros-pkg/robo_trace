#pragma once

// Std
#include <memory>
#include <optional>
// MongoDB
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/document/value.hpp>
// BabelFish
#include <ros_babel_fish/babel_fish_message.h>
// Project
#include "robo_trace/storage/container.hpp"


namespace robo_trace::processing {

class Context {

public:

    typedef std::shared_ptr<Context> Ptr;
    typedef std::shared_ptr<const Context> ConstPtr;

public:

    /**
     *
     */
    Context();

    /**
     *
     */
    Context(const robo_trace::store::Container::Ptr& metadata);

    /**
     *
     */
    ~Context();

    /**
     *
     */
    const robo_trace::store::Container::Ptr& getMetadata() const;
    
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
    const std::optional<const uint8_t*> getUnserializedMessage(size_t& length) const;

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
    const std::optional<bsoncxx::document::view>& getSerializedMessage() const;

    /**
     * 
     */
    const std::optional<const uint8_t* const> getSerializedMessage(size_t& length) const;

    /**
     * 
     */
    void setSerializedMessage(const bsoncxx::document::value& serialized);

    /**
     * Sets the serialized message for this context. Note that with this
     * method a non owning document is passed! Hence the user must ensure
     * that the owner exists for the duration of the lifetime of the context.
     * 
     * @param serialized the serialized message
     */
    void setSerializedMessage(const bsoncxx::document::view serialized);

protected:

    /** */
    const robo_trace::store::Container::Ptr m_metadata;

    /** */
    bool m_terminated;
    
    /** */
    std::optional<bsoncxx::document::view> m_serialized_message;
    /** */
    std::optional<bsoncxx::document::value> m_serialized_owning;

    /**  */
    std::optional<ros_babel_fish::BabelFishMessage::ConstPtr> m_unserialized_message;

};

}