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
#include "robo_trace/storage/persistor.hpp"
#include "robo_trace/storage/stream.hpp"


namespace robo_trace::processing {

class Context {

public:

    typedef std::shared_ptr<Context> Ptr;
    typedef std::shared_ptr<const Context> ConstPtr;

public:

    /**
     *
     */
    Context(const robo_trace::store::StreamHandler::Ptr stream);

    /**
     *
     */
    Context(const robo_trace::store::Container::Ptr& metadata, const robo_trace::store::StreamHandler::Ptr stream);

    /**
     *
     */
    Context(const robo_trace::store::Container::Ptr& metadata, const robo_trace::store::Persistor::Ptr& persistor, const robo_trace::store::StreamHandler::Ptr stream);

    /**
     *
     */
    ~Context();

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
    const robo_trace::store::Container::Ptr& getMetadata() const;

    /**
     * 
     */
    const robo_trace::store::StreamHandler::Ptr getStreamHandler() const;

    /**
     * 
     */
    bool getHasPersistor() const;

    /**
     * 
     */
    const std::optional<robo_trace::store::Persistor::Ptr>& getPersistor() const;

   
    /**
     *
     */
    bool getHasRosMessage() const;

    /**
     * 
     */
    const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& getRosMessage() const;

    /**
     * 
     */
    const std::optional<const uint8_t*> getRosMessageStream(size_t& length) const;

    /**
     *
     */
    void setRosMessage(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress);

    /**
     *
     */
    bool getHasBsonMessage() const;

    /**
     * 
     */
    const std::optional<bsoncxx::document::view>& getBsonMessage() const;

    /**
     * 
     */
    const std::optional<const uint8_t* const> getBsonMessageStream(size_t& length) const;

    /**
     * 
     */
    void setBsonMessage(const bsoncxx::document::value& serialized);

    /**
     * Sets the serialized message for this context. Note that with this
     * method a non owning document is passed! Hence the user must ensure
     * that the owner exists for the duration of the lifetime of the context.
     * 
     * @param serialized the serialized message
     */
    void setBsonMessage(const bsoncxx::document::view serialized);

protected:

    /** */
    const robo_trace::store::Container::Ptr m_metadata;
    /** */
    const robo_trace::store::StreamHandler::Ptr m_stream_handler;
    /** */
    const std::optional<robo_trace::store::Persistor::Ptr> m_persistor;

    /** */
    bool m_terminated;
    
    /** */
    std::optional<bsoncxx::document::view> m_bson_message;
    /** */
    std::optional<bsoncxx::document::value> m_bson_owning;

    /**  */
    std::optional<ros_babel_fish::BabelFishMessage::ConstPtr> m_unserialized_message;

};

}