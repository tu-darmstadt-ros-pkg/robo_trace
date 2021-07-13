#pragma once
// Std
#include <memory>
// Babel fish
#include <ros_babel_fish/babel_fish_message.h>
// MongoDB
#include "robo_trace_plugin_interface/config.h"
#include <mongo/bson/bsonobj.h>
// Project
#include "robo_trace_plugin_interface/config.h"


namespace robo_trace {

class Message {

public:

    typedef std::shared_ptr<Message> Ptr;
    typedef std::shared_ptr<const Message> ConstPtr;

public:

    /**
     * 
     */
    Message(const ros_babel_fish::BabelFishMessage::ConstPtr ingress_message);

    /**
     * 
     */
    virtual ~Message();

    /**
     * 
     */
    size_t getStreamLength() const; 

    /**
     * 
     */
    const uint8_t* const getStreamData() const;

    /**
     * 
     */
    size_t getIngressStreamLength() const; 

    /**
     * 
     */
    const uint8_t* const getIngressStreamData() const;

    /**
     * 
     */
    const ros_babel_fish::BabelFishMessage::ConstPtr& getIngress() const;

    /**
     *
     */
    void setIngress(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress);

    /**
     * 
     */
    size_t getSerializedStreamLength() const;

    /**
     * 
     */
    const uint8_t* const getSerializedStreamData() const;

    /**
     * 
     */
    const mongo::BSONObj& getSerialized() const;

    /**
     * 
     */
    void setSerialized(const mongo::BSONObj& serialized);

protected:

    /**  */
    ros_babel_fish::BabelFishMessage::ConstPtr m_ingress_message;
    /** */
    mongo::BSONObj m_serialized_message;
    
};

}