// Std
#include <memory>
// Babel Fish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace_plugin_interface/processing/message.hpp"



namespace robo_trace {

/**
 * A lazy message implementation. The message is only decoded if needed 
 * and subsequent modifications to either decoded message or its stream 
 * made visible, when needed (hence lazy).
 * 
 * If for example the decoded message is never required, it is never 
 * constructed in the first place.
 * 
 */
class LazyMessage final : public Message {

public:

    typedef std::shared_ptr<LazyMessage> Ptr;
    typedef std::shared_ptr<const LazyMessage> ConstPtr;

public:

    /**
     * 
     */
    LazyMessage(ros_babel_fish::BabelFish& fish, const ros_babel_fish::BabelFishMessage::ConstPtr description);

    /**
     * 
     */
    virtual ~LazyMessage();

    /**
     * 
     */
    virtual size_t getLength() final override; 

    /**
     * 
     */
    virtual const uint8_t* const getStream() final override;

    /**
     * 
     */
    virtual void setStream(std::unique_ptr<uint8_t[]> stream, const size_t length) final override;

    /**
     * 
     */
    virtual bool isDecodable() final override;

    /**
     * 
     */
    virtual const ros_babel_fish::Message::Ptr& getDecoded() final override;


private:

    void synchronize();

private:


    /** The babel fish instance used for decoding message streams. */
    ros_babel_fish::BabelFish& m_fish;
    
    // This ptr will point be a nullptr as long as the initial consistency between
    // inital stream and message exists. 
    std::unique_ptr<uint8_t[]> m_stream_data;
    size_t m_stream_length = 0;

    ros_babel_fish::Message::Ptr m_message;

};

}