// Base
#include "robo_trace/processing/modules/marshalling/backward.hpp"
// BabelFish
#include <ros_babel_fish/message.h>
// Project
#include "robo_trace/processing/translation/translator.hpp"


namespace robo_trace::processing {

BasicMarshallingBackwardProcessor::BasicMarshallingBackwardProcessor(const ros_babel_fish::MessageDescription::ConstPtr& message_description) 
: m_message_description(message_description) {
    //
}
  
BasicMarshallingBackwardProcessor::~BasicMarshallingBackwardProcessor() {
    // 
}

Mode BasicMarshallingBackwardProcessor::getMode() const {
    return Mode::REPLAY;
}

void BasicMarshallingBackwardProcessor::process(const Context::Ptr& context) {
   
    /*
        Currently the serialized message (in Mongo format) is deserialized into a babel 
        fish hierarchical message. Afterwards this hierarchical message is serialized into
        a bytestream that may be published, etc.

        One could try to skip the babel fish intermediate representation.

    */

    const std::optional<bsoncxx::document::view>& o_serialized = context->getBsonMessage();

    if (!o_serialized) {
        throw std::runtime_error("Message to be deserialized not found.");
    }

    const bsoncxx::document::view& serialized = o_serialized.value();
    ros_babel_fish::CompoundMessage::Ptr deserialized = std::make_shared<ros_babel_fish::CompoundMessage>(m_message_description->message_template);
    
    Translation::deserialize(m_message_description->message_template, serialized, *deserialized);
    
    // Write hierarchical message to the byte stream
    ros_babel_fish::BabelFishMessage::Ptr stream_message(new ros_babel_fish::BabelFishMessage());
    stream_message->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");
    stream_message->allocate(deserialized->_sizeInBytes());
    deserialized->writeToStream(stream_message->buffer());
 
    context->setRosMessage(stream_message);
}

}

