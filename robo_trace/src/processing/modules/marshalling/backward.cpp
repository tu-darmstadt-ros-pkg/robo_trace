// Base
#include "robo_trace/processing/modules/marshalling/backward.hpp"
// BabelFish
#include <ros_babel_fish/message.h>
#include <ros_babel_fish/messages/value_message.h>
#include <ros_babel_fish/messages/array_message.h>


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
    
    deserialize(m_message_description->message_template, serialized, *deserialized);
    
    // Write hierarchical message to the byte stream
    ros_babel_fish::BabelFishMessage::Ptr stream_message(new ros_babel_fish::BabelFishMessage());
    stream_message->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");
    stream_message->allocate(deserialized->_sizeInBytes());
    deserialized->writeToStream(stream_message->buffer());
 
    context->setRosMessage(stream_message);
}

void BasicMarshallingBackwardProcessor::deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized) {

    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];

        auto matches = serialized.find(sub_template_name);
        
        // Probably the message definition changed. 
        // TODO: 
        if (matches == serialized.end()) {
            continue;
        }

        const bsoncxx::document::element serialized_element = *matches;

        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            
            case ros_babel_fish::MessageTypes::Compound: {
                
                const bsoncxx::document::view sub_serialized = serialized_element.get_document().view();                  
                ros_babel_fish::CompoundMessage& sub_deserialized = deserialized.as<ros_babel_fish::CompoundMessage>()[sub_template_name].as<ros_babel_fish::CompoundMessage>();

                deserialize(sub_template, sub_serialized, sub_deserialized);
                break;
            }

            case ros_babel_fish::MessageTypes::Bool: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<bool>>().setValue(serialized_element.get_bool());
                break;
            }
            
            case ros_babel_fish::MessageTypes::UInt8:{
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint8_t>>().setValue(static_cast<uint8_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint16_t>>().setValue(static_cast<uint16_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint32_t>>().setValue(static_cast<uint32_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint64_t>>().setValue(static_cast<uint64_t>(serialized_element.get_int64()));
                break;
            }

            case ros_babel_fish::MessageTypes::Int8: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int8_t>>().setValue(static_cast<int8_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int16_t>>().setValue(static_cast<int16_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int32_t>>().setValue(static_cast<int32_t>(serialized_element.get_int32()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int64_t>>().setValue(static_cast<int64_t>(serialized_element.get_int64()));
                break;
            }

            case ros_babel_fish::MessageTypes::Float32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<float>>().setValue(static_cast<float>(serialized_element.get_double()));
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<double>>().setValue(static_cast<double>(serialized_element.get_double()));
                break;
            }

            case ros_babel_fish::MessageTypes::String: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<std::string>>().setValue(serialized_element.get_utf8().value.to_string());
                break;
            }

            case ros_babel_fish::MessageTypes::Time: {
                
                uint32_t secs = static_cast<uint32_t>(serialized_element["secs"].get_int32());
                uint32_t nsecs = static_cast<uint32_t>(serialized_element["nsecs"].get_int32());
                
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<ros::Time>>().setValue(ros::Time(secs, nsecs));
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                int32_t secs = static_cast<int32_t>(serialized_element["secs"].get_int32());
                int32_t nsecs = static_cast<int32_t>(serialized_element["nsecs"].get_int32());
                
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<ros::Duration>>().setValue(ros::Duration(secs, nsecs));
                break;   
            }

            case ros_babel_fish::MessageTypes::Array: {
                
                switch ( sub_template->array.element_type ) {
                    case ros_babel_fish::MessageTypes::Bool: {
                       
                        ros_babel_fish::ArrayMessage<bool>& bool_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<bool>>();
                        const bsoncxx::array::view serialized_array = serialized_element.get_array();
                       
                        for (const bsoncxx::array::element& array_element : serialized_array) {
                            bool_array_message.append(static_cast<bool>(array_element.get_bool()));
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        
                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        // szieof(int8_t) is 1. The compiler should optimize this away.
                        const int data_length = wrapper.size / sizeof(uint8_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<uint8_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint8_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint8_t>(data_length, array_message.isFixedSize(), data);

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {
                        
                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(uint16_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<uint16_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint16_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint16_t>(data_length, array_message.isFixedSize(), data);

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(uint32_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<uint32_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint32_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint32_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(uint64_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<uint64_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint64_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint64_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(int8_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<int8_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int8_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int8_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(int16_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<int16_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int16_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int16_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(int32_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<int32_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int32_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int32_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(int64_t);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<int64_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int64_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int64_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(float);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<float>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<float>>(); 
                        array_message = ros_babel_fish::ArrayMessage<float>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {

                        const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                        const int data_length = wrapper.size / sizeof(double);
                        const uint8_t* data = (const uint8_t*) wrapper.bytes;

                        ros_babel_fish::ArrayMessage<double>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<double>>(); 
                        array_message = ros_babel_fish::ArrayMessage<double>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {
                        
                        ros_babel_fish::ArrayMessage<std::string>& string_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<std::string>>(); 
                        const bsoncxx::array::view serialized_array = serialized_element.get_array();
                       
                        for (const bsoncxx::array::element& array_element : serialized_array) {
                            string_array_message.append(array_element.get_utf8().value.to_string()); 
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {

                        ros_babel_fish::ArrayMessage<ros::Time>& time_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<ros::Time>>(); 
                        const bsoncxx::array::view serialized_array = serialized_element.get_array();
                       
                        for (const bsoncxx::array::element& array_element : serialized_array) {
                            
                            const uint32_t secs = static_cast<uint32_t>(array_element["secs"].get_int32());
                            const uint32_t nsecs = static_cast<uint32_t>(array_element["nsecs"].get_int32());
                  
                            time_array_message.append(ros::Time(secs, nsecs));

                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {

                        ros_babel_fish::ArrayMessage<ros::Duration>& time_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<ros::Duration>>(); 
                        const bsoncxx::array::view serialized_array = serialized_element.get_array();
                       
                        for (const bsoncxx::array::element& array_element : serialized_array) {
                         
                            const int32_t secs = static_cast<int32_t>(array_element["secs"].get_int32());
                            const int32_t nsecs = static_cast<int32_t>(array_element["nsecs"].get_int32());
                  
                            time_array_message.append(ros::Duration(secs, nsecs));
                            
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        
                        ros_babel_fish::CompoundArrayMessage& array_message = deserialized[sub_template_name].as<ros_babel_fish::CompoundArrayMessage>(); 
                        const bsoncxx::array::view serialized_array = serialized_element.get_array();
                        
                        size_t idx = 0;

                        for (const bsoncxx::array::element& array_element : serialized_array) {
                            
                            const bsoncxx::document::view serialized_object = array_element.get_document();

                            ros_babel_fish::Message& sub_message = array_message.isFixedSize() ? array_message[idx] : array_message.appendEmpty();
                            // Should only be of compound type as primitives have their respective special case.
                            ros_babel_fish::CompoundMessage& sub_compound = sub_message.as<ros_babel_fish::CompoundMessage>();
    
                            deserialize(sub_template->array.element_template, serialized_object, sub_compound);
                            
                            ++idx;
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Array:
                    case ros_babel_fish::MessageTypes::None:
                        // These don't exist here
                        break;
                }

                break;
            }
            case ros_babel_fish::MessageTypes::None:
                break;
        }
    }
}

}

