// Base
#include "robo_trace/processing/stage/marshalling/backward.hpp"
// BabelFish
#include <ros_babel_fish/message.h>
#include <ros_babel_fish/messages/value_message.h>
#include <ros_babel_fish/messages/array_message.h>


namespace robo_trace {

BasicMessageMarshallingBackwardStage::BasicMessageMarshallingBackwardStage(const ros_babel_fish::MessageDescription::ConstPtr& message_description) 
: m_message_description(message_description) {
    //
}
  
BasicMessageMarshallingBackwardStage::~BasicMessageMarshallingBackwardStage() {
    // 
}

ProcessingMode BasicMessageMarshallingBackwardStage::getMode() const {
    return ProcessingMode::REPLAY;
}

void BasicMessageMarshallingBackwardStage::process(const ProcessingContext::Ptr& context) {
   
    /*
        Currently the serialized message (in Mongo format) is deserialized into a babel 
        fish hierarchical message. Afterwards this hierarchical message is serialized into
        a bytestream that may be published, etc.

        One could try to skip the babel fish intermediate representation.

    */

    const std::optional<mongo::BSONObj>& o_serialized = context->getSerializedMessage();

    if (!o_serialized) {
        throw std::runtime_error("Message to be deserialized not found.");
    }

    const mongo::BSONObj& serialized = o_serialized.value();
    ros_babel_fish::CompoundMessage::Ptr deserialized = std::make_shared<ros_babel_fish::CompoundMessage>(m_message_description->message_template);
    
    deserialize(m_message_description->message_template, serialized, *deserialized);
    
    // Write hierarchical message to the byte stream
    ros_babel_fish::BabelFishMessage::Ptr stream_message(new ros_babel_fish::BabelFishMessage());
    stream_message->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");
    stream_message->allocate(deserialized->_sizeInBytes());
    deserialized->writeToStream(stream_message->buffer());
 
    context->setUnserializedMessage(stream_message);
}

void BasicMessageMarshallingBackwardStage::deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const mongo::BSONObj& serialized, ros_babel_fish::CompoundMessage& deserialized) {

    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];

        const mongo::BSONElement serialized_element = serialized[sub_template_name];

        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            
            case ros_babel_fish::MessageTypes::Compound: {
                
                const mongo::BSONObj sub_serialized = serialized.getObjectField(sub_template_name);
                ros_babel_fish::CompoundMessage& sub_deserialized = deserialized.as<ros_babel_fish::CompoundMessage>()[sub_template_name].as<ros_babel_fish::CompoundMessage>();

                deserialize(sub_template, sub_serialized, sub_deserialized);
                break;
            }

            case ros_babel_fish::MessageTypes::Bool: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<bool>>().setValue(serialized_element.boolean());
                break;
            }
            
            case ros_babel_fish::MessageTypes::UInt8:{
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint8_t>>().setValue(static_cast<uint8_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint16_t>>().setValue(static_cast<uint16_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint32_t>>().setValue(static_cast<uint32_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<uint64_t>>().setValue(static_cast<uint64_t>(serialized_element._numberLong()));
                break;
            }

            case ros_babel_fish::MessageTypes::Int8: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int8_t>>().setValue(static_cast<int8_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int16_t>>().setValue(static_cast<int16_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int32_t>>().setValue(static_cast<int32_t>(serialized_element._numberInt()));
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<int64_t>>().setValue(static_cast<int64_t>(serialized_element._numberLong()));
                break;
            }

            case ros_babel_fish::MessageTypes::Float32: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<float>>().setValue(static_cast<float>(serialized_element._numberDouble()));
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<double>>().setValue(static_cast<double>(serialized_element._numberDouble()));
                break;
            }

            case ros_babel_fish::MessageTypes::String: {
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<std::string>>().setValue(serialized_element.str());
                break;
            }

            case ros_babel_fish::MessageTypes::Time: {
                
                const mongo::BSONObj serialized_time = serialized.getObjectField(sub_template_name);
                uint32_t secs = static_cast<uint32_t>(serialized_time["secs"]._numberInt());
                uint32_t nsecs = static_cast<uint32_t>(serialized_time["nsecs"]._numberInt());
                
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<ros::Time>>().setValue(ros::Time(secs, nsecs));
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                const mongo::BSONObj serialized_duration = serialized.getObjectField(sub_template_name);
                int32_t secs = static_cast<int32_t>(serialized_duration["secs"]._numberInt());
                int32_t nsecs = static_cast<int32_t>(serialized_duration["nsecs"]._numberInt());
                
                deserialized[sub_template_name].as<ros_babel_fish::ValueMessage<ros::Duration>>().setValue(ros::Duration(secs, nsecs));
                break;   
            }

            case ros_babel_fish::MessageTypes::Array: {
                
                switch ( sub_template->array.element_type ) {
                    case ros_babel_fish::MessageTypes::Bool: {
                        
                        ros_babel_fish::ArrayMessage<bool>& bool_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<bool>>();
                        const mongo::BSONObj serialized_array = serialized.getObjectField(sub_template_name);

                        for(mongo::BSONObj::iterator elem_iterator = serialized_array.begin(); elem_iterator.more(); ) { 
                            bool_array_message.append(static_cast<bool>(elem_iterator.next().boolean()));
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        
                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        // szieof(int8_t) is 1. The compiler should optimize this away.
                        int data_length = data_byte_length / sizeof(uint8_t);

                        ros_babel_fish::ArrayMessage<uint8_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint8_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint8_t>(data_length, array_message.isFixedSize(), data);

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {
                        
                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(uint16_t);

                        ros_babel_fish::ArrayMessage<uint16_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint16_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint16_t>(data_length, array_message.isFixedSize(), data);

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(uint32_t);

                        ros_babel_fish::ArrayMessage<uint32_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint32_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint32_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(uint64_t);

                        ros_babel_fish::ArrayMessage<uint64_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<uint64_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<uint64_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        // szieof(int8_t) is 1. The compiler should optimize this away.
                        int data_length = data_byte_length / sizeof(int8_t);

                        ros_babel_fish::ArrayMessage<int8_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int8_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int8_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(int16_t);

                        ros_babel_fish::ArrayMessage<int16_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int16_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int16_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(int32_t);

                        ros_babel_fish::ArrayMessage<int32_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int32_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int32_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(int64_t);

                        ros_babel_fish::ArrayMessage<int64_t>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<int64_t>>(); 
                        array_message = ros_babel_fish::ArrayMessage<int64_t>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(float);

                        ros_babel_fish::ArrayMessage<float>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<float>>(); 
                        array_message = ros_babel_fish::ArrayMessage<float>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {

                        int data_byte_length;
                        const uint8_t* data = (const uint8_t*) serialized_element.binData(data_byte_length);

                        int data_length = data_byte_length / sizeof(double);

                        ros_babel_fish::ArrayMessage<double>& array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<double>>(); 
                        array_message = ros_babel_fish::ArrayMessage<double>(data_length, array_message.isFixedSize(), data);
                        
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {
                        
                        ros_babel_fish::ArrayMessage<std::string>& string_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<std::string>>(); 
                        const mongo::BSONObj serialized_array = serialized.getObjectField(sub_template_name);

                        for(mongo::BSONObj::iterator elem_iterator = serialized_array.begin(); elem_iterator.more(); ) { 
                            string_array_message.append(elem_iterator.next().str());
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {

                        ros_babel_fish::ArrayMessage<ros::Time>& time_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<ros::Time>>(); 
                        const mongo::BSONObj serialized_array = serialized.getObjectField(sub_template_name);

                        for(mongo::BSONObj::iterator elem_iterator = serialized_array.begin(); elem_iterator.more(); ) { 

                            const mongo::BSONObj serialized_time = elem_iterator.next().Obj();
                            uint32_t secs = static_cast<uint32_t>(serialized_time["secs"]._numberInt());
                            uint32_t nsecs = static_cast<uint32_t>(serialized_time["nsecs"]._numberInt());
                  
                            time_array_message.append(ros::Time(secs, nsecs));

                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {

                        ros_babel_fish::ArrayMessage<ros::Duration>& time_array_message = deserialized[sub_template_name].as<ros_babel_fish::ArrayMessage<ros::Duration>>(); 
                        const mongo::BSONObj serialized_array = serialized.getObjectField(sub_template_name);

                        for(mongo::BSONObj::iterator elem_iterator = serialized_array.begin(); elem_iterator.more(); ) { 

                            const mongo::BSONObj serialized_time = elem_iterator.next().Obj();
                            int32_t secs = static_cast<int32_t>(serialized_time["secs"]._numberInt());
                            int32_t nsecs = static_cast<int32_t>(serialized_time["nsecs"]._numberInt());
                  
                            time_array_message.append(ros::Duration(secs, nsecs));
                            
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        
                        ros_babel_fish::CompoundArrayMessage& array_message = deserialized[sub_template_name].as<ros_babel_fish::CompoundArrayMessage>(); 
                        const mongo::BSONObj serialized_object_array = serialized.getObjectField(sub_template_name);

                        size_t idx = 0;

                        for(mongo::BSONObj::iterator elem_iterator = serialized_object_array.begin(); elem_iterator.more(); ) { 
                            
                            const mongo::BSONObj serialized_object = elem_iterator.next().Obj();
                            
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

