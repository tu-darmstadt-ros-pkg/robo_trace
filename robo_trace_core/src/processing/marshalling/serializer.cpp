// Base
#include "robo_trace_core/processing/marshalling/serializer.hpp"
// Std
#include <string>
// Mongo
#include <mongo/bson/bsonobjbuilder.h>
#include <ros/console.h>


namespace robo_trace {

BasicMessageSerializationStage::BasicMessageSerializationStage(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template) 
: ProcessingStage(ProcessingStage::Mode::FORWARD, "basic_serializer"), m_msg_template(msg_template) {
    //
}

BasicMessageSerializationStage::~BasicMessageSerializationStage() {
    //
}

const ros_babel_fish::MessageTemplate::ConstPtr& BasicMessageSerializationStage::getMessageTemplate() const {
    return m_msg_template;
}

void BasicMessageSerializationStage::process(MessageProcessingContext::Ptr& context) {
   
    mongo::BSONObjBuilder builder;
    size_t bytes_read = 0;
    const ros_babel_fish::BabelFishMessage::ConstPtr& msg = context->getMessage()->getIngress();

    size_t stream_length = msg->size();
    const uint8_t* stream = msg->buffer();
     
    serialize(m_msg_template, builder, stream, bytes_read);

    if (bytes_read != stream_length) {
        context->setStatus(MessageProcessingContext::Status::ERROR, "Serializing message stream consumed " +  std::to_string(bytes_read) + "bytes out of " + std::to_string(stream_length) + "bytes.");
    } else {
        context->getMessage()->setSerialized(builder.obj());
    }
}

void BasicMessageSerializationStage::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read) {
    /*
        Ommiting all boundary checks for performance. We'll catch this . :)
        TODO: Maybe insteaf of advacing bytes_read, rather advance the stream itself.
    */
    
    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& sub_template_name = msg_template->compound.names[idx];
  
        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            case ros_babel_fish::MessageTypes::Compound: {
                
                mongo::BufBuilder& sub_buff_builder = builder.subobjStart(sub_template_name);
                mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);

                serialize(sub_template, sub_obj_builder, stream, bytes_read);

                sub_obj_builder.done();
                break;
            }
            case ros_babel_fish::MessageTypes::Bool: {
                
                uint8_t val = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                ++bytes_read;
                
                builder.appendBool(sub_template_name, val != 0);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt8:{
                
                uint8_t value = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                bytes_read += sizeof(uint8_t);

                // TODO: Maybe cast to unisgned here
                builder.append(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {

                uint16_t value = *reinterpret_cast<const uint16_t*>(stream + bytes_read);
                bytes_read += sizeof(uint16_t);

                // TODO: Maybe cast to unisgned here
                builder.append(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {

                uint32_t value = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                bytes_read += sizeof(uint32_t);

                // TODO: Maybe cast to unisgned here
                builder.append(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {

                uint64_t value = *reinterpret_cast<const uint64_t*>(stream + bytes_read);
                bytes_read += sizeof(uint64_t);

                // TODO: Maybe cast to unisgned here
                builder.appendIntOrLL(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::Int8: {

                int8_t value = *reinterpret_cast<const int8_t*>(stream + bytes_read);
                bytes_read += sizeof(int8_t);

                builder.append(sub_template_name, static_cast<int>(value));
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {

                int16_t value = *reinterpret_cast<const int16_t*>(stream + bytes_read);
                bytes_read += sizeof(int16_t);

                builder.append(sub_template_name, static_cast<int>(value));
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {

                int32_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                bytes_read += sizeof(int32_t);

                builder.append(sub_template_name, static_cast<int>(value));
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {

                int64_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                bytes_read += sizeof(int32_t);

                builder.appendIntOrLL(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::Float32: {

                float value = *reinterpret_cast<const float*>(stream + bytes_read);
                bytes_read += sizeof(float);

                // Mongo has no specialization for float32
                builder.append(sub_template_name, static_cast<double>(value));
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {

                double value = *reinterpret_cast<const double*>(stream + bytes_read);
                bytes_read += sizeof(double);

                builder.append(sub_template_name, value);
                break;
            }
            case ros_babel_fish::MessageTypes::String: {

                const uint8_t *begin = stream + bytes_read;
                const uint32_t length = *reinterpret_cast<const uint32_t*>(begin);
                bytes_read += length + sizeof(uint32_t);

                builder.append(sub_template_name, std::string(reinterpret_cast<const char*>(begin + 4), length));
                break;
            }
            case ros_babel_fish::MessageTypes::Time: {
                
                uint32_t secs = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                uint32_t nsecs = *reinterpret_cast<const uint32_t*>(stream + bytes_read + 4);
                bytes_read += 8;

                // Serialize time as a nested object.
                builder.append(sub_template_name, BSON("secs" << secs << "nsecs" << nsecs));
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                int32_t secs = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                int32_t nsecs = *reinterpret_cast<const int32_t*>(stream + bytes_read + sizeof( int32_t ));
                bytes_read += 8;

                builder.append(sub_template_name, BSON("secs" << secs << "nsecs" << nsecs));
                break;
            }
            // TODO: Handle other cases
            case ros_babel_fish::MessageTypes::Array: {

                mongo::BufBuilder& sub_array_buff_builder = builder.subarrayStart(sub_template_name);
                mongo::BSONArrayBuilder sub_array_builder(sub_array_buff_builder);

                ssize_t length = sub_template->array.length;
                
                bool fixed_length = length >= 0;
                //stream += bytes_read;
                
                if (!fixed_length) {     
                    length = *reinterpret_cast<const uint32_t*>(stream);
                    // stream += sizeof( uint32_t );
                    bytes_read += sizeof(uint32_t);
                }
                
                switch ( sub_template->array.element_type ) {
                    case ros_babel_fish::MessageTypes::Bool: {

                        bytes_read += sizeof(uint8_t) * length;

                        for (size_t idx = 0; idx < length; ++idx) {
                            sub_array_builder.appendBool(*(stream + idx) != 0);
                        }

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {

                        size_t array_byte_length = sizeof(uint8_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {

                        size_t array_byte_length = sizeof(uint16_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {

                        size_t array_byte_length = sizeof(uint32_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {

                        size_t array_byte_length = sizeof(uint64_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {

                        size_t array_byte_length = sizeof(int8_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {

                        size_t array_byte_length = sizeof(int16_t) * length;
                        bytes_read += array_byte_length;

                        builder.appendBinData(sub_template_name, array_byte_length, mongo::BinDataType::BinDataGeneral, stream);
            
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {

                        size_t offset = 0;

                        for (ssize_t idx = 0; idx < length; ++idx){

                            uint32_t str_length = *reinterpret_cast<const uint32_t*>(stream + offset);
                            offset += str_length + sizeof(uint32_t);

                            sub_array_builder.append(std::string(reinterpret_cast<const char*>(stream + offset), str_length));

                        }

                        bytes_read += offset;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        for (ssize_t idx = 0; idx < length; ++idx) {

                            mongo::BufBuilder& sub_obj_buff_builder = sub_array_builder.subobjStart();
                            mongo::BSONObjBuilder sub_obj_builder(sub_obj_buff_builder);

                            serialize(sub_template->array.element_template, sub_obj_builder, stream, bytes_read);

                            sub_obj_builder.done();

                        }
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Array:
                    case ros_babel_fish::MessageTypes::None:
                        // These don't exist here
                        break;
                }

                sub_array_builder.done();
                break;
            }
            case ros_babel_fish::MessageTypes::None:
                break;
        }
    }
}


}