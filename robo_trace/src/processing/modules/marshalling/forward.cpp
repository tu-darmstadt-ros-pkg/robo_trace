// Base
#include "robo_trace/processing/modules/marshalling/forward.hpp"
// Std
#include <string>
// MongoCXX#
#include "bsoncxx/json.hpp"
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
// BabelFish
#include <ros_babel_fish/message_description.h>


namespace robo_trace::processing {

BasicMarshallingForwardProcessor::BasicMarshallingForwardProcessor(const ros_babel_fish::MessageTemplate::ConstPtr message_template)
: m_message_template(message_template) {
    //
}

BasicMarshallingForwardProcessor::~BasicMarshallingForwardProcessor() {
    //
}

Mode BasicMarshallingForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void BasicMarshallingForwardProcessor::process(const Context::Ptr& context) {
    
    const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& o_message = context->getUnserializedMessage();

    if (!o_message) {
        throw std::runtime_error("No message to be serialized provided.");
    }

    const ros_babel_fish::BabelFishMessage::ConstPtr& msg = o_message.value();

    bsoncxx::builder::basic::document builder{};
    size_t bytes_read = 0;
    
    size_t stream_length = msg->size();
    const uint8_t* stream = msg->buffer();
    
    BasicMarshallingForwardProcessor::serialize(m_message_template, builder, stream, bytes_read);

   /* if (bytes_read != stream_length) {
        throw std::runtime_error("Serializing message stream consumed " +  std::to_string(bytes_read) + "bytes out of " + std::to_string(stream_length) + "bytes.");
    } else {
       
    }*/
    
    context->setSerializedMessage(builder.extract());
}

void BasicMarshallingForwardProcessor::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t* stream, size_t& bytes_read) {
    /*
        Ommiting all boundary checks for performance. We'll catch this . :)
        TODO: Maybe insteaf of advacing bytes_read, rather advance the stream itself.
    */
    
    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        
        const std::string& sub_template_name = msg_template->compound.names[idx];
        ROS_INFO_STREAM("Bytes read is: " << bytes_read << " @ " << sub_template_name);

        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            case ros_babel_fish::MessageTypes::Compound: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [&sub_template, &bytes_read, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        BasicMarshallingForwardProcessor::serialize(sub_template, sub_document_builder, stream, bytes_read);
                }));

                break;
            }
            case ros_babel_fish::MessageTypes::Bool: {
                
                uint8_t val = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                ++bytes_read;
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_bool{val != 0}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt8:{
                
                uint8_t value = *reinterpret_cast<const uint8_t*>(stream + bytes_read);
                bytes_read += sizeof(uint8_t);
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {

                uint16_t value = *reinterpret_cast<const uint16_t*>(stream + bytes_read);
                bytes_read += sizeof(uint16_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {

                uint32_t value = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                bytes_read += sizeof(uint32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {

                uint64_t value = *reinterpret_cast<const uint64_t*>(stream + bytes_read);
                bytes_read += sizeof(uint64_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int8: {

                int8_t value = *reinterpret_cast<const int8_t*>(stream + bytes_read);
                bytes_read += sizeof(int8_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {

                int16_t value = *reinterpret_cast<const int16_t*>(stream + bytes_read);
                bytes_read += sizeof(int16_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {

                int32_t value = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                bytes_read += sizeof(int32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {

                int64_t value = *reinterpret_cast<const int64_t*>(stream + bytes_read);
                bytes_read += sizeof(int32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int64{value}));
                break;
            }
            case ros_babel_fish::MessageTypes::Float32: {

                float value = *reinterpret_cast<const float*>(stream + bytes_read);
                bytes_read += sizeof(float);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_double{static_cast<double>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {

                double value = *reinterpret_cast<const double*>(stream + bytes_read);
                bytes_read += sizeof(double);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_double{value}));
                break;
            }
            case ros_babel_fish::MessageTypes::String: {

                const uint8_t *begin = stream + bytes_read;
                const uint32_t length = *reinterpret_cast<const uint32_t*>(begin);
                 ROS_INFO_STREAM("Length: " << length);
                bytes_read += length + sizeof(uint32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, std::string(reinterpret_cast<const char*>(begin + 4), length)));
                break;
            }
            case ros_babel_fish::MessageTypes::Time: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [stream, bytes_read](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        
                        // TODO: Could convert to int32 right away...
                        const uint32_t secs = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{static_cast<int32_t>(secs)}));
                        
                        const uint32_t nsecs = *reinterpret_cast<const uint32_t*>(stream + bytes_read + sizeof(uint32_t));   
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{static_cast<int32_t>(nsecs)}));
                        
                }));

                bytes_read += 2 * sizeof(uint32_t);
                
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [stream, bytes_read](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        
                        int32_t secs = *reinterpret_cast<const int32_t*>(stream + bytes_read);
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{secs}));
                        
                        int32_t nsecs = *reinterpret_cast<const int32_t*>(stream + bytes_read + sizeof(int32_t));   
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{nsecs}));
                        
                }));

                bytes_read += 2 * sizeof(int32_t);

                break;
            }
            case ros_babel_fish::MessageTypes::Array: {

                ssize_t length = sub_template->array.length;        
                bool fixed_length = length >= 0;
                //stream += bytes_read;
                
                if (!fixed_length) {     
                    length = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                    // stream += sizeof( uint32_t );
                    bytes_read += sizeof(uint32_t);
                }
                
                if (length == 0) {
                    continue;
                }
                
                switch ( sub_template->array.element_type) {
                    case ros_babel_fish::MessageTypes::Bool: {
                   
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                            [stream, bytes_read, length](bsoncxx::builder::basic::sub_array array_builder) {
                                for (size_t idx = 0; idx < length; ++idx) {
                                    array_builder.append(bsoncxx::types::b_bool{*(stream + bytes_read + idx) != 0});
                                }
                        })); 

                        bytes_read += sizeof(uint8_t) * length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        
                        const size_t array_byte_length = sizeof(uint8_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
                        
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {
                        
                        const size_t array_byte_length = sizeof(uint16_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;
                       
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
                        
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {
                        
                        size_t array_byte_length = sizeof(uint32_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;
                        
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
              
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {
                        
                        size_t array_byte_length = sizeof(uint64_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
              
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {
                        
                        size_t array_byte_length = sizeof(int8_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
              
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {
                        
                        size_t array_byte_length = sizeof(int16_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));
              
                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {
                        
                        size_t array_byte_length = sizeof(int32_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {
                        
                        size_t array_byte_length = sizeof(int64_t) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {
                        
                        size_t array_byte_length = sizeof(float) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;
                        
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {
                        
                        size_t array_byte_length = sizeof(double) * length;

                        bsoncxx::types::b_binary wrapper;
                        wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
                        wrapper.size = array_byte_length;
                        wrapper.bytes = stream + bytes_read;

                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, wrapper));

                        bytes_read += array_byte_length;
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {
                        
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                            [stream, &bytes_read, length](bsoncxx::builder::basic::sub_array array_builder) {
                                
                                for (size_t idx = 0; idx < length; ++idx){

                                    uint32_t str_length = *reinterpret_cast<const uint32_t*>(stream + bytes_read);
                                    bytes_read += sizeof(uint32_t);

                                    array_builder.append(std::string(reinterpret_cast<const char*>(stream + bytes_read), str_length));
                                  
                                    bytes_read += str_length;

                                }
                                
                        })); 

                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {
                        // TODO:
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {
                        // TODO:
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                            [&sub_template, stream, &bytes_read, length](bsoncxx::builder::basic::sub_array array_builder) {

                                for (size_t idx = 0; idx < length; ++idx) {
                                    array_builder.append([&sub_template, stream, &bytes_read](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        BasicMarshallingForwardProcessor::serialize(sub_template->array.element_template, sub_document_builder, stream, bytes_read);
                                    });
                                }

                            }));
                 
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