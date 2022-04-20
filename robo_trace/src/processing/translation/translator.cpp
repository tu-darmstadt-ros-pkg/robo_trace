/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/translation/translator.hpp"
// Std
#include <string>
// MongoCXX
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
// BabelFish
#include <ros_babel_fish/message.h>
#include <ros_babel_fish/messages/value_message.h>
#include <ros_babel_fish/messages/array_message.h>


namespace robo_trace::processing {

void Translation::upload(bsoncxx::builder::basic::sub_document& builder, const std::string& name, const uint8_t** stream, const size_t length) {

    /*
        Most array types are uploaded as blob.
        TODO: Make this somewhat configurable (i.e. size threshold).
        TODO: Make the destination adjustable (i.e. to GridFS bucket).
    */

    bsoncxx::types::b_binary wrapper;
    wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;    
    wrapper.size = length;
    wrapper.bytes = *stream;

    builder.append(bsoncxx::builder::basic::kvp(name, wrapper));
    
    *stream += length;

}

void Translation::serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream) {

    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];        
        const std::string& sub_template_name = msg_template->compound.names[idx];
       
        // https://github.com/StefanFabian/ros_babel_fish/blob/kinetic/ros_babel_fish/include/ros_babel_fish/generation/message_template.h
        switch (sub_template->type) {
            case ros_babel_fish::MessageTypes::Compound: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [&sub_template, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        Translation::serialize(sub_template, sub_document_builder, stream);
                }));

                break;
            }
            case ros_babel_fish::MessageTypes::Bool: {
                
                const uint8_t val = *reinterpret_cast<const uint8_t*>(*stream);
                *stream += sizeof(uint8_t);
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_bool{val != 0}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt8:{
                
                const uint8_t value = *reinterpret_cast<const uint8_t*>(*stream);
                *stream += sizeof(uint8_t);
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {

                uint16_t value = *reinterpret_cast<const uint16_t*>(*stream);
                *stream += sizeof(uint16_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {

                uint32_t value = *reinterpret_cast<const uint32_t*>(*stream);
                *stream += sizeof(uint32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {

                uint64_t value = *reinterpret_cast<const uint64_t*>(*stream);
                *stream += sizeof(uint64_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int64{static_cast<int64_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int8: {

                int8_t value = *reinterpret_cast<const int8_t*>(*stream);
                *stream += sizeof(int8_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {

                int16_t value = *reinterpret_cast<const int16_t*>(*stream);
                *stream += sizeof(int16_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {

                int32_t value = *reinterpret_cast<const int32_t*>(*stream);
                *stream += sizeof(int32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int32{static_cast<int32_t>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {

                int64_t value = *reinterpret_cast<const int64_t*>(*stream);
                *stream += sizeof(int32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_int64{value}));
                break;
            }
            case ros_babel_fish::MessageTypes::Float32: {

                float value = *reinterpret_cast<const float*>(*stream);
                *stream += sizeof(float);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_double{static_cast<double>(value)}));
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {

                double value = *reinterpret_cast<const double*>(*stream);
                *stream += sizeof(double);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, bsoncxx::types::b_double{value}));
                break;
            }
            case ros_babel_fish::MessageTypes::String: {

                const uint32_t length = *reinterpret_cast<const uint32_t*>(*stream);
                *stream += sizeof(uint32_t);

                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, std::string(reinterpret_cast<const char*>(*stream), length)));
                *stream += length * sizeof(char);

                break;
            }
            case ros_babel_fish::MessageTypes::Time: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        
                        // TODO: Could convert to int32 right away...
                        const uint32_t secs = *reinterpret_cast<const uint32_t*>(*stream);
                        *stream += sizeof(uint32_t);
                        
                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{static_cast<int32_t>(secs)}));
                        
                        const uint32_t nsecs = *reinterpret_cast<const uint32_t*>(*stream); 
                        *stream += sizeof(uint32_t);

                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{static_cast<int32_t>(nsecs)}));
                        
                }));
                
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                
                builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                    [stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                        
                        const int32_t secs = *reinterpret_cast<const int32_t*>(*stream);
                        *stream += sizeof(int32_t);

                        sub_document_builder.append(bsoncxx::builder::basic::kvp("secs", bsoncxx::types::b_int32{secs}));
                        
                        const int32_t nsecs = *reinterpret_cast<const int32_t*>(*stream);  
                        *stream += sizeof(int32_t);

                        sub_document_builder.append(bsoncxx::builder::basic::kvp("nsecs", bsoncxx::types::b_int32{nsecs}));
                        
                }));

                break;
            }
            case ros_babel_fish::MessageTypes::Array: {

                ssize_t length = sub_template->array.length;        
                const bool fixed_length = length >= 0;
                
                if (!fixed_length) {     
                    length = *reinterpret_cast<const uint32_t*>(*stream);
                    *stream += sizeof(uint32_t);
                }
                
                if (length == 0) {
                    continue;
                }
                
                switch ( sub_template->array.element_type) {
                    case ros_babel_fish::MessageTypes::Bool: {
                   
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                            [stream, length](bsoncxx::builder::basic::sub_array array_builder) {
                                for (size_t idx = 0; idx < length; ++idx) {

                                    array_builder.append(bsoncxx::types::b_bool{*stream != 0});
                                    *stream += sizeof(uint8_t);

                                }
                        })); 

                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(uint8_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(uint16_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(uint32_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(uint64_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(int8_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(int16_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(int32_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(int64_t) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(float) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {
                        Translation::upload(builder, sub_template_name, stream, sizeof(double) * length);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {
                        
                        builder.append(bsoncxx::builder::basic::kvp(sub_template_name, 
                            [stream, length](bsoncxx::builder::basic::sub_array array_builder) {
                                
                                for (size_t idx = 0; idx < length; ++idx){

                                    const uint32_t str_length = *reinterpret_cast<const uint32_t*>(*stream);
                                    *stream += sizeof(uint32_t);

                                    array_builder.append(std::string(reinterpret_cast<const char*>(*stream), str_length));
                                    *stream += str_length * sizeof(char);

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
                            [&sub_template, stream, length](bsoncxx::builder::basic::sub_array array_builder) {

                                for (size_t idx = 0; idx < length; ++idx) {
                                    array_builder.append([&sub_template, stream](bsoncxx::builder::basic::sub_document sub_document_builder) {
                                        Translation::serialize(sub_template->array.element_template, sub_document_builder, stream);
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

void Translation::advance(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const uint8_t** stream) {

    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];        
        const std::string& sub_template_name = msg_template->compound.names[idx];
       
        switch (sub_template->type) {
            case ros_babel_fish::MessageTypes::Compound: {
                Translation::advance(sub_template, stream);               
                break;
            }
            case ros_babel_fish::MessageTypes::Bool: {
                *stream += sizeof(uint8_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt8:{
                *stream += sizeof(uint8_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt16: {
                *stream += sizeof(uint16_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt32: {
                *stream += sizeof(uint32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::UInt64: {
                *stream += sizeof(uint64_t);    
                break;
            }
            case ros_babel_fish::MessageTypes::Int8: {
                *stream += sizeof(int8_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int16: {
                *stream += sizeof(int16_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int32: {
                *stream += sizeof(int32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Int64: {
                *stream += sizeof(int32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Float32: {
                *stream += sizeof(float);
                break;
            }
            case ros_babel_fish::MessageTypes::Float64: {
                *stream += sizeof(double);
                break;
            }
            case ros_babel_fish::MessageTypes::String: {

                const uint32_t length = *reinterpret_cast<const uint32_t*>(*stream);
                *stream += sizeof(uint32_t) + length * sizeof(char);

                break;
            }
            case ros_babel_fish::MessageTypes::Time: {
                *stream += 2 * sizeof(uint32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Duration: {
                *stream += 2 * sizeof(int32_t);
                break;
            }
            case ros_babel_fish::MessageTypes::Array: {

                ssize_t length = sub_template->array.length;        
                const bool fixed_length = length >= 0;
                
                if (!fixed_length) {     
                    length = *reinterpret_cast<const uint32_t*>(*stream);
                    *stream += sizeof(uint32_t);
                }
                
                if (length == 0) {
                    continue;
                }
                
                switch ( sub_template->array.element_type) {
                    case ros_babel_fish::MessageTypes::Bool: {
                        *stream += length * sizeof(uint8_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt8: {
                        *stream += length * sizeof(uint8_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt16: {
                        *stream += length * sizeof(uint16_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt32: {
                        *stream += length * sizeof(uint32_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::UInt64: {
                        *stream += length * sizeof(uint64_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int8: {
                        *stream += length * sizeof(int8_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int16: {
                        *stream += length * sizeof(int16_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int32: {
                        *stream += length * sizeof(int32_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Int64: {
                        *stream += length * sizeof(int64_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float32: {
                        *stream += length * sizeof(float);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Float64: {
                        *stream += length * sizeof(double);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::String: {                       
                        for (size_t idx = 0; idx < length; ++idx){

                            const uint32_t str_length = *reinterpret_cast<const uint32_t*>(*stream);
                            *stream += sizeof(uint32_t) + str_length * sizeof(char);
                                
                        }
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Time: {
                        *stream += length * 2 * sizeof(uint32_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Duration: {
                        *stream += length * 2 * sizeof(int32_t);
                        break;
                    }
                    case ros_babel_fish::MessageTypes::Compound: {
                        for (size_t idx = 0; idx < length; ++idx) {
                            Translation::advance(sub_template->array.element_template, stream);
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

void Translation::deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized) {

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

                Translation::deserialize(sub_template, sub_serialized, sub_deserialized);
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
    
                            Translation::deserialize(sub_template->array.element_template, serialized_object, sub_compound);
                            
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
