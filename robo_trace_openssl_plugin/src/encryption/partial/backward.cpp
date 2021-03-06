// Base
#include "robo_trace_openssl_plugin/encryption/partial/backward.hpp"
// Std
#include <algorithm>
#include <string>
#include <stdexcept>
// OpenSSL
#include <openssl/rand.h>
#include <openssl/rsa.h>
#include <openssl/engine.h>
// Ros
#include <ros/console.h>
// BabaelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/message.h>
#include <ros_babel_fish/messages/array_message.h>
#include <ros_babel_fish/messages/value_message.h>


#define STAGE_LOGGER_NAME "robo_trace_openssl_partial_encryption_backward"

namespace robo_trace::plugin::open_ssl {

PartialEncryptionBackwardProcessor::PartialEncryptionBackwardProcessor(const PartialEncryptionModuleConfiguration::Ptr& configuration, const KeyManager::Ptr& key_manager, const ros_babel_fish::DescriptionProvider::Ptr& message_description_provider, const robo_trace::store::Container::Ptr& metadata) 
: m_configuration(configuration), m_key_manager(key_manager) {
    ROS_INFO_STREAM("Construct.");
    /*
        Initialize EVP related members.
    */

    m_decryption_context = EVP_CIPHER_CTX_new();

    if (m_decryption_context == nullptr) {
        throw std::runtime_error("Could not instantiate new EVP context.");
    }

    /*
        Initialize EVP related members.
    */

    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Using encryption method: " << m_configuration->getEncryptionMethod());
    m_encryption_method = EVP_get_cipherbyname(m_configuration->getEncryptionMethod().c_str());
    
    if (m_encryption_method == nullptr) {
        throw std::runtime_error("Encryption method could not be resolved.");
    }

    /*
        Setup the IV vector.
    */

    int iv_length = EVP_CIPHER_iv_length(m_encryption_method);
    m_iv.resize(iv_length);

    /*
        Fetch the message description.
    */

    std::string message_type = metadata->getString("message_type");

    ROS_DEBUG_STREAM_NAMED(STAGE_LOGGER_NAME, "Fetching template for: " << message_type);
    ros_babel_fish::MessageDescription::ConstPtr message_description = message_description_provider->getMessageDescription(message_type);
    
    if (message_description == nullptr) {
        throw std::runtime_error("No message description for message found!");
    }
    
    m_message_description = message_description;
   
    /*
        Fetch the message description.
    */

    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>& enrcyption_trees = m_configuration->getEncryptionTargetsTree();

    std::string message_type_adjusted = message_type;
    std::replace(message_type_adjusted.begin(), message_type_adjusted.end(), '/', '-');

    std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = enrcyption_trees.find(message_type_adjusted);    

    if (matches == enrcyption_trees.end()) {
        m_encryption_tree = nullptr;
        //throw std::runtime_error("Could not find encryption tree for message.");
    } else {   
        m_encryption_tree = matches->second;
    }

    /*
        Load  the enrcyption key.
    */

    evp_pkey_ctx_ptr pkey_context(EVP_PKEY_CTX_new(m_key_manager->getAuthorityPrivateKey(), nullptr));
    
    if (EVP_PKEY_decrypt_init(pkey_context.get()) <= 0) {
        throw std::runtime_error("Failed initializing pkey encryption context.");
    }

    if (EVP_PKEY_CTX_set_rsa_padding(pkey_context.get(), RSA_PKCS1_OAEP_PADDING) <= 0) {
        throw std::runtime_error("Failed seeting RSA padding.");
    }

    size_t encrypted_key_size;
    unsigned char* encrypted_key = (unsigned char*) metadata->getBinData("key", encrypted_key_size); 

    size_t decrypted_key_size;

    if (EVP_PKEY_decrypt(pkey_context.get(), NULL, &decrypted_key_size, encrypted_key, encrypted_key_size) <= 0) {
        throw std::runtime_error("Failed fetching decrypted key size.");
    }
 
    m_key.resize(decrypted_key_size);
    
    if (EVP_PKEY_decrypt(pkey_context.get(), &m_key[0], &decrypted_key_size, encrypted_key, encrypted_key_size) <= 0) {
        throw std::runtime_error("Failed decrypting key.");
    }

}

PartialEncryptionBackwardProcessor::~PartialEncryptionBackwardProcessor() = default;

robo_trace::processing::Mode PartialEncryptionBackwardProcessor::getMode() const {
    return robo_trace::processing::Mode::REPLAY;
}
 
void PartialEncryptionBackwardProcessor::process(const robo_trace::processing::Context::Ptr& context) {
   
    /*
        Load in the IV.
    */

    size_t iv_length = 0;
    unsigned char* iv_data = (unsigned char*) context->getMetadata()->getBinData("iv", iv_length);

    m_iv.assign(&iv_data[0], &iv_data[iv_length]);

    /*
        Deserialize the message.
    */   

    const std::optional<bsoncxx::document::view>& o_serialized = context->getBsonMessage();

    if (!o_serialized) {
        throw std::runtime_error("Serialized message not present.");
    }

    const bsoncxx::document::view& serialized = o_serialized.value();
    ros_babel_fish::CompoundMessage::Ptr deserialized = std::make_shared<ros_babel_fish::CompoundMessage>(m_message_description->message_template);
    
    deserialize(m_message_description->message_template, serialized, *deserialized, m_encryption_tree);
    
    // Write hierarchical message to the byte stream
    ros_babel_fish::BabelFishMessage::Ptr stream_message(new ros_babel_fish::BabelFishMessage());
    stream_message->morph(m_message_description->md5, m_message_description->datatype, m_message_description->message_definition, "0");
    stream_message->allocate(deserialized->_sizeInBytes());
    deserialized->writeToStream(stream_message->buffer());
    
    context->setRosMessage(stream_message);
   
}

void PartialEncryptionBackwardProcessor::deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
    
    for (size_t idx = 0; idx < msg_template->compound.names.size(); ++idx) {
        
        const ros_babel_fish::MessageTemplate::ConstPtr& sub_template = msg_template->compound.types[idx];
        const std::string& name = msg_template->compound.names[idx];

        auto matches = serialized.find(name);
        
        // Probably the message definition changed. 
        // TODO: 
        if (matches == serialized.end()) {
            continue;
        }

        const bsoncxx::document::element serialized_element = *matches;

        // May need to encrypt this message.
        if (encryption_tree != nullptr &&
            // Is the current sub template a target? Yes? Encrypt!
            std::find(encryption_tree->targets.begin(), encryption_tree->targets.end(), name) != encryption_tree->targets.end()) {
            
            if(!EVP_DecryptInit_ex(m_decryption_context, m_encryption_method, NULL, &m_key[0], &m_iv[0])) {
                throw std::runtime_error("Failed to initialize EVP context!");
            }

            const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();
            const size_t encrypted_legth = wrapper.size;
            const unsigned char* encrypted_data = (const unsigned char*) wrapper.bytes;

            // Trim or grow the decryption buffer to a fitting size.
            m_decryption_buffer.resize(encrypted_legth);
            
            int plain_text_length = 0;

            if(!EVP_DecryptUpdate(m_decryption_context, &m_decryption_buffer[0], &plain_text_length, encrypted_data, encrypted_legth)) {
                throw std::runtime_error("Failed feeding message for decryption.");
            }

            int length = 0;

            if(!EVP_DecryptFinal_ex(m_decryption_context, &m_decryption_buffer[0] + plain_text_length, &length)) {
                throw std::runtime_error("Failed finalizing decryption.");
            }

            size_t decrypted_length = plain_text_length + length;
            size_t bytes_read = 0;

            /*
                Althoug kind of paradox, for in order to serialize the message to a bytestream, we
                can send through ROS, we first must deserialize the bytestream, such that we can 
                construct the complete byte stream...  
            */
            switch (sub_template->type) {
                
                case ros_babel_fish::MessageTypes::Compound: {

                    const ros_babel_fish::CompoundMessage* message = ros_babel_fish::CompoundMessage::fromStream(sub_template, &m_decryption_buffer[0], decrypted_length, bytes_read);
                    
                    deserialized[name].as<ros_babel_fish::CompoundMessage>() = *message;
                    deserialized[name].detachFromStream();

                    delete message;
                    break;
                }

                case ros_babel_fish::MessageTypes::Bool: 
                    deserialized[name].as<ros_babel_fish::ValueMessage<bool>>().setValue(static_cast<bool>(*reinterpret_cast<const uint8_t*>(&m_decryption_buffer[0]) != 0));
                    break;    

                case ros_babel_fish::MessageTypes::UInt8:
                    deserialized[name].as<ros_babel_fish::ValueMessage<uint8_t>>().setValue(*reinterpret_cast<const uint8_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::UInt16:
                    deserialized[name].as<ros_babel_fish::ValueMessage<uint16_t>>().setValue(*reinterpret_cast<const uint16_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::UInt32:
                    deserialized[name].as<ros_babel_fish::ValueMessage<uint32_t>>().setValue(*reinterpret_cast<const uint32_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::UInt64:
                    deserialized[name].as<ros_babel_fish::ValueMessage<uint64_t>>().setValue(*reinterpret_cast<const uint64_t*>(&m_decryption_buffer[0]));
                    break;

                case ros_babel_fish::MessageTypes::Int8:
                    deserialized[name].as<ros_babel_fish::ValueMessage<int8_t>>().setValue(*reinterpret_cast<const int8_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::Int16:
                    deserialized[name].as<ros_babel_fish::ValueMessage<int16_t>>().setValue(*reinterpret_cast<const int16_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::Int32:
                    deserialized[name].as<ros_babel_fish::ValueMessage<int32_t>>().setValue(*reinterpret_cast<const int32_t*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::Int64:
                    deserialized[name].as<ros_babel_fish::ValueMessage<int64_t>>().setValue(*reinterpret_cast<const int64_t*>(&m_decryption_buffer[0]));
                    break;

                case ros_babel_fish::MessageTypes::Float32:
                    deserialized[name].as<ros_babel_fish::ValueMessage<float>>().setValue(*reinterpret_cast<const float*>(&m_decryption_buffer[0]));
                    break;
                case ros_babel_fish::MessageTypes::Float64:
                    deserialized[name].as<ros_babel_fish::ValueMessage<double>>().setValue(*reinterpret_cast<const double*>(&m_decryption_buffer[0]));
                    break;

                case ros_babel_fish::MessageTypes::String: 
                    deserialized[name].as<ros_babel_fish::ValueMessage<std::string>>().setValue(std::string(
                        // Start Address
                        reinterpret_cast<const char*>(&m_decryption_buffer[0] + 4), 
                        // Length
                        *reinterpret_cast<const uint32_t*>(&m_decryption_buffer[0])
                    ));
                    break;

                case ros_babel_fish::MessageTypes::Time:
                    deserialized[name].as<ros_babel_fish::ValueMessage<ros::Time>>().setValue(ros::Time(
                        // Secs
                        *reinterpret_cast<const uint32_t*>(&m_decryption_buffer[0]),
                        // Nsecs
                        *reinterpret_cast<const uint32_t*>(&m_decryption_buffer[0] + 4)
                    ));
                    break;
                case ros_babel_fish::MessageTypes::Duration: 
                    deserialized[name].as<ros_babel_fish::ValueMessage<ros::Duration>>().setValue(ros::Duration(
                        // Secs
                        *reinterpret_cast<const int32_t*>(&m_decryption_buffer[0]),
                        // Nsecs
                        *reinterpret_cast<const int32_t*>(&m_decryption_buffer[0] + 4)
                    ));
                    break;

                case ros_babel_fish::MessageTypes::Array: {
                    
                    ssize_t length = sub_template->array.length;

                    switch (sub_template->array.element_type) {

                        case ros_babel_fish::MessageTypes::Bool:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Bool>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                            
                        case ros_babel_fish::MessageTypes::UInt8:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::UInt8>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::UInt16:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::UInt16>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::UInt32:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::UInt32>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::UInt64:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::UInt64>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;

                        case ros_babel_fish::MessageTypes::Int8:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Int8>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::Int16:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Int16>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::Int32:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Int32>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::Int64:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Int64>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;

                        case ros_babel_fish::MessageTypes::Float32:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Float32>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::Float64:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Float64>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;

                        case ros_babel_fish::MessageTypes::String:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::String>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                            
                        case ros_babel_fish::MessageTypes::Time:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Time>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;
                        case ros_babel_fish::MessageTypes::Duration:
                            deserialized[name] = ros_babel_fish::ArrayMessage<ros_babel_fish::message_type_traits::value_type<ros_babel_fish::MessageTypes::Duration>::value>::fromStream(length, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break;

                        case ros_babel_fish::MessageTypes::Compound:
                            deserialized[name] = ros_babel_fish::CompoundArrayMessage::fromStream(length, sub_template->array.element_template, &m_decryption_buffer[0], decrypted_length, bytes_read);
                            break; 

                        case ros_babel_fish::MessageTypes::Array:
                        case ros_babel_fish::MessageTypes::None:
                            // These don't exist here
                            break;
                    }
                    
                }
                case ros_babel_fish::MessageTypes::None:
                    break;
            }

            
            //====================================

        } else {
            deserialize(sub_template, name, serialized_element, deserialized, encryption_tree);
        }
    }

}

void PartialEncryptionBackwardProcessor::deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const std::string& name, const bsoncxx::document::element& serialized_element, ros_babel_fish::CompoundMessage& deserialized, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr encryption_tree) {
   
    switch (msg_template->type) {
        
        case ros_babel_fish::MessageTypes::Compound: {
            
            PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;

            if (encryption_tree != nullptr) {

                std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                // No targets under this component.
                if (matches != encryption_tree->children.end()) {
                    sub_encryption_tree = matches->second;
                }

            }

            const bsoncxx::document::view sub_serialized = serialized_element.get_document().view();                  
            ros_babel_fish::CompoundMessage& sub_deserialized = deserialized.as<ros_babel_fish::CompoundMessage>()[name].as<ros_babel_fish::CompoundMessage>();

            deserialize(msg_template, sub_serialized, sub_deserialized, sub_encryption_tree);
                
            break;
        }

        case ros_babel_fish::MessageTypes::Bool: {
            deserialized[name].as<ros_babel_fish::ValueMessage<bool>>().setValue(serialized_element.get_bool());
            break;
        }
        
        case ros_babel_fish::MessageTypes::UInt8: {
            deserialized[name].as<ros_babel_fish::ValueMessage<uint8_t>>().setValue(static_cast<uint8_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt16: {
            deserialized[name].as<ros_babel_fish::ValueMessage<uint16_t>>().setValue(static_cast<uint16_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt32: {
            deserialized[name].as<ros_babel_fish::ValueMessage<uint32_t>>().setValue(static_cast<uint32_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::UInt64: {
            deserialized[name].as<ros_babel_fish::ValueMessage<uint64_t>>().setValue(static_cast<uint64_t>(serialized_element.get_int64()));
            break;
        }

        case ros_babel_fish::MessageTypes::Int8: {
            deserialized[name].as<ros_babel_fish::ValueMessage<int8_t>>().setValue(static_cast<int8_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::Int16: {
            deserialized[name].as<ros_babel_fish::ValueMessage<int16_t>>().setValue(static_cast<int16_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::Int32: {
            deserialized[name].as<ros_babel_fish::ValueMessage<int32_t>>().setValue(static_cast<int32_t>(serialized_element.get_int32()));
            break;
        }
        case ros_babel_fish::MessageTypes::Int64: {
            deserialized[name].as<ros_babel_fish::ValueMessage<int64_t>>().setValue(static_cast<int64_t>(serialized_element.get_int64()));
            break;
        }

        case ros_babel_fish::MessageTypes::Float32: {
            deserialized[name].as<ros_babel_fish::ValueMessage<float>>().setValue(static_cast<float>(serialized_element.get_double()));
            break;
        }
        case ros_babel_fish::MessageTypes::Float64: {
            deserialized[name].as<ros_babel_fish::ValueMessage<double>>().setValue(static_cast<double>(serialized_element.get_double()));
            break;
        }

        case ros_babel_fish::MessageTypes::String: {
            deserialized[name].as<ros_babel_fish::ValueMessage<std::string>>().setValue(serialized_element.get_utf8().value.to_string());
            break;
        }

        case ros_babel_fish::MessageTypes::Time: {
            
            uint32_t secs = static_cast<uint32_t>(serialized_element["secs"].get_int32());
            uint32_t nsecs = static_cast<uint32_t>(serialized_element["nsecs"].get_int32());
            
            deserialized[name].as<ros_babel_fish::ValueMessage<ros::Time>>().setValue(ros::Time(secs, nsecs));
            
            break;
        }
        case ros_babel_fish::MessageTypes::Duration: {
            
            int32_t secs = static_cast<int32_t>(serialized_element["secs"].get_int32());
            int32_t nsecs = static_cast<int32_t>(serialized_element["nsecs"].get_int32());
            
            deserialized[name].as<ros_babel_fish::ValueMessage<ros::Duration>>().setValue(ros::Duration(secs, nsecs));
            
            break;   
        }

        // TODO: Handle other cases
        case ros_babel_fish::MessageTypes::Array: {
            
            switch (msg_template->array.element_type) {
                case ros_babel_fish::MessageTypes::Bool: {
                    
                    ros_babel_fish::ArrayMessage<bool>& bool_array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<bool>>();
                    const bsoncxx::array::view serialized_array = serialized_element.get_array();
                    
                    for (const bsoncxx::array::element& array_element : serialized_array) {
                        bool_array_message.append(static_cast<bool>(array_element.get_bool()));
                    }

                    break;
                }
                case ros_babel_fish::MessageTypes::UInt8: {
                    
                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(uint8_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<uint8_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<uint8_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<uint8_t>(data_length, array_message.isFixedSize(), data);

                    break;
                }
                case ros_babel_fish::MessageTypes::UInt16: {
                    
                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(uint16_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<uint16_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<uint16_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<uint16_t>(data_length, array_message.isFixedSize(), data);

                    break;
                }
                case ros_babel_fish::MessageTypes::UInt32: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(uint32_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<uint32_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<uint32_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<uint32_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::UInt64: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(uint64_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<uint64_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<uint64_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<uint64_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Int8: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(int8_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<int8_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<int8_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<int8_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Int16: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(int16_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<int16_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<int16_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<int16_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Int32: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(int32_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<int32_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<int32_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<int32_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Int64: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(int64_t);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<int64_t>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<int64_t>>(); 
                    array_message = ros_babel_fish::ArrayMessage<int64_t>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Float32: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(float);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<float>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<float>>(); 
                    array_message = ros_babel_fish::ArrayMessage<float>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::Float64: {

                    const bsoncxx::types::b_binary wrapper = serialized_element.get_binary();

                    const int data_length = wrapper.size / sizeof(double);
                    const uint8_t* data = (const uint8_t*) wrapper.bytes;

                    ros_babel_fish::ArrayMessage<double>& array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<double>>(); 
                    array_message = ros_babel_fish::ArrayMessage<double>(data_length, array_message.isFixedSize(), data);
                    
                    break;
                }
                case ros_babel_fish::MessageTypes::String: {
                    
                    ros_babel_fish::ArrayMessage<std::string>& string_array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<std::string>>(); 
                    const bsoncxx::array::view serialized_array = serialized_element.get_array();
                    
                    for (const bsoncxx::array::element& array_element : serialized_array) {
                        string_array_message.append(array_element.get_utf8().value.to_string()); 
                    }

                    break;
                }
                case ros_babel_fish::MessageTypes::Time: {
                    
                    ros_babel_fish::ArrayMessage<ros::Time>& time_array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<ros::Time>>(); 
                    const bsoncxx::array::view serialized_array = serialized_element.get_array();
                    
                    for (const bsoncxx::array::element& array_element : serialized_array) {
                        
                        const uint32_t secs = static_cast<uint32_t>(array_element["secs"].get_int32());
                        const uint32_t nsecs = static_cast<uint32_t>(array_element["nsecs"].get_int32());
                
                        time_array_message.append(ros::Time(secs, nsecs));

                    }

                    break;
                }
                case ros_babel_fish::MessageTypes::Duration: {

                    ros_babel_fish::ArrayMessage<ros::Duration>& time_array_message = deserialized[name].as<ros_babel_fish::ArrayMessage<ros::Duration>>(); 
                    const bsoncxx::array::view serialized_array = serialized_element.get_array();
                    
                    for (const bsoncxx::array::element& array_element : serialized_array) {
                        
                        const int32_t secs = static_cast<int32_t>(array_element["secs"].get_int32());
                        const int32_t nsecs = static_cast<int32_t>(array_element["nsecs"].get_int32());
                
                        time_array_message.append(ros::Duration(secs, nsecs));
                        
                    }

                    break;
                }
                case ros_babel_fish::MessageTypes::Compound: {
                    
                    ros_babel_fish::CompoundArrayMessage& array_message = deserialized[name].as<ros_babel_fish::CompoundArrayMessage>(); 
                    const bsoncxx::array::view serialized_array = serialized_element.get_array();
                    
                    size_t idx = 0;

                    for (const bsoncxx::array::element& array_element : serialized_array) {
                        
                        PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr sub_encryption_tree = nullptr;

                        if (encryption_tree != nullptr) {

                            std::unordered_map<std::string, PartialEncryptionModuleConfiguration::EncryptionTarget::Ptr>::const_iterator matches = encryption_tree->children.find(name);

                            // No targets under this component.
                            if (matches != encryption_tree->children.end()) {
                                sub_encryption_tree = matches->second;
                            }

                        }

                        const bsoncxx::document::view serialized_object = array_element.get_document();

                        ros_babel_fish::Message& sub_message = array_message.isFixedSize() ? array_message[idx] : array_message.appendEmpty();
                        // Should only be of compound type as primitives have their respective special case.
                        ros_babel_fish::CompoundMessage& sub_compound = sub_message.as<ros_babel_fish::CompoundMessage>();

                        deserialize(msg_template->array.element_template, serialized_object, sub_compound, sub_encryption_tree);
                        
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